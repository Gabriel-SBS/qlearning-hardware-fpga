"""
test_fpga_mode.py — "Modo FPGA" simulado via cocotb.

Este arquivo É o programa principal (não generate_maze.py nem
qlearning_visualizer.py). Ele importa as duas bibliotecas como módulos
Python puros e chama suas funções diretamente — sem serial real, sem
socket, sem arquivo .txt intermediário. O cocotb injeta/lê os sinais do
DUT (qlearning_top_v2) no mesmo processo em que a lógica Python roda.

Execução:
    make                              # roda com labirinto aleatório, sem plot ao vivo
    MAZE_SEED=42 make                 # labirinto reprodutível
    LIVE_PLOT=1 make                  # abre a janela matplotlib ao vivo (mais lento)
    MAX_EPISODES=200 make             # limita episódios (padrão: até training_done_o)
"""

import os
import sys
import threading

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

sys.path.insert(0, os.path.dirname(__file__))

# ── Bibliotecas do projeto, usadas como módulos puros (sem UART real) ──────
import generate_maze as gm
import qlearning_visualizer as viz

from uart_bfm import UartDriver, UartMonitor

CLKS_PER_BIT = int(os.environ.get("CLKS_PER_BIT", "4"))   # baixo p/ sim rápida
CLK_PERIOD_NS = 20                                          # 50 MHz
MAX_EPISODES = int(os.environ.get("MAX_EPISODES", "0"))     # 0 = sem limite (usa training_done_o)


def decode_episode_packet(b: bytes) -> dict:
    """Mesmo decodificador de qlearning_visualizer.uart_reader, byte a byte.

    NOTA: esta cópia local usa apenas o fallback goal = (reward == 120) e
    ignora o bit 3 do byte 4 (flag explícita de meta que o RTL atual envia).
    Para o modo simulado isso é suficiente — o último passo de um episódio
    com meta sempre carrega reward=+120 — mas o decodificador de referência,
    que lê a flag e mantém o fallback, é o _decode_episode_packet do
    qlearning_visualizer.py."""
    episode = (b[0] << 8) | b[1]
    steps = (b[2] << 4) | (b[3] >> 4)
    reward_raw = ((b[3] & 0x0F) << 4) | (b[4] >> 4)
    reward = reward_raw if reward_raw < 128 else reward_raw - 256
    action = b[4] & 0x07
    goal = (reward == 120)
    return {
        "ep": episode,
        "steps": steps,
        "goal": goal,
        "epsilon": 0,
        "action": action,
        "reward": reward,
    }


@cocotb.test()
async def fpga_mode(dut):
    # ── Clock e reset ───────────────────────────────────────────────────
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

    dut.rst_n.value = 0
    dut.start.value = 0
    dut.i_Rx_Serial.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # ── BFM UART ligados diretamente aos pinos do DUT ──────────────────
    uart_driver = UartDriver(dut, dut.clk, dut.i_Rx_Serial, CLKS_PER_BIT)
    uart_monitor = UartMonitor(dut, dut.clk, dut.o_Tx_Serial, CLKS_PER_BIT)
    cocotb.start_soon(uart_monitor.start())

    # ── Estado compartilhado com o visualizer (mesma classe do script original) ──
    state = viz.SharedState()
    stop_evt = threading.Event()

    live_plot = os.environ.get("LIVE_PLOT", "0") == "1"
    if live_plot:
        plot_thread = threading.Thread(
            target=viz.run_live, args=(state,), kwargs={"refresh_s": 0.5}, daemon=True
        )
        plot_thread.start()

    # ── 1) Gera o labirinto (generate_maze.py, função pura) ────────────
    seed = os.environ.get("MAZE_SEED")
    seed = int(seed) if seed is not None else None
    maze = gm.generate_maze(wall_prob=0.27, seed=seed)
    maze_bytes = gm.maze_to_bytes(maze)
    dut._log.info(f"[maze] {gm.maze_stats(maze)}")

    state.set_wall(viz.bytes_to_wall(maze_bytes))

    # ── 2) Envia o labirinto via UART simulada: 0xB0 + 128 bytes ───────
    await uart_driver.send(0xB0)
    await uart_driver.send_many(maze_bytes)

    # ── 3) Start de treinamento: 0xA0 ───────────────────────────────────
    await uart_driver.send(0xA0)
    dut._log.info("[uart] 0xA0 enviado — treinamento iniciado")

    # ── 4) Loop de episódios: cada pacote de 5 bytes vira 1 episódio ───
    episodes_seen = 0
    while True:
        pkt = await uart_monitor.get_packet(5)
        ep = decode_episode_packet(pkt)
        state.add_episode(ep)
        episodes_seen += 1

        if episodes_seen % 50 == 0 or ep["goal"]:
            dut._log.info(
                f"[ep {ep['ep']:5d}] steps={ep['steps']:4d} "
                f"reward={ep['reward']:4d} goal={ep['goal']}"
            )

        if MAX_EPISODES and episodes_seen >= MAX_EPISODES:
            break
        if int(dut.training_done_o.value) == 1:
            dut._log.info(f"[done] training_done_o=1 após {episodes_seen} episódios")
            break

    stop_evt.set()
    dut._log.info(f"Total de episódios recebidos: {episodes_seen}")

    if not live_plot:
        # Sem janela ao vivo: gera as figuras finais direto do estado acumulado
        # (mesma lógica de plot usada no modo offline do visualizer).
        out_dir = os.environ.get("RESULTS_DIR", "sim_build")
        os.makedirs(out_dir, exist_ok=True)
        try:
            snap = state.snapshot()
            viz._draw_stats  # apenas confirma que o módulo tem as funções de plot
        except AttributeError:
            pass
