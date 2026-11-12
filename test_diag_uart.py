"""
test_diag_uart.py — diagnóstico: verdade interna da FSM vs pacotes UART
decodificados com o MESMO _decode_episode_packet do visualizador.
"""
import os, sys
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly

sys.path.insert(0, os.path.dirname(__file__))
from uart_bfm import UartDriver, UartMonitor
from qlearning_visualizer import _decode_episode_packet

import numpy as np

CLKS_PER_BIT = int(os.environ.get("CLKS_PER_BIT", "4"))
N_EPISODES   = int(os.environ.get("DIAG_EPISODES", "400"))


def maze_to_bytes(maze):
    bits = [maze[i // 32, i % 32] for i in range(1024)]
    out = bytearray(128)
    for bi in range(128):
        v = 0
        for b in range(8):
            v |= (int(bits[bi * 8 + b]) << b)
        out[bi] = v
    return bytes(out)


@cocotb.test()
async def diag(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.i_Rx_Serial.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    drv = UartDriver(dut, dut.clk, dut.i_Rx_Serial, CLKS_PER_BIT)
    mon = UartMonitor(dut, dut.clk, dut.o_Tx_Serial, CLKS_PER_BIT)

    tx_bytes = bytearray()
    mon.on_byte = lambda b: tx_bytes.append(b)
    cocotb.start_soon(mon.start())

    # ── verdade interna: amostra mon_* a cada mon_valid ──────────────────
    truth = []

    async def watch_truth():
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.fsm_inst.mon_valid.value) == 1:
                ep = int(dut.fsm_inst.mon_episode.value)
                st = int(dut.fsm_inst.mon_steps.value)
                rw = int(dut.fsm_inst.mon_reward.value)
                if rw >= 128:
                    rw -= 256
                truth.append({"ep": ep, "steps": st, "reward": rw,
                              "goal": rw == 120})

    cocotb.start_soon(watch_truth())

    # ── programa o mapa (opcional: DIAG_MAZE=default usa WALL_MAP_DEFAULT) ─
    maze_sel = os.environ.get("DIAG_MAZE", "labirinto_bom.npy")
    if maze_sel != "default":
        maze = np.load(os.path.join(os.path.dirname(__file__), maze_sel)).astype(np.uint8)
        maze[0, 0] = 0
        maze[31, 31] = 0
        await drv.send(0xB0)
        for b in maze_to_bytes(maze):
            await drv.send(b)
        await ClockCycles(dut.clk, 20)
    dut._log.info(f"map_ready = {int(dut.map_ready.value)} (maze={maze_sel})")

    # ── start ─────────────────────────────────────────────────────────────
    await drv.send(0xA0)

    # ── roda até N_EPISODES na verdade interna ────────────────────────────
    guard = 0
    while len(truth) < N_EPISODES and guard < 20000:
        await ClockCycles(dut.clk, 500)
        guard += 1
    await ClockCycles(dut.clk, 2000)  # drena TX pendente

    dut._log.info(f"verdade interna: {len(truth)} episódios; "
                  f"UART: {len(tx_bytes)} bytes ({len(tx_bytes)/5:.1f} pacotes)")


    # ── decodifica UART com o decoder do visualizador ─────────────────────
    decoded = []
    buf = bytearray(tx_bytes)
    flag_ok = flag_bad = 0
    while len(buf) >= 5:
        b = buf[:5]
        del buf[:5]
        d = _decode_episode_packet(b)
        d["flag_bit"] = bool(b[4] & 0x08)
        decoded.append(d)

    # ── relatório ─────────────────────────────────────────────────────────
    t_goals = sum(1 for t in truth if t["goal"])
    d_goals = sum(1 for d in decoded if d["goal"])
    print("=" * 72)
    print(f"VERDADE FSM : {len(truth)} eps, metas={t_goals} "
          f"({100*t_goals/max(1,len(truth)):.1f}%), "
          f"steps médios={np.mean([t['steps'] for t in truth]):.1f}")
    print(f"UART decode : {len(decoded)} pacotes, metas={d_goals} "
          f"({100*d_goals/max(1,len(decoded)):.1f}%), "
          f"steps médios={np.mean([d['steps'] for d in decoded]):.1f}"
          if decoded else "UART decode : ZERO pacotes")
    print("=" * 72)

    # verdade indexada por episódio
    truth_by_ep = {t["ep"]: t for t in truth}
    mism = 0
    for d in decoded[:120]:
        t = truth_by_ep.get(d["ep"])
        flag = ""
        if t is None:
            flag = "  << EP INEXISTENTE NA VERDADE"
            mism += 1
        elif (t["steps"] != d["steps"] or t["reward"] != d["reward"]):
            flag = f"  << DIVERGE (verdade: steps={t['steps']} rew={t['reward']})"
            mism += 1
        print(f"pkt ep={d['ep']:5d} steps={d['steps']:4d} rew={d['reward']:4d} "
              f"goal={int(d['goal'])} act={d['action']}{flag}")
    print(f"--- mismatches nos primeiros 120 pacotes: {mism}")
    print(f"--- primeiros 40 bytes crus: {bytes(tx_bytes[:40]).hex(' ')}")

    # taxa de captura: quantos episódios da verdade têm pacote?
    dec_eps = {d["ep"] for d in decoded}
    captured = sum(1 for t in truth if t["ep"] in dec_eps)
    print(f"--- episódios com pacote UART: {captured}/{len(truth)}")
    cap_goal = sum(1 for t in truth if t["ep"] in dec_eps and t["goal"])
    print(f"--- metas entre os capturados: {cap_goal}")

    # curva de sucesso por milhar (verdade vs decodificado)
    for lo in range(0, len(truth), 1000):
        hi = min(lo + 1000, len(truth))
        tt = [t for t in truth if lo <= t["ep"] < hi]
        dd = [d for d in decoded if lo <= d["ep"] < hi]
        tg = sum(1 for t in tt if t["goal"]); dg = sum(1 for d in dd if d["goal"])
        ts = np.mean([t['steps'] for t in tt]) if tt else 0
        ds = np.mean([d['steps'] for d in dd]) if dd else 0
        print(f"eps {lo:4d}-{hi:4d}: VERDADE {tg:4d}/{len(tt):4d} metas "
              f"(steps méd {ts:6.1f})  |  UART {dg:4d}/{len(dd):4d} metas "
              f"(steps méd {ds:6.1f})")

    # viés de drop: entre eps da verdade SEM pacote, qual % era meta?
    dropped = [t for t in truth if t["ep"] not in dec_eps]
    if dropped:
        dg = sum(1 for t in dropped if t["goal"])
        print(f"--- drops: {len(dropped)} eps sem pacote; {dg} eram meta "
              f"({100*dg/len(dropped):.1f}%)")

    # verificação do bit de flag de meta (novo protocolo)
    for d in decoded:
        t = truth_by_ep.get(d["ep"])
        if t is not None:
            if d["flag_bit"] == t["goal"]:
                flag_ok += 1
            else:
                flag_bad += 1
    print(f"--- flag de meta (bit 3 do byte 4): {flag_ok} ok, {flag_bad} errados")

    # ── teste de reinício a partir de S_DONE (novo comportamento) ─────────
    if len(truth) >= 5000:
        await ClockCycles(dut.clk, 100)
        td = int(dut.training_done_o.value)
        n_before = len(truth)
        pk_before = len(tx_bytes)
        await drv.send(0xA0)          # start via UART com FSM em S_DONE
        await ClockCycles(dut.clk, 60000)  # ~algumas dezenas de episódios
        n_new = len(truth) - n_before
        new_eps = [t["ep"] for t in truth[n_before:n_before+5]]
        print(f"### RESTART: training_done={td} antes do 0xA0 | "
              f"novos episódios após restart: {n_new} | "
              f"primeiros eps: {new_eps} | "
              f"novos bytes UART: {len(tx_bytes)-pk_before}")