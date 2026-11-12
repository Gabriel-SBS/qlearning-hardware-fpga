"""
run_diag.py
============
Runner Python (cocotb + Icarus, sem Makefile) do teste de diagnóstico
test_diag_uart.py: compila o RTL completo em sim_build_diag/ e compara a
"verdade interna" da FSM (sinais mon_* amostrados a cada mon_valid) com os
pacotes UART decodificados pelo MESMO _decode_episode_packet do visualizador.
Use quando suspeitar de divergência entre a telemetria exibida e o que o
hardware realmente treinou.

Uso:  python run_diag.py            (variáveis: CLKS_PER_BIT, DIAG_EPISODES)
"""

import os
from pathlib import Path
from cocotb_tools.runner import get_runner

PROJ = Path(__file__).resolve().parent
CLKS_PER_BIT = os.environ.get("CLKS_PER_BIT", "4")

SOURCES = ["qlearning_top_v2.v", "uart_tx.v", "uart_rx.v", "maze_env.v",
           "q_table_ram5.v", "lfsr16.v", "argmax5.v", "ula_bellman.v",
           "fsm_qlearning_v2.v"]

runner = get_runner("icarus")
runner.build(
    sources=[PROJ / s for s in SOURCES],
    hdl_toplevel="qlearning_top_v2",
    parameters={"CLKS_PER_BIT": CLKS_PER_BIT},
    build_dir=PROJ / "sim_build_diag",
    timescale=("1ns", "1ps"),
    always=True,
)
runner.test(
    hdl_toplevel="qlearning_top_v2",
    test_module="test_diag_uart",
    build_dir=PROJ / "sim_build_diag",
    test_dir=PROJ,
)
