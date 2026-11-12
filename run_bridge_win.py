"""
run_bridge_win.py — sobe a simulação "FPGA virtual" no Windows (ou Linux)
sem Makefile, sem make e sem socat, usando o runner Python do cocotb com o
Icarus Verilog.

Uso (num terminal; deixe rodando):
    python run_bridge_win.py

Depois, em outros terminais:
    python qlearning_visualizer.py --port socket://127.0.0.1:5555
    python generate_maze.py        (porta UART: socket://127.0.0.1:5555)

Requisitos (uma vez só):
    pip install cocotb pyserial numpy matplotlib
    Icarus Verilog para Windows (instalador "iverilog-v12" — bleyer.org/icarus)
    e garanta que a pasta bin do Icarus esteja no PATH (ex.: C:\\iverilog\\bin).
"""

import os
import sys
from pathlib import Path

from cocotb_tools.runner import get_runner

PROJ = Path(__file__).resolve().parent
CLKS_PER_BIT = os.environ.get("CLKS_PER_BIT", "4")

SOURCES = [
    "qlearning_top_v2.v",
    "uart_tx.v",
    "uart_rx.v",
    "maze_env.v",
    "q_table_ram5.v",
    "lfsr16.v",
    "argmax5.v",
    "ula_bellman.v",
    "fsm_qlearning_v2.v",
]


def main() -> None:
    faltando = [s for s in SOURCES if not (PROJ / s).exists()]
    if faltando:
        sys.exit(f"[erro] fontes Verilog não encontradas: {faltando}")

    runner = get_runner("icarus")
    runner.build(
        sources=[PROJ / s for s in SOURCES],
        hdl_toplevel="qlearning_top_v2",
        parameters={"CLKS_PER_BIT": CLKS_PER_BIT},
        build_dir=PROJ / "sim_build_win",
        timescale=("1ns", "1ps"),  # os fontes não têm `timescale; sem isso o
                                   # Icarus assume precisão de 1 s e o Clock
                                   # de 20 ns do cocotb falha ao ser criado
        always=True,
    )
    print(f"[run_bridge_win] compilado OK (CLKS_PER_BIT={CLKS_PER_BIT}). "
          f"Subindo a ponte TCP ...", flush=True)
    runner.test(
        hdl_toplevel="qlearning_top_v2",
        test_module="test_fpga_hw_bridge_win",
        build_dir=PROJ / "sim_build_win",
    )


if __name__ == "__main__":
    main()
