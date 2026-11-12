TOPLEVEL_LANG = verilog
SIM ?= xcelium

VERILOG_SOURCES += $(PWD)/qlearning_top_v2.v
VERILOG_SOURCES += $(PWD)/uart_tx.v
VERILOG_SOURCES += $(PWD)/uart_rx.v
VERILOG_SOURCES += $(PWD)/maze_env.v
VERILOG_SOURCES += $(PWD)/q_table_ram5.v
VERILOG_SOURCES += $(PWD)/lfsr16.v
VERILOG_SOURCES += $(PWD)/argmax5.v
VERILOG_SOURCES += $(PWD)/ula_bellman.v
VERILOG_SOURCES += $(PWD)/fsm_qlearning_v2.v

TOPLEVEL = qlearning_top_v2
MODULE   = test_fpga_mode

# Baud "virtual" bem mais rápido para não gastar horas simulando 115200 baud real.
# CLKS_PER_BIT=4 já é suficiente para exercitar toda a FSM de uart_tx/uart_rx.
CLKS_PER_BIT ?= 4

# Sintaxe de override de parâmetro muda por simulador:
#   Icarus    -> -P<module>.<param>=<value>          (via COMPILE_ARGS)
#   Xcelium   -> -defparam <module>.<param>=<value>  (via SIM_ARGS)
ifeq ($(SIM),xcelium)
    SIM_ARGS += -defparam qlearning_top_v2.CLKS_PER_BIT=$(CLKS_PER_BIT)
else
    COMPILE_ARGS += -Pqlearning_top_v2.CLKS_PER_BIT=$(CLKS_PER_BIT)
endif

# Se o binário xrun não estiver em PATH, aponte o diretório aqui, por ex.:
# XCELIUM_BIN_DIR := /tools/cadence/XCELIUM2309/tools/bin

include $(shell cocotb-config --makefiles)/Makefile.sim
