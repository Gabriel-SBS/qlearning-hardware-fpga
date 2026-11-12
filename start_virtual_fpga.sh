#!/usr/bin/env bash
# start_virtual_fpga.sh — cria um par de portas seriais virtuais linkadas
# (como um cabo null-modem) usando socat. Um lado (/tmp/vcom0) e usado pelos
# scripts REAIS (generate_maze.py, qlearning_visualizer.py); o outro
# (/tmp/vcom1) e usado pelo test_fpga_hw_bridge.py (cocotb).
#
# IMPORTANTE - ORDEM DE INICIALIZACAO:
#   Este par de PTYs e criado NOVO a cada execucao. A simulacao (cocotb) abre
#   o PTY quando sobe e segura esse file descriptor ate morrer. Se voce
#   reiniciar este script com uma simulacao ANTIGA ainda rodando, os symlinks
#   /tmp/vcom0 e /tmp/vcom1 passam a apontar para um par NOVO, enquanto a
#   simulacao continua grudada no PTY ANTIGO (orfao). Resultado: generate_maze
#   escreve no PTY novo e diz "enviado com sucesso", mas a simulacao NUNCA
#   recebe os bytes ("ainda esperando bytes" eterno) e o visualizer nao recebe
#   nada de volta. Este script MATA socat/simulacao antigos antes de subir um
#   par limpo. Ordem correta:
#       1) ./start_virtual_fpga.sh        (deixe este terminal aberto)
#       2) make -f Makefile.bridge        (2o terminal - a simulacao)
#       3) generate_maze.py / qlearning_visualizer.py  (3o terminal)
#   Se reiniciar as portas, reinicie TAMBEM a simulacao (passo 2).
#
# Requer: socat (sudo apt-get install -y socat)

set -e

VCOM_A=/tmp/vcom0   # lado "hardware"  -- generate_maze.py / qlearning_visualizer.py
VCOM_B=/tmp/vcom1   # lado "simulador" -- test_fpga_hw_bridge.py (cocotb)

cleanup_stale() {
    local killed=0
    if pgrep -f "socat.*link=$VCOM_A" >/dev/null 2>&1; then
        echo "  [!] socat antigo segurando $VCOM_A -- encerrando..."
        pkill -f "socat.*link=$VCOM_A" 2>/dev/null || true
        killed=1
    fi
    for pat in "xmsim.*qlearning" "vvp .*sim.vvp" "test_fpga_hw_bridge"; do
        if pgrep -f "$pat" >/dev/null 2>&1; then
            echo "  [!] simulacao antiga ($pat) presa a um PTY orfao -- encerrando..."
            pkill -f "$pat" 2>/dev/null || true
            killed=1
        fi
    done
    if [ "$killed" = "1" ]; then
        echo "  [i] Sessoes antigas encerradas. Reinicie a simulacao (make -f Makefile.bridge) DEPOIS deste script."
        sleep 0.5
    fi
    rm -f "$VCOM_A" "$VCOM_B" 2>/dev/null || true
}

cleanup_stale

echo "Criando par de portas seriais virtuais:"
echo "  $VCOM_A  <-- scripts reais (generate_maze.py, qlearning_visualizer.py)"
echo "  $VCOM_B  <-- simulacao cocotb (test_fpga_hw_bridge.py)"
echo
echo "Deixe este terminal aberto. Ctrl+C encerra o par de portas."
echo "  >>> Suba a simulacao (make -f Makefile.bridge) SO DEPOIS de ver os PTYs abaixo. <<<"
echo

exec socat -d -d \
    pty,raw,echo=0,link=$VCOM_A \
    pty,raw,echo=0,link=$VCOM_B
