# FPGA virtual no Windows (modo C, hub TCP)

Este é o guia rápido do modo C da seção 5 do [README.md](README.md). A arquitetura, o protocolo UART e os módulos estão explicados lá.

No Windows não há `socat` nem PTY, então o par `/tmp/vcom0`/`/tmp/vcom1` que usamos no Linux virou um hub TCP em localhost, aberto pela própria simulação (`test_fpga_hw_bridge_win.py`, iniciado pelo `run_bridge_win.py`). O pyserial trata `socket://host:porta` como uma porta serial comum, então o `generate_maze.py` e o `qlearning_visualizer.py` funcionam sem nenhuma mudança: basta passar a URL como porta.

Em relação ao par de PTYs, isso tem algumas vantagens. Funciona direto no Windows, sem com0com e sem driver. Cada script abre sua própria conexão TCP; o hub manda a saída do DUT para todos, junta as entradas e repete para os outros o que um cliente envia. É assim que o visualizador fica sabendo do mapa que o `generate_maze.py` mandou, e acabou a briga de dois processos pela mesma fila de entrada de um PTY. Também não sobra PTY nem link simbólico órfão: se a simulação cair, os clientes recebem erro de conexão na hora. E funciona no Linux e no macOS, para quem quiser largar o socat.

## Instalação (uma vez só)

1. Instale o Python 3.10 ou mais novo, marcando "Add python.exe to PATH" no instalador.
2. Instale os pacotes Python num terminal (PowerShell ou cmd):

       pip install cocotb pyserial numpy matplotlib

3. Instale o Icarus Verilog para Windows: baixe o `iverilog-v12-...exe` em https://bleyer.org/icarus/ e marque a opção de adicionar ao PATH (ou adicione `C:\iverilog\bin` à mão). Para conferir:

       iverilog -V

## Como rodar (3 terminais)

No terminal 1, suba a simulação e deixe rodando. Ela compila com o Icarus pelo runner do cocotb e abre o hub TCP:

    python run_bridge_win.py

Espere aparecer `[hub] escutando em socket://127.0.0.1:5555 ...`.

No terminal 2, abra o visualizador:

    python qlearning_visualizer.py --port socket://127.0.0.1:5555

No terminal 3, gere e envie o labirinto. No menu de envio pela UART, informe a porta `socket://127.0.0.1:5555`, ou use a linha de comando:

    python generate_maze.py
    # ou, sem menu:
    python generate_maze.py --seed 42 --send socket://127.0.0.1:5555 --start --no-plot

Tanto faz abrir o terminal 2 ou o 3 primeiro, e os dois podem ser fechados e abertos de novo à vontade: o hub aceita reconexão enquanto a simulação estiver rodando.

## Variáveis de ambiente

    SIM_TCP_HOST      host do hub (padrão 127.0.0.1)
    SIM_TCP_PORT      porta do hub (padrão 5555)
    CLKS_PER_BIT      divisor de baud do RTL simulado (padrão 4, mais rápido)
    BRIDGE_MAX_CYCLES limite de ciclos da simulação (padrão 0 = sem limite)

O `--baud` que se passa aos scripts não faz diferença no TCP, porque só a ordem dos bytes importa. O tempo de cada bit é o do `CLKS_PER_BIT` do DUT simulado, que o `uart_bfm.py` gera bit a bit contra o clock simulado de 50 MHz.

## Observações

* O Icarus é bem mais lento que o Xcelium. Com `CLKS_PER_BIT=4`, a primeira telemetria leva uns 1 a 2 minutos para aparecer, e isso é normal.
* Com a FPGA real, os scripts continuam aceitando portas como `COM3`. No Windows o sistema já dá acesso exclusivo à COM, então as proteções de fila compartilhada nem entram em ação.
* Se precisar de portas COM virtuais de verdade (por exemplo, para testar outro programa do Windows que só entende COM), o driver open-source com0com cria um par COM5↔COM6, que faz o papel do socat. A ponte antiga (`test_fpga_hw_bridge.py`, com `SIM_SERIAL_PORT=COM6`) funcionaria assim, mas com o hub TCP isso não é necessário.
* A compilação vai para `sim_build_win/` (`sim.vvp` e `cmds.f`). Pode apagar essa pasta quando quiser, porque o `run_bridge_win.py` sempre recompila (`always=True`).
