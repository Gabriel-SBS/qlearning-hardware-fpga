# FPGA virtual no Windows — co-simulação via hub TCP (modo C)

> Guia rápido do **modo C** descrito no `README.md` (seção 5). Leia o README
> principal para a arquitetura, o protocolo UART e a descrição dos módulos.

No Windows não existe `socat` nem PTY, então o par `/tmp/vcom0`/`/tmp/vcom1`
do Linux foi substituído por um **hub TCP em localhost** embutido na própria
simulação (`test_fpga_hw_bridge_win.py`, iniciado por `run_bridge_win.py`).
O pyserial trata `socket://host:porta` como uma porta serial comum, então
`generate_maze.py` e `qlearning_visualizer.py` funcionam **sem nenhuma
mudança** — basta apontar a "porta" para a URL.

Vantagens sobre o par de PTYs:

* Funciona no Windows nativamente (sem com0com, sem driver).
* Cada script tem sua própria conexão TCP: o hub replica a saída do DUT para
  todos, mescla as entradas e ecoa o que um cliente envia para os outros —
  é assim que o visualizador "escuta" o upload de mapa do `generate_maze.py`.
  Acabou a disputa de dois processos pela mesma fila de entrada de um PTY.
* Sem PTY/symlink órfão: se a simulação morrer, os clientes recebem erro de
  conexão explícito na hora.
* Também funciona no Linux/macOS, se você preferir aposentar o socat.

## Instalação (uma vez só)

1. **Python 3.10+** (marque "Add python.exe to PATH" no instalador).
2. **Pacotes Python**, num terminal (PowerShell ou cmd):

       pip install cocotb pyserial numpy matplotlib

3. **Icarus Verilog para Windows**: baixe o instalador `iverilog-v12-...exe`
   em https://bleyer.org/icarus/ e instale marcando a opção de adicionar ao
   PATH (ou adicione `C:\iverilog\bin` manualmente). Confirme com:

       iverilog -V

## Como rodar (3 terminais)

**Terminal 1** — a simulação (compila com Icarus via runner do cocotb e sobe
o hub TCP; deixe rodando):

    python run_bridge_win.py

Espere aparecer:  `[hub] escutando em socket://127.0.0.1:5555 ...`

**Terminal 2** — o visualizador:

    python qlearning_visualizer.py --port socket://127.0.0.1:5555

**Terminal 3** — gerar e enviar o labirinto (no menu de envio UART, informe a
porta como `socket://127.0.0.1:5555`; ou use a CLI direto):

    python generate_maze.py
    # ou, sem menu:
    python generate_maze.py --seed 42 --send socket://127.0.0.1:5555 --start --no-plot

A ordem entre os terminais 2 e 3 não importa, e ambos podem ser fechados e
reabertos à vontade — o hub aceita reconexões enquanto a simulação viver.

## Variáveis de ambiente úteis

    SIM_TCP_HOST      host do hub (padrão 127.0.0.1)
    SIM_TCP_PORT      porta do hub (padrão 5555)
    CLKS_PER_BIT      divisor de baud do RTL simulado (padrão 4, mais rápido)
    BRIDGE_MAX_CYCLES limite de ciclos da simulação (padrão 0 = infinito)

O `--baud` passado aos scripts é irrelevante num transporte TCP — só a ordem
dos bytes importa; o timing de bits real é o do `CLKS_PER_BIT` do DUT
simulado, bit-bangado pelo `uart_bfm.py` contra o clock de 50 MHz simulado.

## Observações

* **Velocidade**: o Icarus é bem mais lento que o Xcelium. Com
  `CLKS_PER_BIT=4`, a primeira telemetria aparece em ~1–2 minutos de relógio
  de parede — normal.
* **FPGA real em casa**: os scripts continuam aceitando portas nativas
  (`COM3` etc.); no Windows o SO já garante acesso exclusivo à COM, então as
  proteções de fila compartilhada nem são necessárias nesse caso.
* **Portas COM virtuais de verdade**: se precisar de COMx reais (ex.: testar
  outro software Windows que só entende COM), o driver open-source com0com
  cria um par COM5↔COM6 equivalente ao socat; a ponte antiga
  (`test_fpga_hw_bridge.py`, com `SIM_SERIAL_PORT=COM6`) funcionaria nesse
  arranjo. O hub TCP dispensa isso.
* **Artefatos de build**: a compilação vai para `sim_build_win/`
  (`sim.vvp` + `cmds.f`); pode apagar a pasta à vontade — `run_bridge_win.py`
  recompila sempre (`always=True`).
