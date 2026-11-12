# Q-Learning 32×32 em FPGA

Projeto feito no Programa CI Digital (CEPEDI). Um agente de Q-Learning tabular que aprende a sair de um labirinto 32×32 rodando inteiro em hardware, descrito em Verilog. A FSM tem 5 estados e faz um passo de treinamento por ciclo de clock. O mapa e o comando de início chegam pela UART, e no fim de cada episódio o hardware devolve pela mesma UART um pacote de telemetria.

Junto com o RTL há ferramentas em Python para gerar labirintos e acompanhar o treino ao vivo, e três jeitos de co-simular tudo com cocotb, sem precisar da placa. Um deles funciona no Windows.

**Autores:** Gabriel Sena Barbosa Silva ([@Gabriel-SBS](https://github.com/Gabriel-SBS)) e João Vítor Cedraz Carneiro ([@JvCedrazC](https://github.com/JvCedrazC)).

## Sumário

1. [Arquitetura](#1-arquitetura)
2. [Protocolo UART](#2-protocolo-uart)
3. [Módulos RTL](#3-módulos-rtl)
4. [Ferramentas Python](#4-ferramentas-python)
5. [Modos de co-simulação](#5-modos-de-co-simulação)
6. [Testes](#6-testes)
7. [Bugs que encontramos e corrigimos](#7-bugs-que-encontramos-e-corrigimos)
8. [Arquivos](#8-arquivos)
9. [Limitações](#9-limitações)

## 1. Arquitetura

### Hardware (qlearning_top_v2)

```
                          ┌────────────────────────────────────────────────┐
 i_Rx_Serial ──▶ uart_rx ┤  Máquina de comandos (RX_CMD / RX_MAP)         │
                          │   0xA0 → start   0xFF → soft reset             │
                          │   0xB0 → próximos 128 bytes = mapa → prog_*    │
                          └───────────────┬────────────────────────────────┘
                                          ▼ prog_we / addr / data
      ┌──────────────┐  state/action ┌──────────┐ next_state/reward/done
      │ fsm_qlearning│──────────────▶│ maze_env │────────────────────────┐
      │     _v2      │◀──────────────│ (32×32,  │                        │
      │  (5 estados) │               │ mapa dinâmico)                    │
      └──┬───────┬───┘               └──────────┘                        │
         │       │ rand_byte/rand_high ◀── lfsr16 (PRNG ε-greedy)        │
         │       │                                                       │
         │       ▼ addr / addr_n / we / wr_action / din                  │
         │  ┌──────────────┐  Q(s,0..4) + Q(s',0..4) no MESMO ciclo      │
         │  │ q_table_ram5 │──▶ argmax5 ×2 ──▶ ula_bellman ──▶ din ──────┘
         │  │ (5 bancos)   │      (dentro da FSM, tudo combinacional)
         │  └──────────────┘
         │
         ▼ mon_valid / mon_episode / mon_steps / mon_reward / mon_goal / mon_action
   Serializador TX (5 bytes/episódio) ──▶ uart_tx ──▶ o_Tx_Serial
```

Todo o passo de treinamento acontece num único ciclo, no estado `S_STEP`, e é combinacional: ler Q(s,·), escolher a ação (ε-greedy), consultar o ambiente, ler Q(s',·), tirar o argmax e aplicar a atualização de Bellman. O `maze_env` fica no top-level e não dentro da FSM, porque a máquina de recepção da UART precisa da porta de programação do mapa.

### Software

```
generate_maze.py ──0xFF/0xB0+128B/0xA0──▶ ┌──────────────────────────┐
                                          │  FPGA real (COM/ttyUSB)  │
qlearning_visualizer.py ◀──5 B/episódio── │  OU simulação cocotb via │
   (3 painéis + time-travel)              │  PTY socat / hub TCP     │
                                          └──────────────────────────┘
```

O `generate_maze.py` e o `qlearning_visualizer.py` são os mesmos contra a FPGA real e contra a simulação. Só muda a porta: `COM3`, `/dev/ttyUSB0`, `/tmp/vcom0` ou `socket://127.0.0.1:5555`.

## 2. Protocolo UART

UART 8N1 a 115200 baud por padrão (`CLKS_PER_BIT = 434` com clock de 50 MHz; o parâmetro pode ser mudado).

### RX: comandos para o hardware

Cada comando é 1 byte, menos o upload de mapa.

| Byte  | Efeito |
|-------|--------|
| `0xA0` | Start: inicia ou reinicia o treinamento, se `map_ready = 1`. Em `S_DONE`, reinicia sem reset: mantém o mapa e a Q-table e zera os contadores e o epsilon. |
| `0xFF` | Soft reset global (8 ciclos). Volta para o `WALL_MAP_DEFAULT`, então o mapa precisa ser reenviado depois. |
| `0xB0` | Upload de mapa: os 128 bytes seguintes são o WALL_MAP (byte 0 = bits[7:0] … byte 127 = bits[1023:1016]; o bit N é a célula (N÷32, N mod 32)). `map_ready` sobe depois do 128º byte. |
| outros | Ignorados. |

### TX: telemetria

São 5 bytes no fim de cada episódio, tenha ele chegado à meta ou estourado o limite de passos.

| Byte | Conteúdo |
|------|----------|
| 0 | `episode[15:8]` |
| 1 | `episode[7:0]` |
| 2 | `steps[11:4]` |
| 3 | `{steps[3:0], reward[7:4]}` |
| 4 | `{reward[3:0], goal, action[2:0]}`; o bit 3 (`goal`) diz se a meta foi atingida |

O envio é disparado por `mon_valid`, um pulso em `S_NEW_EP` a cada fim de episódio. A meta vem de `mon_goal`, que fica ligado até o episódio fechar. A [seção 7](#7-bugs-que-encontramos-e-corrigimos) conta por que chegamos a esses dois sinais.

## 3. Módulos RTL

### `qlearning_top_v2.v` (top-level)
Junta tudo: UART RX e TX, a máquina de comandos (start, soft reset e upload do mapa em 128 bytes), o serializador da telemetria de 5 bytes, o `maze_env` com mapa dinâmico e o núcleo de Q-Learning (`fsm_qlearning_v2`, `q_table_ram5` e `lfsr16`). Tem duas detecções de borda importantes: `ep_done_pulse` (igual a `mon_valid`), que dispara o pacote, e `tx_done_pulse`, que avança o serializador, porque o `o_Tx_Done` do `uart_tx` dura 2 ciclos. O parâmetro é `CLKS_PER_BIT`. As saídas de status são `led_ep` (4 bits menos significativos do episódio) e `training_done_o`.

### `fsm_qlearning_v2.v` (FSM e datapath do agente)
A FSM tem 5 estados (`S_IDLE → S_INIT_EP → S_STEP ⟲ → S_NEW_EP → S_DONE`). A primeira versão tinha 23, e esta faz cerca de 11 vezes mais passos por segundo, um por ciclo. Ela instancia dois `argmax5`, um para Q(s,·) e outro para Q(s',·), e a `ula_bellman`. Dentro de `S_STEP`, a cadeia leitura → ε-greedy → ambiente → leitura de Q(s',·) → argmax → Bellman → escrita é toda combinacional. O preço é um caminho combinacional longo, que reduz o Fmax; o cabeçalho do arquivo descreve um pipeline de 2 estágios para o caso de a síntese não fechar timing.

Parâmetros: `MAX_STEPS`, `MAX_EPISODES` e `EPSILON_INIT/MIN/DECAY/DECAY_START`. O epsilon cai 1 a cada `EPSILON_DECAY` episódios a partir de `EPSILON_DECAY_START`, até chegar a `EPSILON_MIN`. Telemetria: `mon_valid` (pulso em todo fim de episódio), `mon_goal` (fica ligado) e `mon_state/action/reward/episode/steps`. O ambiente fica fora da FSM (portas `env_*`, `cur_state_o` e `cur_action_o`) para o mapa poder ser reprogramado pelo top. Em `S_DONE`, um novo `start` funciona sem reset e mantém mapa e Q-table.

### `maze_env.v` (ambiente do labirinto)
A grade 32×32 é um vetor de 1024 bits (`wall_map`, 1 = parede). O estado tem 10 bits: `[9:5]` é a linha e `[4:0]` a coluna. O agente começa em (0,0) e a meta é (31,31). São cinco ações: cima, baixo, esquerda, direita e parar. Recompensas: +120 na meta, −1 por passo, −5 ao bater em parede ou borda, −2 ao parar.

A transição é combinacional. A porta de programação (`prog_we`, `prog_byte_addr`, `prog_byte_data`, síncrona) sobrescreve o mapa byte a byte, na mesma convenção do `generate_maze.py`. `map_ready` sobe depois de 128 bytes e já vale 1 no reset, porque o `WALL_MAP_DEFAULT` é um mapa válido.

### `q_table_ram5.v` (Q-table em 5 bancos)
Uma RAM única de 5120 × 16, endereçada por `state*5+action`, não daria para ler as cinco ações num ciclo. Por isso são 5 RAMs de 1024 × 16, uma por ação, todas endereçadas pelo estado. Assim Q(s,0..4) e Q(s',0..4) saem no mesmo ciclo por duas portas de leitura assíncronas (`addr` e `addr_n`). A escrita é síncrona e vai para um banco só, escolhido por `wr_action`. Todas as posições começam em −16, que é −1,0 em Q4.

### `ula_bellman.v` (atualização de Bellman em ponto fixo Q4)
Calcula `Q_new = Q_old + α·(R·16 + γ·maxQ' − Q_old)` com α = 0,5 (`>>> 1`) e γ = 0,9375 (`x − (x >>> 4)`), tudo multiplicado por 16 (4 bits fracionários). Com inteiros puros, o termo da recompensa truncava para zero e o valor da meta não chegava ao início de caminhos com mais de 60 passos; a escala Q4 resolveu isso. O maior Q possível é 30720, abaixo de 32767, então 16 bits não estouram.

### `argmax5.v`
Recebe cinco Q-values de 16 bits com sinal e devolve o índice (0 a 4) e o valor do maior. Em caso de empate, ganha o menor índice.

### `lfsr16.v` (gerador pseudoaleatório para o ε-greedy)
LFSR de 16 bits com polinômio x¹⁶+x¹⁴+x¹³+x¹¹+1, período 65535 e semente `0x5A5A`. Os bits [7:0] (`rand_out`) escolhem a ação aleatória (`% 5` na FSM). Os bits [15:8] (`rand_high`) são comparados com o epsilon para decidir entre explorar e aproveitar.

### `uart_tx.v` e `uart_rx.v` (UART 8N1)
`CLKS_PER_BIT` é parametrizável: 434 dá 115200 baud a 50 MHz, e na simulação usamos 4. O RX tem sincronizador de 2 flip-flops, confere o start bit e amostra no meio do bit; erro de framing é descartado sem aviso. No TX, atenção: `o_Tx_Done` fica em 1 por 2 ciclos (fim do STOP e CLEAN), então quem usa esse sinal precisa detectar a borda. O cabeçalho do módulo e a seção 7 explicam.

### Testbenches Verilog (Icarus ou Xcelium, sem cocotb)

| Arquivo | Alvo | O que verifica |
|---|---|---|
| `argmax5_tb.v` | argmax5 | empates, negativos, extremos |
| `lfsr16_tb.v` | lfsr16 | reset, enable, 500 ciclos |
| `ula_bellman_tb.v` | ula_bellman | casos de α e γ em Q4 |
| `q_table_ram5_tb.v` | q_table_ram5 | −16 inicial nos 5 bancos, escrita só no banco de `wr_action`, independência das 2 portas de leitura, extremos, `wr_action` inválido |
| `maze_env_tb.v` | maze_env | movimentos, bordas, paredes, meta |
| `tb_prog_port.v` | maze_env (porta de programação) | escreve um padrão diferente em cada endereço e lê os 128 bytes de volta (regressão do bug 9) |
| `fsm_qlearning_v2_tb.v` | fsm_qlearning_v2 | com RAM5, LFSR e maze_env como no top: as 2 portas em `S_STEP`, 1 escrita por passo com leitura de volta, `mon_valid` uma vez por episódio, decaimento do epsilon, restart em `S_DONE` mantendo a Q-table (regressão do bug 4) |
| `tb_uart_map_e2e.v` | sistema completo | 0xFF, 0xB0 + `mapa_top` e 0xA0 pela UART; compara o `wall_map` bit a bit e confere o aprendizado (usa `mapa_top_bytes.hex` e `mapa_top_expected.hex`) |
| `qlearning_top_v2_tb.v` | sistema completo | treino completo, contando as metas (o comando de compilação está no cabeçalho) |

Os antigos `fsm_qlearning_tb.v` e `q_table_ram_tb.v` dependiam dos fontes da v1 (`fsm_qlearning.v` e `q_table_ram.v`), que não estão nesta pasta. Nós os trocamos por `fsm_qlearning_v2_tb.v` e `q_table_ram5_tb.v`, já na arquitetura atual. O `maze_env_tb.v` também mudou. Ele ainda instanciava o maze_env antigo, sem clk nem rst_n; sem reset, o `wall_map` dinâmico ficava em X e o teste de parede interna falhava sem avisar. Esse teste ainda procurava uma parede em (0,1) que só existia no mapa padrão antigo, e agora usa (0,12)→(0,13) do `WALL_MAP_DEFAULT` atual. Cada testbench traz no cabeçalho o comando de compilação para o Icarus, e todos passam sem falhas.

## 4. Ferramentas Python

### `generate_maze.py`
Gera e gerencia labirintos, por menu interativo (sem argumentos) ou pela linha de comando. Cria mapas aleatórios (`--wall-prob`, `--seed`), carrega e salva `.npy`, mostra o mapa em ASCII e no Matplotlib, exporta a string hex do `WALL_MAP_DEFAULT` (conferindo a volta) e envia o mapa pela UART (`--send PORTA --start`). Os testes cocotb importam as funções `generate_maze`, `maze_to_bytes`, `maze_to_hex` e `bytes_to_maze` direto deste arquivo.

### `qlearning_visualizer.py`
Mostra o treino ao vivo e deixa voltar no tempo. Roda em três threads:

1. `uart_reader` decodifica o mapa (0xB0 + 128 B) e os pacotes de 5 B, e aguenta ecos de comando e colisões com o marcador 0xB0.
2. `qtable_updater` reconstrói a Q-table em software, refazendo os episódios com ε-greedy até bater com o resultado que o hardware informou. O decaimento do epsilon segue o do RTL (`_epsilon_rtl`).
3. A interface Matplotlib atualiza a uns 20 Hz.

Os painéis mostram a ocupação do episódio (reconstruída em software, e o painel diz isso), a política argmax(Q) e as estatísticas da telemetria real. Cada episódio guarda um snapshot, e dá para navegar pelo slider, pelo botão [AO VIVO], pelas setas ←/→ (±1) e ↑/↓ (±25), por Home/End e pela tecla V. Também tem menu interativo, modo offline para logs antigos (`--log`) e exportação de animação (`--output`, `--fps`).

### Utilitários
* `edit_maze.py`: editor gráfico de `.npy`; o clique alterna a parede e 'S' salva.
* `converter_npy.py`: converte o PNG de um labirinto em `.npy` com OpenCV, lendo o pixel central de cada célula.
* `plot_maze.py`: desenha um WALL_MAP a partir da string hex do Verilog.

### Mapas incluídos (`.npy` e `.png`)
`labirinto_bom.npy` (tem solução, a 62 passos pela BFS), `mapa_top.npy` (70 passos), `mapa_melhor.npy` e `maze.npy`. Os mapas `fechado*.npy`, `mapa_top_fechado.npy` e `meta_fechada.npy` são densos ou fechados de propósito: usamos esses para investigar por que a telemetria sumia nos episódios que estouravam o limite de passos. Os vídeos `visu_*.mp4` e o `qlearning_viz.gif` são gravações de execuções de referência.

## 5. Modos de co-simulação

Nos três modos o DUT é o RTL de verdade (`qlearning_top_v2`). O cocotb controla clock e reset, e o `uart_bfm.py` faz a UART bit a bit nos pinos (`UartDriver` em `i_Rx_Serial` e `UartMonitor` em `o_Tx_Serial`). Por padrão `CLKS_PER_BIT=4`, para a simulação andar mais rápido. O baud da simulação não precisa ser o da porta real, porque só a ordem dos bytes importa.

### Modo A: tudo num processo (`test_fpga_mode.py` + `Makefile`)
Importa `generate_maze.py` e `qlearning_visualizer.py` como bibliotecas e chama as funções direto, sem arquivos intermediários nem porta serial. Com `MAZE_SEED` o resultado se repete, o que serve para CI.

```bash
make                      # SIM ?= xcelium; use SIM=icarus sem licença
MAZE_SEED=42 make         # labirinto reprodutível
MAX_EPISODES=300 make     # para mais cedo (variável do testbench)
LIVE_PLOT=1 make          # janela do matplotlib ao vivo (mais lento)
CLKS_PER_BIT=4 make       # baud simulado (o Makefile repassa para -defparam/-P)
```

### Modo B: ponte PTY com socat, no Linux (`test_fpga_hw_bridge.py`)
Os scripts reais, sem mudança e com pyserial de verdade, conversam com a simulação por um cabo null-modem virtual:

```bash
# Terminal 1: par de PTYs (mata sessões órfãs antes; deixe aberto)
./start_virtual_fpga.sh
# Terminal 2: simulação no lado /tmp/vcom1
make -f Makefile.bridge SIM=icarus
# Terminal 3: scripts reais no lado /tmp/vcom0
python3 generate_maze.py --seed 42 --send /tmp/vcom0 --start --no-plot
python3 qlearning_visualizer.py --port /tmp/vcom0
```

Variáveis de ambiente: `SIM_SERIAL_PORT` (padrão `/tmp/vcom1`), `SIM_SERIAL_BAUD` e `CLKS_PER_BIT`. Se reiniciar o script das portas, reinicie a simulação também; senão ela continua presa num PTY órfão. O script já mata as sessões antigas sozinho.

### Modo C: hub TCP, no Windows, Linux ou macOS (`run_bridge_win.py`)
Aqui não há socat nem PTY. A própria simulação abre um hub TCP em `socket://127.0.0.1:5555`, e o pyserial trata essa URL como uma porta serial comum. Cada cliente tem sua conexão: o hub manda a saída do DUT para todos, junta as entradas e repete para os outros clientes o que um deles envia. É assim que o visualizador fica sabendo do mapa que o `generate_maze.py` enviou. O passo a passo de instalação e uso está no [README_WINDOWS.md](README_WINDOWS.md).

```bash
python run_bridge_win.py                                        # terminal 1
python qlearning_visualizer.py --port socket://127.0.0.1:5555   # terminal 2
python generate_maze.py            # terminal 3 (porta = a URL acima)
```

Variáveis de ambiente: `SIM_TCP_HOST` e `SIM_TCP_PORT` (padrão 127.0.0.1:5555), `CLKS_PER_BIT` (padrão 4) e `BRIDGE_MAX_CYCLES` (0 = sem limite).

## 6. Testes

Os testes estão em camadas. Rode de baixo para cima para saber onde está a falha:

```bash
make MODULE=test_loopback   # 1) só o BFM: driver → monitor no mesmo fio
make -f Makefile.tx         # 2) uart_tx.v sozinho, bytes em sequência
python run_diag.py          # 3) sinais internos (mon_* em mon_valid) contra os pacotes
                            #    UART decodificados pelo decodificador do visualizador
make                        # 4) cadeia completa (test_fpga_mode.py)
```

Se a camada 1 falha, o problema está no BFM. Se só a 4 falha, o problema está no RTL; foi assim que achamos o bug do `tx_done`. O `test_diag_uart.py` (camada 3) é a ferramenta para quando o visualizador mostra uma coisa e o hardware parece ter treinado outra.

Os testbenches Verilog da seção 3 testam cada módulo sozinho, com o comando de compilação no cabeçalho de cada arquivo. Por exemplo:

```bash
# unitários
iverilog -g2012 -o sim_ram5 q_table_ram5_tb.v q_table_ram5.v && vvp sim_ram5
iverilog -g2012 -o sim_fsm_v2 fsm_qlearning_v2_tb.v fsm_qlearning_v2.v \
         q_table_ram5.v ula_bellman.v argmax5.v lfsr16.v maze_env.v && vvp sim_fsm_v2
# sistema completo (RTL + UART, sem cocotb)
iverilog -g2012 -o sim_e2e tb_uart_map_e2e.v qlearning_top_v2.v \
         fsm_qlearning_v2.v q_table_ram5.v ula_bellman.v argmax5.v \
         lfsr16.v maze_env.v uart_tx.v uart_rx.v && vvp sim_e2e
```

## 7. Bugs que encontramos e corrigimos

Todos já estão corrigidos nos fontes, e os comentários nos módulos apontam para esta lista.

1. **O `tx_done` de 2 ciclos embaralhava os pacotes.** O `o_Tx_Done` do `uart_tx.v` fica em 1 por 2 ciclos (STOP e CLEAN), mas o serializador do top avançava com `if (tx_done)`, sem detectar borda. Ele pulava 2 estados por byte e perdia 1 de cada 2 bytes do pacote. Corrigimos com o registrador `tx_done_r` e `tx_done_pulse = tx_done & ~tx_done_r` no top. Deixamos o pulso de 2 ciclos no `uart_tx` por compatibilidade, e o cabeçalho do módulo agora descreve esse comportamento.

2. **Episódios que estouravam o limite de passos não mandavam nada pela UART.** O gatilho era a borda de `mon_done`, que só pulsa quando o agente chega à meta. Em labirintos densos, o treino inteiro rodava em silêncio. Trocamos por `mon_valid`, um pulso em `S_NEW_EP` a cada fim de episódio, no mesmo ciclo em que `mon_episode` e `mon_steps` são registrados.

3. **A flag de meta da telemetria não era confiável.** `mon_done` já está em 0 no ciclo em que `mon_valid` aparece, então o TX não conseguia amostrá-lo. Criamos o `mon_goal`, que sobe em `S_STEP` com `ep_done` e só volta a 0 em `S_INIT_EP`, e um bit `goal` no byte 4 do pacote. O decodificador Python lê esse bit e, para RTL antigo, ainda aceita `reward == 120` como meta.

4. **Não havia como sair de `S_DONE`.** Só o soft reset 0xFF tirava a FSM de lá, e ele volta para o `WALL_MAP_DEFAULT`. Agora `S_DONE` aceita um novo `start` e mantém o mapa e a Q-table.

5. **O replay do visualizador não batia com o hardware.** O epsilon do replay estava fixo em 0, então a política reconstruída achava a meta enquanto a telemetria real mostrava 0 %. Agora `_epsilon_rtl(ep)` segue o decaimento do RTL, e `_replay_consistente` tenta várias trajetórias, mexendo no epsilon, até chegar ao mesmo `(steps, goal)` que o hardware informou.

6. **O marcador 0xB0 se confundia com dados de telemetria.** O protocolo não escapa o 0xB0, então um byte de dado com esse valor virava "início de mapa" e 128 bytes de telemetria eram lidos como paredes. Para reduzir o problema, o 0xB0 só vale como cabeçalho na fronteira de um pacote, com o buffer vazio. Os ecos de 0xFF e 0xA0 que o hub repete também são filtrados nessa fronteira.

7. **O pyserial apagava dados num PTY compartilhado.** O `open()` do pyserial chama `tcflush(TCIFLUSH)`. Num PTY a fila de entrada é do dispositivo, então abrir a porta num processo apagava telemetria que o outro ainda não tinha lido. A função `_open_porta_sem_flush()` desliga esse flush em portas POSIX; em `socket://` e em COMx isso não acontece.

8. **PTY órfão depois de reiniciar o socat.** A simulação continuava com o descritor do par antigo, enquanto os links simbólicos apontavam para o par novo. O `start_virtual_fpga.sh` agora mata as sessões antigas antes de subir, e o `uart_reader` do visualizador reconecta sozinho se a porta cair. O hub TCP do modo C não tem esse problema.

9. **Índice truncado na porta de programação do mapa (`maze_env.v`).** A escrita usava `wall_map[prog_byte_addr * 7'd8 +: 8]`. A base de um part-select indexado tem a largura determinada pelos próprios operandos, e o produto de dois valores de 7 bits fica com 7 bits: `127×8 = 1016` virava 1016 mod 128. Os 128 bytes davam 8 voltas sobre os bits [127:0]. As linhas 4 a 31 ficavam com o `WALL_MAP_DEFAULT` e as linhas 0 a 3 acabavam com as linhas 28 a 31 do mapa enviado.

   Na prática, todo mapa enviado pela UART virava uma mistura que quase nunca tinha solução. O hardware estourava o limite de passos em 100 % dos episódios, enquanto o replay do visualizador, que usava o mapa certo, chegava à meta. Investigamos essa diferença com o `mapa_melhor.npy` e o `mapa_top.npy`. O `labirinto_bom.npy` funcionava por coincidência: ele é igual ao `WALL_MAP_DEFAULT`, e a mistura preservava o caminho de 62 passos.

   A correção foi `wall_map[{prog_byte_addr, 3'b000} +: 8]`; a concatenação dá um índice de 10 bits. Os testes de regressão são o `tb_prog_port.v` (padrão diferente em cada endereço e leitura dos 128 bytes) e o `tb_uart_map_e2e.v`, que manda 0xFF, 0xB0 + 128 bytes e 0xA0 pela UART, compara o `wall_map` bit a bit e confere que o hardware aprende o `mapa_top.npy`: 500 metas nos últimos 500 de 2000 episódios.

## 8. Arquivos

| Tipo | Arquivos |
|---|---|
| RTL | `qlearning_top_v2.v`, `fsm_qlearning_v2.v`, `maze_env.v`, `q_table_ram5.v`, `ula_bellman.v`, `argmax5.v`, `lfsr16.v`, `uart_tx.v`, `uart_rx.v` |
| Testbenches Verilog | `argmax5_tb.v`, `lfsr16_tb.v`, `ula_bellman_tb.v`, `q_table_ram5_tb.v`, `maze_env_tb.v`, `tb_prog_port.v`, `fsm_qlearning_v2_tb.v`, `tb_uart_map_e2e.v`, `qlearning_top_v2_tb.v` |
| Testes cocotb | `test_fpga_mode.py`, `test_fpga_hw_bridge.py`, `test_fpga_hw_bridge_win.py`, `test_diag_uart.py`, `test_loopback.py`, `test_tx_only.py`, `uart_bfm.py` |
| Execução | `Makefile` (modo A), `Makefile.bridge` (modo B), `Makefile.tx` (uart_tx sozinho), `run_bridge_win.py` (modo C), `run_diag.py`, `start_virtual_fpga.sh` |
| Ferramentas Python | `generate_maze.py`, `qlearning_visualizer.py`, `edit_maze.py`, `converter_npy.py`, `plot_maze.py` |
| Dados e mídia | `*.npy` (mapas), `*.png`, `visu_*.mp4`, `qlearning_viz.gif` |
| Documentação | `README.md` e `README_WINDOWS.md` (modo C no Windows) |

Para instalar as dependências Python:

```bash
pip install cocotb pyserial numpy matplotlib     # opencv-python para o converter_npy.py
```

Simuladores: o Icarus Verilog, que é gratuito e basta para tudo aqui, ou o Cadence Xcelium (`xrun` no PATH, ou `XCELIUM_BIN_DIR` no Makefile).

## 9. Limitações

* O `CLKS_PER_BIT` do Verilog e o do BFM em Python têm que ser iguais. Os Makefiles e runners já repassam a variável de ambiente por `-defparam` (Xcelium), `-P` (Icarus) ou `parameters=` (runner do cocotb).
* Com `MAX_STEPS=1000` e `MAX_EPISODES=5000`, que são os parâmetros do `fsm_inst` no top, a simulação completa demora mesmo com `CLKS_PER_BIT=4`. Durante o desenvolvimento, use `MAX_EPISODES` (modo A) ou `BRIDGE_MAX_CYCLES` (modo C). O Icarus é bem mais lento que o Xcelium.
* Com `LIVE_PLOT=1`, o Matplotlib roda numa thread separada e redesenha no máximo uma vez a cada `refresh_s`. Responde bem, mas não acompanha ciclo a ciclo.
* O hardware só manda `(episode, steps, reward, goal, action)`; a trajetória não é transmitida. Por isso o painel de ocupação do visualizador é uma reconstrução estatística feita em software, e o painel avisa isso.
* Ainda é possível, em tese, que um byte 0xB0 caia exatamente numa fronteira de pacote (item 6). Para eliminar isso de vez, o framing do hardware precisaria de escape ou CRC.
* O caminho combinacional de `S_STEP` é longo: 2 leituras de RAM, 2 argmax, o ambiente e o Bellman em série. Isso pode limitar o Fmax na síntese, e o cabeçalho da FSM descreve o pipeline de 2 estágios como alternativa.
