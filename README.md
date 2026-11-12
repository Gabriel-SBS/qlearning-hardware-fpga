# Q-Learning 32×32 em FPGA — RTL + Co-simulação cocotb + Visualização ao vivo

Acelerador de **Q-Learning tabular** em Verilog: um agente aprende a resolver
um labirinto 32×32 inteiramente em hardware (FSM de 5 estados, ~1 ciclo por
passo de treinamento), recebendo o mapa e o comando de start via **UART** e
transmitindo telemetria de episódios de volta pela mesma UART. Um conjunto de
ferramentas Python (geração de labirintos, visualizador ao vivo com navegação
temporal) e três modos de co-simulação **cocotb** permitem desenvolver e
depurar todo o sistema sem FPGA física — inclusive no Windows.

**Autores:** Gabriel Sena Barbosa Silva ([@Gabriel-SBS](https://github.com/Gabriel-SBS)) e João Vítor Cedraz ([@JvCedrazC](https://github.com/JvCedrazC)).

---

## Sumário

1. [Visão geral da arquitetura](#1-visão-geral-da-arquitetura)
2. [Protocolo UART](#2-protocolo-uart)
3. [Descrição dos módulos RTL](#3-descrição-dos-módulos-rtl)
4. [Ferramentas Python](#4-ferramentas-python)
5. [Modos de co-simulação](#5-modos-de-co-simulação)
6. [Testes e verificação](#6-testes-e-verificação)
7. [Bugs reais encontrados e corrigidos](#7-bugs-reais-encontrados-e-corrigidos)
8. [Inventário de arquivos](#8-inventário-de-arquivos)
9. [Limitações conhecidas](#9-limitações-conhecidas)

---

## 1. Visão geral da arquitetura

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

O passo de treinamento inteiro — ler Q(s,·), escolher ação (ε-greedy),
consultar o ambiente, ler Q(s',·), argmax e atualização de Bellman — fecha
**combinacionalmente em um único ciclo** no estado `S_STEP`. O `maze_env` é
instanciado no top-level (não dentro da FSM) para que a porta de programação
do mapa fique acessível à máquina de recepção UART.

### Software (co-simulação e visualização)

```
generate_maze.py ──0xFF/0xB0+128B/0xA0──▶ ┌──────────────────────────┐
                                          │  FPGA real (COM/ttyUSB)  │
qlearning_visualizer.py ◀──5 B/episódio── │  OU simulação cocotb via │
   (3 painéis + time-travel)              │  PTY socat / hub TCP     │
                                          └──────────────────────────┘
```

`generate_maze.py` e `qlearning_visualizer.py` funcionam **sem nenhuma
modificação** tanto contra a FPGA real quanto contra a simulação — só muda a
string da porta (`COM3`, `/dev/ttyUSB0`, `/tmp/vcom0` ou
`socket://127.0.0.1:5555`).

---

## 2. Protocolo UART

UART 8N1, baud padrão 115200 (CLKS_PER_BIT = 434 @ 50 MHz; parametrizável).

### RX — comandos para o hardware (1 byte, exceto upload de mapa)

| Byte  | Efeito |
|-------|--------|
| `0xA0` | **Start** — inicia/reinicia o treinamento (só se `map_ready = 1`). Em `S_DONE`, reinicia SEM reset: preserva mapa e Q-table, zera contadores e epsilon. |
| `0xFF` | **Soft reset** global (8 ciclos). ⚠ Restaura o `WALL_MAP_DEFAULT` — reenvie o mapa depois. |
| `0xB0` | **Upload de mapa** — os próximos 128 bytes são o WALL_MAP (byte 0 = bits[7:0] … byte 127 = bits[1023:1016]; bit N = célula (N÷32, N mod 32)). `map_ready` sobe após o 128º byte. |
| outros | Ignorados. |

### TX — telemetria (5 bytes ao fim de CADA episódio: meta OU timeout)

| Byte | Conteúdo |
|------|----------|
| 0 | `episode[15:8]` |
| 1 | `episode[7:0]` |
| 2 | `steps[11:4]` |
| 3 | `{steps[3:0], reward[7:4]}` |
| 4 | `{reward[3:0], goal, action[2:0]}` — `goal` (bit 3) = flag explícita de meta atingida |

O gatilho de transmissão é `mon_valid` (pulso em `S_NEW_EP` para **todo** fim
de episódio) e a flag de meta é `mon_goal` (sticky durante o fim do episódio)
— ver [seção 7](#7-bugs-reais-encontrados-e-corrigidos) para o histórico
dessas escolhas.

---

## 3. Descrição dos módulos RTL

### `qlearning_top_v2.v` — Top-level
Integra tudo: UART RX/TX, máquina de comandos (start / soft reset / upload de
mapa em 128 bytes), serializador de telemetria de 5 bytes, `maze_env` com
mapa dinâmico, núcleo Q-Learning (`fsm_qlearning_v2` + `q_table_ram5` +
`lfsr16`). Contém duas detecções de borda críticas: `ep_done_pulse`
(= `mon_valid`) para disparar o pacote, e `tx_done_pulse` para avançar o
serializador (o `o_Tx_Done` do `uart_tx` dura 2 ciclos). Parâmetro:
`CLKS_PER_BIT`. Saídas de status: `led_ep` (4 LSBs do episódio),
`training_done_o`.

### `fsm_qlearning_v2.v` — FSM + datapath do agente (otimizada)
FSM de **5 estados** (`S_IDLE → S_INIT_EP → S_STEP ⟲ → S_NEW_EP → S_DONE`),
contra 23 estados da v1 — ~11× mais throughput (1 ciclo/passo). Instancia
dois `argmax5` (Q(s,·) e Q(s',·)) e a `ula_bellman`; a cadeia
leitura → ε-greedy → ambiente → leitura Q(s',·) → argmax → Bellman → escrita
é toda combinacional dentro de `S_STEP`. Trade-off documentado no cabeçalho:
caminho combinacional longo reduz Fmax (pipeline de 2 estágios é a saída se a
síntese reclamar). Parâmetros: `MAX_STEPS`, `MAX_EPISODES`, `EPSILON_INIT/
MIN/DECAY/DECAY_START` (decaimento: −1 a cada `EPSILON_DECAY` episódios a
partir de `EPSILON_DECAY_START`, saturando em `EPSILON_MIN`). Telemetria:
`mon_valid` (pulso em todo fim de episódio), `mon_goal` (sticky), `mon_state/
action/reward/episode/steps`. O ambiente é externo (portas `env_*` +
`cur_state_o`/`cur_action_o`) para suportar o mapa reprogramável no top.
`S_DONE` aceita novo `start` sem reset (preserva mapa e Q-table).

### `maze_env.v` — Ambiente do labirinto (combinacional + mapa dinâmico)
Grade 32×32 num vetor de 1024 bits (`wall_map`, 1 = parede). Estado de 10
bits: `[9:5]` = linha, `[4:0]` = coluna; início (0,0), meta (31,31). Cinco
ações: CIMA/BAIXO/ESQUERDA/DIREITA/PARAR. Recompensas: meta +120, passo −1,
parede/borda −5, parar −2. A transição é puramente combinacional; a porta de
programação (`prog_we`/`prog_byte_addr`/`prog_byte_data`, síncrona) sobrescreve
o mapa byte a byte com a mesma convenção do `generate_maze.py`; `map_ready`
sobe após 128 bytes (e já vale 1 no reset, pois `WALL_MAP_DEFAULT` é válido).

### `q_table_ram5.v` — Q-table em 5 bancos paralelos
Em vez de uma RAM 5120×16 endereçada por `state*5+action`, são **5 RAMs de
1024×16 (uma por ação)**, todas endereçadas por `state` — Q(s,0..4) e
Q(s',0..4) saem em UM ciclo por duas portas de leitura assíncronas
(`addr`/`addr_n`). Escrita síncrona em um único banco, selecionado por
`wr_action`. Inicialização: −16 (= −1.0 em Q4) em todas as posições.

### `ula_bellman.v` — Atualização de Bellman em ponto fixo Q4
`Q_new = Q_old + α·(R·16 + γ·maxQ' − Q_old)` com α = 0.5 (`>>> 1`) e
γ = 0.9375 (`x − (x >>> 4)`), tudo em escala ×16 (4 bits fracionários).
A escala Q4 resolve o truncamento a zero do sinal de recompensa que, com
inteiros puros, impedia o valor da meta de se propagar por caminhos de 60+
passos. Q máximo teórico 30720 < 32767 → sem overflow em 16 bits.

### `argmax5.v` — Seletor de maior Q-value (combinacional)
Recebe cinco Q-values signed de 16 bits, devolve o índice (0..4) e o valor
máximo. Empate → menor índice (prioridade estática).

### `lfsr16.v` — PRNG para exploração ε-greedy
LFSR de 16 bits, polinômio x¹⁶+x¹⁴+x¹³+x¹¹+1, período 65535, seed `0x5A5A`.
`rand_out` (bits [7:0]) escolhe a ação aleatória (`% 5` na FSM);
`rand_high` (bits [15:8]) é comparado com epsilon para decidir
explorar × aproveitar.

### `uart_tx.v` / `uart_rx.v` — UART 8N1
`CLKS_PER_BIT` parametrizável (434 → 115200 @ 50 MHz; 4 na simulação).
RX: sincronizador de 2 FFs, confirmação do start bit e amostragem no meio do
período, framing error descartado em silêncio. TX: ⚠ `o_Tx_Done` fica em
alto por **2 ciclos** (fim de STOP + CLEAN) — consumidores devem usar
detecção de borda (ver cabeçalho do módulo e seção 7).

### Testbenches Verilog standalone (Icarus/Xcelium, sem cocotb)

| Arquivo | Alvo | Observação |
|---|---|---|
| `argmax5_tb.v` | argmax5 | empates, negativos, extremos |
| `lfsr16_tb.v` | lfsr16 | reset, enable, 500 ciclos |
| `ula_bellman_tb.v` | ula_bellman | casos de α/γ em Q4 |
| `q_table_ram5_tb.v` | q_table_ram5 | init −16 nos 5 bancos, seletividade de banco por `wr_action`, independência das 2 portas de leitura, extremos, `wr_action` inválido |
| `maze_env_tb.v` | maze_env | movimentos, bordas, paredes, meta |
| `tb_prog_port.v` | maze_env (porta prog) | escreve padrão único por endereço e lê de volta os 128 bytes (regressão do bug 9) |
| `fsm_qlearning_v2_tb.v` | fsm_qlearning_v2 | topologia do top (RAM5+LFSR+maze_env): contrato das 2 portas em `S_STEP`, 1 escrita/passo com read-back, `mon_valid` 1×/episódio, decaimento de epsilon, restart em `S_DONE` preservando a Q-table (regressão do bug 4) |
| `tb_uart_map_e2e.v` | sistema completo | 0xFF→0xB0+`mapa_top`→0xA0 pela UART, compara `wall_map` bit a bit e confere aprendizado (usa `mapa_top_bytes.hex` / `mapa_top_expected.hex`) |
| `qlearning_top_v2_tb.v` | sistema completo | treino completo, conta metas (comando de compilação no cabeçalho) |

> Os antigos `fsm_qlearning_tb.v` / `q_table_ram_tb.v` (que exigiam os fontes
> v1 `fsm_qlearning.v` / `q_table_ram.v`, ausentes desta pasta) foram
> **substituídos** por `fsm_qlearning_v2_tb.v` / `q_table_ram5_tb.v`, portados
> para a arquitetura atual. O `maze_env_tb.v` também foi atualizado: ele ainda
> instanciava o maze_env legado (puramente combinacional, sem clk/rst_n) —
> sem reset, o `wall_map` dinâmico ficava em X e o caso "parede interna"
> falhava silenciosamente (e testava uma parede em (0,1) que só existia no
> mapa default antigo; agora usa (0,12)→(0,13) do `WALL_MAP_DEFAULT` atual).
> Cada testbench traz o comando de compilação Icarus no cabeçalho; todos
> passam com 0 falhas.

---

## 4. Ferramentas Python

### `generate_maze.py` — gerador/gerenciador de labirintos
Menu interativo (sem argumentos) ou CLI. Gera mapas aleatórios
(`--wall-prob`, `--seed`), carrega/salva `.npy`, pré-visualiza em ASCII e
Matplotlib, exporta a string hex do `WALL_MAP_DEFAULT` (com verificação
round-trip) e envia o mapa via UART (`--send PORTA --start`). Funções puras
(`generate_maze`, `maze_to_bytes`, `maze_to_hex`, `bytes_to_maze`) são
importadas como biblioteca pelos testes cocotb.

### `qlearning_visualizer.py` — visualização ao vivo + navegação temporal
Três threads: `uart_reader` (decodifica mapa 0xB0+128 B e pacotes de 5 B,
com framing robusto a ecos de comando e colisões do marcador 0xB0),
`qtable_updater` (reconstrói a Q-table em software com replay ε-greedy
**consistente com o desfecho** reportado pelo HW, espelhando o decaimento de
epsilon do RTL via `_epsilon_rtl`) e a UI Matplotlib a ~20 Hz. Painéis:
ocupação do episódio (reconstrução SW, rotulada como tal), política argmax(Q)
e estatísticas da telemetria real. Snapshots por episódio permitem
**time-travel**: slider, botão [AO VIVO], setas ←/→ (±1), ↑/↓ (±25),
Home/End, tecla V. Também tem menu interativo, modo offline para logs
legados (`--log`) e export de animação (`--output`, `--fps`).

### Utilitários
* `edit_maze.py` — editor gráfico de `.npy`: clique alterna parede, 'S' salva.
* `converter_npy.py` — PNG de labirinto → `.npy` via OpenCV (amostra o pixel
  central de cada célula da grade).
* `plot_maze.py` — plota um WALL_MAP a partir da string hex do Verilog.

### Mapas incluídos (`.npy` + `.png`)
`labirinto_bom.npy` (solúvel, distância BFS 62), `mapa_top.npy` (70 passos),
`mapa_melhor.npy`, `maze.npy`, e os casos de estresse `fechado*.npy` /
`mapa_top_fechado.npy` / `meta_fechada.npy` (mapas densos/fechados usados
para depurar o silêncio de telemetria em episódios por timeout). Vídeos
`visu_*.mp4` / `qlearning_viz.gif` registram execuções de referência.

---

## 5. Modos de co-simulação

Nos três modos o DUT é o **RTL real** (`qlearning_top_v2`), com clock/reset
controlados pelo cocotb e UART bit-bangada nos pinos por `uart_bfm.py`
(`UartDriver` → `i_Rx_Serial`; `UartMonitor` ← `o_Tx_Serial`). `CLKS_PER_BIT=4`
por padrão para acelerar a simulação (o baud "virtual" não precisa bater com
o baud real da porta — só a ordem dos bytes importa).

### Modo A — Tudo em um processo (`test_fpga_mode.py` + `Makefile`)
Importa `generate_maze.py`/`qlearning_visualizer.py` como bibliotecas e chama
suas funções diretamente: zero arquivos intermediários, zero serial.
Determinístico (`MAZE_SEED`), ideal para CI.

```bash
make                      # SIM ?= xcelium; use SIM=icarus sem licença
MAZE_SEED=42 make         # labirinto reprodutível
MAX_EPISODES=300 make     # corta cedo (env var do testbench)
LIVE_PLOT=1 make          # janela matplotlib ao vivo (mais lento)
CLKS_PER_BIT=4 make       # baud simulado (Makefile propaga ao -defparam/-P)
```

### Modo B — Ponte PTY/socat, Linux (`test_fpga_hw_bridge.py`)
Scripts reais **inalterados**, com pyserial de verdade, falando com a
simulação através de um cabo null-modem virtual:

```bash
# Terminal 1 — par de PTYs (mata sessões órfãs antes; deixe aberto)
./start_virtual_fpga.sh
# Terminal 2 — simulação no lado /tmp/vcom1
make -f Makefile.bridge SIM=icarus
# Terminal 3 — scripts reais no lado /tmp/vcom0
python3 generate_maze.py --seed 42 --send /tmp/vcom0 --start --no-plot
python3 qlearning_visualizer.py --port /tmp/vcom0
```

Env vars: `SIM_SERIAL_PORT` (padrão `/tmp/vcom1`), `SIM_SERIAL_BAUD`,
`CLKS_PER_BIT`. ⚠ Se reiniciar o script das portas, reinicie TAMBÉM a
simulação — senão ela fica presa num PTY órfão (o script já mata sessões
antigas automaticamente).

### Modo C — Hub TCP, Windows/Linux/macOS (`run_bridge_win.py`)
Sem socat/PTY: a própria simulação sobe um **hub TCP** em
`socket://127.0.0.1:5555` (o pyserial trata a URL como porta serial comum).
Cada cliente tem sua conexão: o hub replica a saída do DUT para todos, mescla
as entradas e **ecoa o que um cliente envia para os outros** — é assim que o
visualizador "escuta" o upload de mapa feito pelo `generate_maze.py`.
Ver **README_WINDOWS.md** para o passo a passo completo de instalação e uso.

```bash
python run_bridge_win.py                                        # terminal 1
python qlearning_visualizer.py --port socket://127.0.0.1:5555   # terminal 2
python generate_maze.py            # terminal 3 (porta = a URL acima)
```

Env vars: `SIM_TCP_HOST`/`SIM_TCP_PORT` (padrão 127.0.0.1:5555),
`CLKS_PER_BIT` (padrão 4), `BRIDGE_MAX_CYCLES` (0 = infinito).

---

## 6. Testes e verificação

Regressão em camadas — rode de baixo para cima para isolar falhas:

```bash
make MODULE=test_loopback   # 1) só o BFM: driver → monitor no mesmo fio
make -f Makefile.tx         # 2) uart_tx.v isolado, bytes costas-com-costas
python run_diag.py          # 3) verdade interna (mon_* @ mon_valid) vs pacotes
                            #    UART decodificados pelo decoder do visualizador
make                        # 4) cadeia completa (test_fpga_mode.py)
```

Se (1) falhar, o bug é no BFM; se só (4) falhar com os demais passando, o bug
é no RTL — foi exatamente assim que o bug do `tx_done` foi isolado. O
`test_diag_uart.py` (passo 3) é a ferramenta para suspeitas de divergência
entre o que o visualizador mostra e o que o hardware realmente treinou.

Testbenches Verilog standalone (seção 3) cobrem cada módulo isoladamente —
cada arquivo traz o comando de compilação exato no cabeçalho. Exemplos:

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

---

## 7. Bugs reais encontrados e corrigidos

Histórico condensado — todos já corrigidos nos fontes desta pasta; os
comentários nos módulos apontam para cá.

1. **`tx_done` de 2 ciclos embaralhava os pacotes.** `o_Tx_Done` do
   `uart_tx.v` fica em alto por 2 ciclos (STOP→CLEAN), mas o serializador do
   top avançava em `if (tx_done)` sem detecção de borda — pulava 2 estados
   por byte e descartava 1 a cada 2 bytes do pacote. Correção: registrador
   `tx_done_r` + `tx_done_pulse = tx_done & ~tx_done_r` no top (o pulso de
   2 ciclos do uart_tx foi mantido por compatibilidade; o cabeçalho do módulo
   agora documenta o comportamento real).

2. **Episódios por timeout ficavam mudos na UART.** O gatilho antigo era a
   borda de `mon_done`, que só pulsa quando a meta é atingida — em labirintos
   densos o treinamento inteiro rodava em silêncio. Correção: `mon_valid`,
   pulso emitido em `S_NEW_EP` para TODO fim de episódio, no mesmo ciclo em
   que `mon_episode`/`mon_steps` são registrados.

3. **Flag de meta não confiável na telemetria.** `mon_done` já vale 0 no
   ciclo em que `mon_valid` fica visível, então não servia para ser amostrado
   pelo TX. Correção: `mon_goal` sticky (sobe em `S_STEP` com `ep_done`, só
   limpa em `S_INIT_EP`) + bit explícito `goal` no byte 4 do pacote. O
   decodificador Python lê o bit e mantém o fallback `reward == 120` para RTL
   antigo.

4. **`S_DONE` era beco sem saída.** Só o soft reset 0xFF escapava, e ele
   restaura o `WALL_MAP_DEFAULT`. Correção: `S_DONE` aceita novo `start`
   preservando mapa e Q-table.

5. **Replay do visualizador divergia do hardware.** O epsilon era fixado em
   0 (replay 100% guloso) — a política reconstruída "achava" a meta enquanto
   a telemetria real mostrava 0%. Correção: `_epsilon_rtl(ep)` espelha
   exatamente o decaimento do RTL, e `_replay_consistente` tenta múltiplas
   trajetórias (recozendo epsilon) até bater com o desfecho `(steps, goal)`
   reportado.

6. **Marcador 0xB0 colidia com dados de telemetria.** 0xB0 não é escapado no
   protocolo; um byte de dado com esse valor virava "início de mapa" e 128
   bytes de telemetria eram lidos como paredes. Mitigação: 0xB0 só é aceito
   como cabeçalho na fronteira de um pacote (buffer vazio); ecos de
   0xFF/0xA0 replicados pelo hub também são filtrados nessa fronteira.

7. **Flush destrutivo do pyserial em PTY compartilhado.** O `open()` do
   pyserial executa `tcflush(TCIFLUSH)`; num PTY a fila de entrada pertence
   ao dispositivo, então abrir a porta num processo apagava telemetria ainda
   não lida pelo outro. Correção: `_open_porta_sem_flush()` neutraliza o
   flush em portas POSIX (irrelevante em `socket://` e COMx).

8. **PTY órfão após reiniciar o socat.** A simulação segurava o file
   descriptor do par antigo enquanto os symlinks apontavam para o novo.
   Correções: `start_virtual_fpga.sh` mata sessões antigas antes de subir, e
   o `uart_reader` do visualizador reconecta automaticamente se a porta cair.
   (O hub TCP do modo C elimina a classe inteira do problema.)

9. **Índice truncado na porta de programação do mapa (`maze_env.v`).**
   A escrita usava `wall_map[prog_byte_addr * 7'd8 +: 8]`; a base de um
   part-select indexado é **autodeterminada**, e a multiplicação de dois
   operandos de 7 bits produz resultado de 7 bits — `127×8 = 1016` era
   truncado módulo 128. Os 128 bytes davam 8 voltas sobre os bits [127:0]:
   as linhas 4–31 ficavam com o `WALL_MAP_DEFAULT` e as linhas 0–3 acabavam
   com as linhas 28–31 do mapa enviado. Resultado: todo upload via UART
   criava um mapa "Frankenstein" geralmente **insolúvel** → HW em timeout em
   100% dos episódios, enquanto o replay do visualizador (no mapa correto)
   atingia a meta — a causa raiz da divergência HW × replay investigada com
   `mapa_melhor.npy`/`mapa_top.npy`. `labirinto_bom.npy` "funcionava" por
   coincidência: ele é idêntico ao `WALL_MAP_DEFAULT`, e seu Frankenstein
   preserva o caminho de 62 passos. Correção:
   `wall_map[{prog_byte_addr, 3'b000} +: 8]` (concatenação → índice pleno de
   10 bits). Regressões: `tb_prog_port.v` (padrão único por endereço,
   read-back dos 128 bytes) e `tb_uart_map_e2e.v` (0xFF→0xB0+128B→0xA0 via
   UART, compara `wall_map` bit a bit e confere que o HW aprende o mapa:
   500/500 metas nos últimos 500 de 2000 episódios com `mapa_top.npy`).

---

## 8. Inventário de arquivos

| Categoria | Arquivos |
|---|---|
| RTL (design) | `qlearning_top_v2.v`, `fsm_qlearning_v2.v`, `maze_env.v`, `q_table_ram5.v`, `ula_bellman.v`, `argmax5.v`, `lfsr16.v`, `uart_tx.v`, `uart_rx.v` |
| Testbenches Verilog | `argmax5_tb.v`, `lfsr16_tb.v`, `ula_bellman_tb.v`, `q_table_ram5_tb.v`, `maze_env_tb.v`, `tb_prog_port.v`, `fsm_qlearning_v2_tb.v`, `tb_uart_map_e2e.v`, `qlearning_top_v2_tb.v` |
| Testes cocotb | `test_fpga_mode.py`, `test_fpga_hw_bridge.py`, `test_fpga_hw_bridge_win.py`, `test_diag_uart.py`, `test_loopback.py`, `test_tx_only.py`, `uart_bfm.py` |
| Runners/Makefiles | `Makefile` (modo A), `Makefile.bridge` (modo B), `Makefile.tx` (uart_tx isolado), `run_bridge_win.py` (modo C), `run_diag.py`, `start_virtual_fpga.sh` |
| Ferramentas Python | `generate_maze.py`, `qlearning_visualizer.py`, `edit_maze.py`, `converter_npy.py`, `plot_maze.py` |
| Dados/mídia | `*.npy` (mapas), `*.png` (renders), `visu_*.mp4`, `qlearning_viz.gif` |
| Docs | `README.md` (este arquivo), `README_WINDOWS.md` (guia do modo C no Windows) |

Instalação de dependências Python:

```bash
pip install cocotb pyserial numpy matplotlib     # opencv-python p/ converter_npy.py
```

Simuladores: Icarus Verilog (gratuito; suficiente para tudo aqui) ou Cadence
Xcelium (`xrun` no PATH, ou `XCELIUM_BIN_DIR` no Makefile).

---

## 9. Limitações conhecidas

* O `CLKS_PER_BIT` do parâmetro Verilog e o usado pelo BFM Python **precisam
  ser o mesmo valor** — os Makefiles/runners já propagam a env var via
  `-defparam` (Xcelium) / `-P` (Icarus) / `parameters=` (runner cocotb).
* Com `MAX_STEPS=1000` e `MAX_EPISODES=5000` (parâmetros do `fsm_inst` no
  top), a simulação completa é lenta mesmo com `CLKS_PER_BIT=4`; use a env
  var `MAX_EPISODES` (modo A) ou `BRIDGE_MAX_CYCLES` (modo C) durante o
  desenvolvimento. Icarus é bem mais lento que Xcelium.
* `LIVE_PLOT=1` roda o Matplotlib numa thread separada com redraw throttled
  (~1 frame/`refresh_s`) — responsivo, mas não "tempo real" ciclo a ciclo.
* O hardware transmite apenas `(episode, steps, reward, goal, action)` — a
  **trajetória não é transmitida**; o painel de ocupação do visualizador é
  uma reconstrução estatística em software (rotulada como tal na UI).
* A colisão residual do marcador 0xB0 na fronteira de pacote é
  matematicamente possível (ver item 6 acima); eliminar 100% exigiria mudar
  o framing no hardware (escape/CRC).
* `caminho combinacional de S_STEP` é longo (2× RAM + 2× argmax + ambiente +
  Bellman em série) — pode limitar o Fmax na síntese; o cabeçalho da FSM
  descreve o pipeline de 2 estágios como plano B.
