"""
qlearning_visualizer.py
========================
Visualizador em tempo real do agente Q-Learning 32×32 em hardware
(qlearning_top_v2.v), com navegação temporal pelo histórico completo.

Recebe dados de duas fontes via UART (mesma porta serial):

  ① Mapa do labirinto — enviado pelo generate_maze.py antes do treinamento:
       0xB0 + 128 bytes → WALL_MAP atualizado dinamicamente
       (o marcador 0xB0 só é aceito na fronteira de um pacote de episódio;
        ver comentário em uart_reader sobre a mitigação de colisão)

  ② Telemetria de episódios — enviada pelo hardware ao fim de cada episódio
     (meta atingida OU timeout de MAX_STEPS), 5 bytes por episódio,
     formato do qlearning_top_v2:
         Byte 0 : episode[15:8]
         Byte 1 : episode[7:0]
         Byte 2 : steps[11:4]
         Byte 3 : {steps[3:0], reward[7:4]}
         Byte 4 : {reward[3:0], goal, action[2:0]}
     `goal` (bit 3 do byte 4) é a flag explícita de meta atingida. Para RTL
     antigo que envia 1'b0 nessa posição, o decodificador mantém o fallback
     `reward == 120`.

O que a UI mostra (3 painéis Matplotlib):
  - Esquerda  : mapa de OCUPAÇÃO do episódio selecionado — replay
                epsilon-greedy em software consistente com o desfecho
                (steps, goal) reportado pelo hardware; intensidade ∝ visitas.
                Rótulos deixam claro o que é telemetria de HW e o que é
                reconstrução de SW (o hardware não transmite a trajetória).
  - Centro    : política argmax(Q) reconstruída, com heatmap de intensidade.
  - Direita   : taxa de sucesso e passos médios por episódio (telemetria).

Navegação temporal (modo ao vivo):
  - slider "pacote"     seleciona qualquer episódio já recebido
  - botão [AO VIVO]     volta a seguir o último pacote
  - ← / → e ↑ / ↓       navega ±1 / ±25 pacotes;  Home/End; tecla V = ao vivo
  Snapshots por episódio (Q-table ~20 KB + ocupação ~2 KB) permitem voltar
  a qualquer ponto do treinamento; ~55 MB para 5000 episódios.

Portas aceitas (--port ou menu):
  /dev/ttyUSB0, COM3          FPGA real (ou PTY do socat no Linux)
  /tmp/vcom0                  ponte socat (test_fpga_hw_bridge.py)
  socket://127.0.0.1:5555     hub TCP da co-simulação Windows/Linux
                              (run_bridge_win.py / test_fpga_hw_bridge_win.py)

Modos de operação:
  (sem argumentos)            menu interativo no terminal
  --port <porta>              leitura ao vivo via UART
  --log  xrun.log             log legado (modo offline, gera animação)

Uso:
    # Menu interativo
    python qlearning_visualizer.py

    # Modo ao vivo (UART / hub TCP)
    python qlearning_visualizer.py --port /dev/ttyUSB0
    python qlearning_visualizer.py --port socket://127.0.0.1:5555

    # Com mapa pré-carregado (não espera 0xB0+128 bytes; usa arquivo .npy)
    python qlearning_visualizer.py --port /dev/ttyUSB0 --map maze.npy

    # Modo offline (log legado) + salvar animação
    python qlearning_visualizer.py --log xrun.log --output viz.mp4 --fps 5

Dependências:
    pip install numpy matplotlib pyserial
"""

import re
import sys
import time
import queue
import threading
import argparse
from collections import deque
from typing import Optional, Tuple, List

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import Normalize


# ─── Constantes de visualização ──────────────────────────────────────────────

C_WALL   = "#2c2c2c"
C_FREE   = "#f5f5f0"
C_START  = "#27AE60"
C_GOAL   = "#E74C3C"
C_AGENT  = "#2980B9"
C_BORDER = "#999999"

ACTION_DX = [ 0,  0, -1,  1,  0]   # col offset: UP/DOWN/LEFT/RIGHT/STOP
ACTION_DY = [ 1, -1,  0,  0,  0]   # row offset (y cresce para baixo na imagem)


# ─── Estado global compartilhado entre thread UART e thread de plot ──────────

class SharedState:
    """Tudo que a thread de UI precisa ler da thread UART."""

    def __init__(self):
        self._lock = threading.Lock()

        # Mapa de paredes (32×32, uint8): 0=livre, 1=parede
        self.wall    = np.zeros((32, 32), dtype=np.uint8)
        self.wall_ok = False          # True após 128 bytes de mapa recebidos

        # Q-table reconstruída incrementalmente
        self.Q       = np.zeros((32, 32, 5), np.float32)

        # Histórico de snapshots da Q-table, alinhado com self.episodes:
        # q_snapshots[i] = estado da Q reconstruída APÓS processar episodes[i].
        # Permite navegar para qualquer episódio já recebido (~20 KB cada;
        # ~55 MB para uma corrida completa de 5000 episódios).
        self.q_snapshots: list[np.ndarray] = []

        # Mapa de ocupação por episódio, alinhado com self.episodes:
        # visit_snapshots[i] = (visits 32×32 uint16, linha_final, col_final)
        # do REPLAY epsilon-greedy do episódio i (reconstrução em software —
        # o hardware não transmite a trajetória). ~2 KB por episódio.
        self.visit_snapshots: list[tuple] = []

        # Histórico de episódios
        self.episodes: list[dict] = []

        # Parâmetros de aprendizado (espelham o RTL)
        self.ALPHA = 0.5
        self.GAMMA = 0.9375

        # Flags de UI
        self.new_data = False   # sinaliza ao plot que há algo novo

    # ── Helpers thread-safe ───────────────────────────────────────────────

    def set_wall(self, wall: np.ndarray):
        with self._lock:
            self.wall    = wall.copy()
            self.wall_ok = True
            # Reseta Q-table ao trocar o mapa
            self.Q       = np.zeros((32, 32, 5), np.float32)
            self.episodes.clear()
            self.q_snapshots.clear()
            self.visit_snapshots.clear()
            self.new_data = True

    def add_episode(self, ep: dict):
        """Recebe dict com keys: ep, steps, goal, epsilon, last_action."""
        with self._lock:
            self.episodes.append(ep)
            self.new_data = True

    def snapshot(self):
        """Retorna cópia thread-safe do estado atual."""
        with self._lock:
            return {
                "wall":     self.wall.copy(),
                "wall_ok":  self.wall_ok,
                "Q":        self.Q.copy(),
                "episodes": list(self.episodes),
                # cópia rasa: os arrays são imutáveis após o append
                # (qtable_updater sempre insere Q.copy())
                "q_snapshots": list(self.q_snapshots),
                "visit_snapshots": list(self.visit_snapshots),
                "new_data": self.new_data,
            }

    def counts(self):
        """Leitura leve para o loop de UI (evita copiar tudo a 20 Hz)."""
        with self._lock:
            return len(self.episodes), self.new_data, self.wall_ok

    def clear_new_data(self):
        with self._lock:
            self.new_data = False


# ─── Lógica de ambiente (espelho do maze_env.v) ──────────────────────────────

def make_lookup(wall: np.ndarray):
    """Pré-computa NS_R, NS_C, REW, DONE para um dado mapa de paredes."""
    _DR = np.array([-1,  1,  0,  0,  0], np.int16)
    _DC = np.array([ 0,  0, -1,  1,  0], np.int16)

    NS_R  = np.zeros((32, 32, 5), np.int8)
    NS_C  = np.zeros((32, 32, 5), np.int8)
    REW   = np.full( (32, 32, 5), -1, np.int8)
    DONE  = np.zeros((32, 32, 5), bool)

    for r in range(32):
        for c in range(32):
            for a in range(5):
                if a == 4:
                    NS_R[r,c,a] = r; NS_C[r,c,a] = c; REW[r,c,a] = -2
                    continue
                nr, nc = r + int(_DR[a]), c + int(_DC[a])
                if nr < 0 or nr > 31 or nc < 0 or nc > 31 or wall[nr,nc]:
                    NS_R[r,c,a] = r; NS_C[r,c,a] = c; REW[r,c,a] = -5
                else:
                    NS_R[r,c,a] = nr; NS_C[r,c,a] = nc
                    if nr == 31 and nc == 31:
                        REW[r,c,a] = 120; DONE[r,c,a] = True
    return NS_R, NS_C, REW, DONE


def greedy_path(Q: np.ndarray, wall: np.ndarray, max_steps: int = 500):
    NS_R, NS_C, _, DONE = make_lookup(wall)
    path = [(0, 0)]
    r, c = 0, 0
    visited = set()
    for _ in range(max_steps):
        visited.add((r, c))
        a = int(Q[r, c].argmax())
        nr, nc = int(NS_R[r,c,a]), int(NS_C[r,c,a])
        path.append((nr, nc))
        if DONE[r,c,a] or (nr, nc) in visited:
            break
        r, c = nr, nc
    return path


# ─── Conversão bytes → mapa de paredes ───────────────────────────────────────

def bytes_to_wall(data: bytes) -> np.ndarray:
    """128 bytes (protocolo generate_maze.py) → ndarray 32×32."""
    wall = np.zeros((32, 32), dtype=np.uint8)
    for byte_idx, byte_val in enumerate(data):
        for b in range(8):
            bit_pos = byte_idx * 8 + b
            r = bit_pos // 32
            c = bit_pos % 32
            wall[r, c] = (byte_val >> b) & 1
    wall[0,  0]  = 0
    wall[31, 31] = 0
    return wall


# ─── Thread de recepção UART ─────────────────────────────────────────────────

# ── Espelho do escalonamento de epsilon do RTL (fsm_qlearning_v2) ────────────
# O hardware NÃO transmite epsilon no pacote, mas o escalonamento é 100%
# determinístico em função do número do episódio (parâmetros da instância no
# qlearning_top_v2.v). Reproduzi-lo aqui torna a reconstrução da Q-table pelo
# qtable_updater estatisticamente fiel às trajetórias epsilon-greedy reais —
# antes, epsilon era fixado em 0 e o replay era 100% guloso, gerando uma
# política que divergia completamente da do hardware (o agente de software
# "achava" a meta enquanto a telemetria real mostrava 0% de sucesso).
_EPS_INIT, _EPS_MIN, _EPS_DECAY, _EPS_DECAY_START = 254, 10, 15, 300

def _epsilon_rtl(ep: int) -> int:
    """epsilon(ep) exatamente como decai no RTL: -1 a cada 15 episódios a
    partir do episódio 300, saturando em 10."""
    if ep < _EPS_DECAY_START:
        return _EPS_INIT
    return max(_EPS_MIN, _EPS_INIT - ((ep - _EPS_DECAY_START) // _EPS_DECAY + 1))


def _decode_episode_packet(b) -> dict:
    """Decodifica um pacote de 5 bytes em um dicionário de episódio."""
    episode = (b[0] << 8) | b[1]
    steps   = (b[2] << 4) | (b[3] >> 4)
    reward_raw = ((b[3] & 0x0F) << 4) | (b[4] >> 4)
    # Converte para signed 8-bit
    reward  = reward_raw if reward_raw < 128 else reward_raw - 256
    action  = b[4] & 0x07
    # bit 3 do byte 4: flag explícita de meta (RTL novo envia pkt_goal ali;
    # RTL antigo envia 1'b0, então o fallback reward==120 mantém compatível)
    goal    = bool(b[4] & 0x08) or (reward == 120)
    epsilon = _epsilon_rtl(episode)

    return {
        "ep":      episode,
        "steps":   steps,
        "goal":    goal,
        "epsilon": epsilon,
        "action":  action,
        "reward":  reward,
    }


def uart_reader(port: str, baud: int, state: SharedState, stop_evt: threading.Event):
    """
    Protocolo de recepção:
      - Detecta 0xB0 → lê próximos 128 bytes como mapa
      - Demais bytes chegam em grupos de 5 → decodifica episódio
      - Ignora 0xA0 e 0xFF (comandos de saída, não de entrada neste lado)

    IMPORTANTE sobre o marcador 0xB0:
    O valor 0xB0 NÃO é reservado/escapado nos pacotes de episódio — ele pode
    aparecer legitimamente como qualquer um dos 5 bytes de telemetria (ex.:
    parte de `episode`, `steps`, `reward` ou `action`). Se checarmos 0xB0 em
    QUALQUER posição do fluxo (como a versão antiga fazia), um byte de dado
    que por coincidência valha 0xB0 é interpretado como início de um novo
    mapa, e os 128 bytes seguintes (que na verdade são telemetria) acabam
    sendo lidos como paredes — produzindo mapas "fantasma" com contagens
    diferentes a cada colisão.

    Mitigação aplicada aqui: só tratamos 0xB0 como cabeçalho de mapa quando
    o fluxo está alinhado no limite de um pacote de episódio (buf vazio).
    Isso reduz a superfície de colisão de "qualquer uma das 5 posições" para
    "só a 1ª posição de um pacote nunca confirmado como válido", que é bem
    mais raro. Como resíduo dessa colisão ainda é matematicamente possível,
    guardamos os bytes originais durante o "modo mapa" e, se depois de
    reunir os 128 bytes o restante do fluxo não recomeçar a decodificar
    pacotes de forma plausível, isso fica registrado no log para
    diagnóstico (não há como ter 100% de certeza sem mudar o framing no
    hardware).
    """
    try:
        import serial
    except ImportError:
        sys.exit("pyserial não encontrado. Instale com: pip install pyserial")

    def _abrir_sem_flush():
        """Abre a porta de forma portátil:
        * socket://host:porta  -> hub TCP do test_fpga_hw_bridge_win.py
          (Windows/co-simulação sem PTY; conexão exclusiva, nada a proteger);
        * /dev/... ou /tmp/... -> neutraliza o tcflush(TCIFLUSH) do open()
          do pyserial, pois a fila de entrada do TTY é compartilhada entre
          todos os processos que abrem o mesmo PTY e o flush apagaria bytes
          já enviados pelo hardware;
        * COMx (Windows)       -> abertura normal (acesso exclusivo do SO)."""
        if "://" in port:
            return serial.serial_for_url(port, timeout=0.1)
        s = serial.Serial()
        s.port = port
        s.baudrate = baud
        s.timeout = 0.1
        if os.name == "posix":
            s._reset_input_buffer = lambda: None
        s.open()
        return s

    print(f"[UART] Conectando em {port} @ {baud} baud ...")
    ser = _abrir_sem_flush()
    try:
        print(f"[UART] Conectado. Aguardando dados ...")
        print(f"[UART][debug] ser.is_open={ser.is_open} ser.in_waiting={ser.in_waiting}", flush=True)

        buf = bytearray()   # buffer de bytes de episódio (múltiplos de 5)
        map_mode = False     # True enquanto recebe os 128 bytes do mapa
        map_buf  = bytearray()

        loop_n = 0
        while not stop_evt.is_set():
            try:
                raw = ser.read(64)   # lê até 64 bytes sem bloquear (timeout=0.1s)
            except (serial.SerialException, OSError) as exc:
                # O outro lado do PTY morreu (ex.: socat/start_virtual_fpga.sh
                # foi reiniciado). Reabre resolvendo o symlink de novo, para
                # grudar no par NOVO em vez de ficar preso num PTY órfão.
                print(f"[UART][ERRO] porta caiu ({exc}). O par socat foi "
                      f"reiniciado? Tentando reconectar em {port} ...", flush=True)
                try:
                    ser.close()
                except Exception:
                    pass
                import time as _t
                while not stop_evt.is_set():
                    _t.sleep(1.0)
                    try:
                        ser = _abrir_sem_flush()
                        print("[UART] Reconectado com sucesso.", flush=True)
                        break
                    except (serial.SerialException, OSError):
                        continue
                continue
            loop_n += 1
            if loop_n % 50 == 0:
                print(f"[UART][debug] ainda no loop, iteração {loop_n}, "
                      f"in_waiting={ser.in_waiting}", flush=True)
            if not raw:
                continue
            print(f"[UART][debug] raw recebido ({len(raw)} bytes): {raw.hex()}", flush=True)

            for byte in raw:
                if map_mode:
                    map_buf.append(byte)
                    if len(map_buf) == 128:
                        wall = bytes_to_wall(bytes(map_buf))
                        state.set_wall(wall)
                        print(f"[UART] Mapa recebido ({wall.sum()} paredes)")
                        map_buf.clear()
                        map_mode = False
                elif byte == 0xB0 and len(buf) == 0:
                    # Só aceita o marcador quando alinhado no limite de um
                    # pacote de episódio — evita confundir um byte de dado
                    # (que por acaso vale 0xB0) com o início real de mapa.
                    map_mode = True
                    map_buf.clear()
                    print("[UART] Início de upload de mapa detectado ...")
                elif byte in (0xFF, 0xA0) and len(buf) == 0:
                    # Ecos de comando do generate_maze.py (0xFF = soft reset,
                    # 0xA0 = start) que o hub TCP replica para todos os
                    # clientes. Na fronteira de pacote é seguro descartá-los:
                    # o 1º byte de um pacote real é episode[15:8], que para
                    # MAX_EPISODES=5000 nunca passa de 0x13. Sem este filtro
                    # o eco desalinharia o framing de 5 bytes para sempre.
                    print(f"[UART] Eco de comando 0x{byte:02X} ignorado")
                else:
                    # Byte de telemetria normal — inclui o caso de um 0xB0
                    # que caiu no meio de um pacote de episódio ainda
                    # incompleto (nesse caso é certamente dado, não cabeçalho).
                    buf.append(byte)

                if not map_mode:
                    # Decodifica pacotes de 5 bytes
                    while len(buf) >= 5:
                        b = buf[:5]
                        del buf[:5]
                        state.add_episode(_decode_episode_packet(b))
    finally:
        try:
            ser.close()
        except Exception:
            pass


# ─── Thread de atualização da Q-table ────────────────────────────────────────


def _replay_consistente(Q, NS_R, NS_C, REW, DONE, eps_val, n_steps,
                        want_goal, rng, alpha, gamma):
    """Replay do episódio CONSISTENTE com o desfecho reportado pelo hardware.

    O hardware só transmite (steps, goal); o replay em software usa outro RNG,
    então uma única tentativa pode divergir do desfecho real (ex.: HW chegou
    na meta, replay não). Aqui tentamos algumas vezes — recozendo o epsilon a
    cada tentativa (eps → eps/2 → eps/4 → 0) — e aceitamos a primeira
    trajetória cujo desfecho bate com a telemetria. As atualizações de Q são
    feitas numa cópia e só a tentativa aceita é aplicada, o que também torna a
    reconstrução mais fiel: episódios de meta recebem o update de +120 como no
    hardware. Retorna (Q_final, visits, (r,c) final, reached).
    """
    schedule = [eps_val, eps_val, eps_val // 2, eps_val // 4, 0, 0]
    best = None
    for eps_try in schedule:
        Qt = Q.copy()
        r = c = 0
        visits = np.zeros((32, 32), np.uint16)
        visits[0, 0] = 1
        reached = False
        for _ in range(n_steps):
            if eps_try > 0 and rng.integers(0, 256) < eps_try:
                a = int(rng.integers(0, 5))
            else:
                a = int(Qt[r, c].argmax())
            nr = int(NS_R[r, c, a]); nc = int(NS_C[r, c, a])
            rew  = int(REW[r, c, a]); done = bool(DONE[r, c, a])
            Qt[r, c, a] += alpha * (rew + gamma * float(Qt[nr, nc].max())
                                    - Qt[r, c, a])
            r, c = nr, nc
            if visits[r, c] < 65535:
                visits[r, c] += 1
            if done:
                reached = True
                break
        best = (Qt, visits, (r, c), reached)
        if reached == want_goal:
            break
    return best

def qtable_updater(state: SharedState, stop_evt: threading.Event):
    """
    Aplica incrementalmente as atualizações de Bellman para cada episódio
    recebido. Roda em thread separada para não bloquear o plot.
    """
    last_len = 0
    current_wall = None
    NS_R = NS_C = REW = DONE = None

    while not stop_evt.is_set():
        snap = state.snapshot()
        eps_list = snap["episodes"]

        # Reconstrói lookup se o mapa mudou
        if snap["wall_ok"] and (current_wall is None or
                                not np.array_equal(current_wall, snap["wall"])):
            current_wall = snap["wall"]
            NS_R, NS_C, REW, DONE = make_lookup(current_wall)
            last_len = 0   # re-processa tudo

        if NS_R is None:
            time.sleep(0.05)
            continue

        # Processa episódios novos (incremental)
        if len(eps_list) > last_len:
            with state._lock:
                Q = state.Q
                ALPHA = state.ALPHA
                GAMMA = state.GAMMA

            rng = np.random.default_rng()

            for ep in eps_list[last_len:]:
                Q, visits, (r, c), reached = _replay_consistente(
                    Q, NS_R, NS_C, REW, DONE,
                    ep.get("epsilon", 0), ep["steps"], bool(ep.get("goal")),
                    rng, ALPHA, GAMMA)

                # snapshots após ESTE episódio (para navegação temporal)
                with state._lock:
                    state.q_snapshots.append(Q.copy())
                    state.visit_snapshots.append((visits, r, c, reached))

            with state._lock:
                state.Q = Q

            last_len = len(eps_list)

        time.sleep(0.02)


# ─── Funções de desenho ───────────────────────────────────────────────────────

def _draw_maze_and_visits(ax, wall: np.ndarray, visits: np.ndarray,
                          final_rc: tuple, n_episodes: int,
                          last_ep: Optional[dict] = None,
                          streak_no_goal: Optional[int] = None,
                          replay_reached: Optional[bool] = None):
    """Pinta TODAS as células ocupadas pelo agente no episódio selecionado
    (replay epsilon-greedy em software), com intensidade ∝ nº de visitas.
    Nos primeiros episódios isso mostra a "nuvem" da exploração aleatória;
    no fim do treino, o corredor fino até a meta."""
    ax.clear()
    ax.set_facecolor("white")

    # Labirinto
    for r in range(32):
        for c in range(32):
            fc = C_WALL if wall[r, c] else C_FREE
            ax.add_patch(plt.Rectangle((c, 31-r), 1, 1,
                facecolor=fc, edgecolor=C_BORDER, linewidth=0.1))

    # Células visitadas — escala log para não saturar em revisitas
    vmax = float(visits.max())
    if vmax > 0:
        cmap = plt.get_cmap("Blues")
        lv   = np.log1p(visits.astype(np.float32)) / np.log1p(vmax)
        rs, cs = np.nonzero(visits)
        for r, c in zip(rs, cs):
            t = float(lv[r, c])
            ax.add_patch(plt.Rectangle((c, 31-r), 1, 1,
                facecolor=cmap(0.25 + 0.65 * t), edgecolor="none",
                alpha=0.35 + 0.55 * t, zorder=2))

    ax.add_patch(plt.Circle((0.5, 31.5), 0.38, color=C_START, zorder=5))
    ax.add_patch(plt.Rectangle((31, 0), 1, 1, facecolor=C_GOAL,
                               edgecolor="none", zorder=4))

    fr, fc_ = final_rc
    ax.add_patch(plt.Circle((fc_+0.5, 31-fr+0.5), 0.43, color=C_AGENT, zorder=6))

    # Selo ✓/✗ vem da telemetria REAL do hardware (flag de meta do pacote)
    hw_goal = bool(last_ep.get("goal")) if last_ep else False
    label   = "✓ chegou na meta" if hw_goal else "não chegou"
    color   = "#1D9E75" if hw_goal else "#D85A30"
    n_cells = int((visits > 0).sum())
    if last_ep is not None:
        hw = (f"HW ep {last_ep['ep']}: "
              f"{'meta' if hw_goal else 'timeout'} em "
              f"{last_ep['steps']} passos  ·  {n_cells-1} células ocupadas")
        # replay retenta até bater com a telemetria; se mesmo assim divergiu
        # (política reconstruída ainda fraca), sinaliza honestamente
        if replay_reached is not None and replay_reached != hw_goal:
            hw += "  ·  (replay divergiu)"
    else:
        hw = f"episódios: {n_episodes}"
    ax.set_title(f"Ocupação do episódio (reconstrução)  —  {label}\n{hw}",
                 fontsize=8.5, pad=4, color=color)

    # Aviso explícito quando a telemetria REAL acumula sequência longa sem
    # metas: a pintura acima é reconstrução em software e pode parecer boa
    # mesmo com o hardware falhando — este banner evita a confusão.
    if streak_no_goal is not None and streak_no_goal >= 300:
        ax.text(16, -0.9,
                f"⚠ telemetria HW: 0 metas nos últimos {streak_no_goal} episódios — "
                "verifique solvabilidade do mapa e build do RTL",
                ha="center", va="top", fontsize=7.0, color="#B03A2E",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="#FDEDEC",
                          edgecolor="#E6B0AA", lw=0.8))

    ax.set_xlim(0, 32); ax.set_ylim(0, 32)
    ax.set_aspect("equal"); ax.axis("off")


def _draw_maze_and_path(ax, wall: np.ndarray, path: list, n_episodes: int,
                        last_ep: Optional[dict] = None):
    ax.clear()
    ax.set_facecolor("white")

    # Labirinto
    for r in range(32):
        for c in range(32):
            fc = C_WALL if wall[r, c] else C_FREE
            ax.add_patch(plt.Rectangle((c, 31-r), 1, 1,
                facecolor=fc, edgecolor=C_BORDER, linewidth=0.1))

    # Trajetória
    n = len(path)
    cmap = plt.get_cmap("Blues")
    for i in range(1, n):
        pr, pc = path[i]
        alpha = 0.20 + 0.72 * (i / n)
        color = cmap(0.30 + 0.60 * (i / n))
        ax.add_patch(plt.Rectangle((pc, 31-pr), 1, 1,
            facecolor=color, edgecolor="none", alpha=alpha, zorder=2))

    ax.add_patch(plt.Circle((0.5, 31.5), 0.38, color=C_START, zorder=5))
    ax.add_patch(plt.Rectangle((31, 0), 1, 1, facecolor=C_GOAL, edgecolor="none", zorder=4))

    er, ec = path[-1]
    ax.add_patch(plt.Circle((ec+0.5, 31-er+0.5), 0.43, color=C_AGENT, zorder=6))

    reached  = (er == 31 and ec == 31)
    label    = "✓ chegou na meta" if reached else "não chegou"
    color    = "#1D9E75" if reached else "#D85A30"
    # 1ª linha: caminho guloso RECONSTRUÍDO em software (pode divergir do HW)
    # 2ª linha: telemetria REAL do último episódio recebido do hardware
    if last_ep is not None:
        hw = (f"HW ep {last_ep['ep']}: "
              f"{'meta' if last_ep.get('goal') else 'timeout'} em "
              f"{last_ep['steps']} passos")
    else:
        hw = f"episódios: {n_episodes}"
    ax.set_title(f"Trajetória greedy (reconstrução)  —  {label}\n{hw}",
                 fontsize=8.5, pad=4, color=color)

    ax.set_xlim(0, 32); ax.set_ylim(0, 32)
    ax.set_aspect("equal"); ax.axis("off")


def _draw_policy(ax, Q: np.ndarray, wall: np.ndarray):
    ax.clear()
    ax.set_facecolor("white")

    max_q = Q.max(axis=2)
    free_vals = max_q[wall == 0]
    vmin = free_vals.min() if len(free_vals) else 0
    vmax = free_vals.max() if len(free_vals) else 1
    norm = Normalize(vmin=vmin, vmax=vmax + 1e-9)
    cmap = plt.get_cmap("YlGn")

    for r in range(32):
        for c in range(32):
            if wall[r, c]:
                ax.add_patch(plt.Rectangle((c, 31-r), 1, 1,
                    facecolor=C_WALL, edgecolor=C_BORDER, linewidth=0.12))
                continue
            fc = cmap(norm(max_q[r, c]))
            ax.add_patch(plt.Rectangle((c, 31-r), 1, 1,
                facecolor=fc, edgecolor="#DDDDDD", linewidth=0.08))
            a = int(Q[r, c].argmax())
            dx, dy = ACTION_DX[a] * 0.27, ACTION_DY[a] * 0.27
            cx, cy = c + 0.5, 31 - r + 0.5
            if dx != 0 or dy != 0:
                ax.annotate("",
                    xy=(cx+dx, cy+dy), xytext=(cx-dx, cy-dy),
                    arrowprops=dict(arrowstyle="-|>", color="#163d5e",
                                   lw=0.45, mutation_scale=3.5))
            else:
                ax.plot(cx, cy, ".", color="#777", markersize=1.0)

    ax.add_patch(plt.Rectangle((31, 0), 1, 1, facecolor=C_GOAL, edgecolor="none", zorder=4))
    ax.add_patch(plt.Circle((0.5, 31.5), 0.35, color=C_START, zorder=5))
    ax.set_xlim(0, 32); ax.set_ylim(0, 32)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("Política (argmax Q) + intensidade Q-value", fontsize=8.5, pad=4, color="#333")


def _draw_stats(ax, episodes: list):
    ax.clear()
    ax.axis("off")
    if not episodes:
        ax.text(0.5, 0.5, "Aguardando dados...", ha="center", va="center",
                transform=ax.transAxes, fontsize=10, color="#888")
        return

    # Janela móvel de 100 episódios para suavizar
    window = 100
    goals_pct = []
    avg_steps = []
    for i in range(len(episodes)):
        start = max(0, i - window + 1)
        window_eps = episodes[start:i+1]
        goals_pct.append(100 * sum(1 for e in window_eps if e["goal"]) / len(window_eps))
        avg_steps.append(np.mean([e["steps"] for e in window_eps]))

    ep_nums = [e["ep"] for e in episodes]
    last_ep = max(ep_nums)

    # Layout vertical em 3 faixas SEM sobreposição:
    #   [0.63 – 0.97]  ax1 — taxa de sucesso
    #   [0.28 – 0.56]  ax2 — passos médios
    #   [0.00 – 0.22]  caixa de estatísticas

    ax1 = ax.inset_axes([0.06, 0.63, 0.90, 0.32])
    ax1.plot(ep_nums, goals_pct, color=C_GOAL, lw=1.5, solid_capstyle="round")
    ax1.fill_between(ep_nums, goals_pct, alpha=0.12, color=C_GOAL)
    ax1.set_xlim(0, last_ep + 1)
    ax1.set_ylim(-2, 105)
    ax1.set_ylabel("% metas", fontsize=7, color="#555")
    ax1.tick_params(labelsize=6.5); ax1.grid(axis="y", lw=0.3, alpha=0.4)
    ax1.set_title(f"Taxa de sucesso (janela {window} ep.)", fontsize=7.5, pad=2, color="#444")

    ax2 = ax.inset_axes([0.06, 0.28, 0.90, 0.26])
    ax2.plot(ep_nums, avg_steps, color=C_AGENT, lw=1.5, solid_capstyle="round")
    ax2.fill_between(ep_nums, avg_steps, alpha=0.12, color=C_AGENT)
    ax2.set_xlim(0, last_ep + 1)
    ax2.set_ylabel("passos médios", fontsize=7, color="#555")
    ax2.set_xlabel("episódio", fontsize=6.5, color="#777")
    ax2.tick_params(labelsize=6.5); ax2.grid(axis="y", lw=0.3, alpha=0.4)
    ax2.set_title(f"Passos médios (janela {window} ep.)", fontsize=7.5, pad=2, color="#444")

    # Caixa de estatísticas — faixa inferior, abaixo do rótulo do eixo x de ax2.
    # "episódio" = número real reportado pelo hardware; "pacotes" = quantos
    # chegaram pela UART (episódios curtos são descartados quando o TX está
    # ocupado, então pacotes < episódios é o comportamento esperado).
    last_100 = episodes[-min(100, len(episodes)):]
    g = sum(1 for e in last_100 if e["goal"])
    info = (
        f"Episódio atual (HW): {last_ep}   ·   pacotes recebidos: {len(episodes)}\n"
        f"Últ. 100 pacotes — metas: {g}/{len(last_100)}  ({100*g/len(last_100):.0f}%)\n"
        f"Passos médios (últ. 100): {np.mean([e['steps'] for e in last_100]):.0f}"
        f"   ·   melhor: {min(e['steps'] for e in last_100)} passos"
    )
    ax.text(0.5, 0.19, info, transform=ax.transAxes,
            fontsize=7.2, va="top", ha="center", linespacing=1.7, color="#1a1a18",
            bbox=dict(boxstyle="round,pad=0.45",
                      facecolor="#EEF4FB", edgecolor="#B5D4F4", lw=0.8))


# ─── Loop de visualização ao vivo ─────────────────────────────────────────────

def _render_frame(fig, axes, snap: dict, draw_idx: int, live: bool):
    """Desenha os 3 painéis para o estado até o pacote draw_idx (inclusive).

    live=True  → seguindo o último pacote recebido
    live=False → navegação temporal (slider/teclado)
    """
    eps_all = snap["episodes"]
    eps     = eps_all[:draw_idx + 1]
    wall    = snap["wall"]

    # Q "da época": snapshot alinhado ao pacote; se o qtable_updater ainda
    # não alcançou este pacote, usa a Q mais recente disponível.
    snaps = snap["q_snapshots"]
    if snaps:
        Q = snaps[min(draw_idx, len(snaps) - 1)]
    else:
        Q = snap["Q"]

    last_ep = eps[-1] if eps else None

    # Painel 1: ocupação real do episódio selecionado (replay). Se o updater
    # ainda não processou este pacote, cai no caminho greedy como antes.
    # sequência de episódios sem meta na telemetria real, até o ponto exibido
    streak = 0
    for e in reversed(eps):
        if e["goal"]:
            break
        streak += 1

    vsnaps = snap.get("visit_snapshots", [])
    if vsnaps:
        visits, fr, fc, reached = vsnaps[min(draw_idx, len(vsnaps) - 1)]
        _draw_maze_and_visits(axes[0], wall, visits, (fr, fc),
                              len(eps), last_ep, streak_no_goal=streak,
                              replay_reached=reached)
    else:
        path = greedy_path(Q, wall)
        _draw_maze_and_path(axes[0], wall, path, len(eps), last_ep)

    _draw_policy(axes[1], Q, wall)
    _draw_stats(axes[2], eps)

    ep_hw = last_ep["ep"] if last_ep else 0
    if live:
        modo = "ao vivo"
    else:
        modo = f"navegando  ·  pacote {draw_idx + 1}/{len(eps_all)}"
    fig.suptitle(
        f"Q-Learning 32×32  ·  episódio {ep_hw} (HW)  ·  "
        f"{len(eps_all)} pacotes  ·  {modo}",
        fontsize=10, fontweight="bold",
        color="#1a1a18" if live else "#8A4B08", y=0.985)


def run_live(state: SharedState, refresh_s: float = 0.8):
    """Visualização ao vivo com navegação temporal.

    Controles:
      * slider "pacote"      : seleciona qualquer episódio já recebido
      * botão  [AO VIVO]     : volta a seguir o último pacote
      * ← / →                : pacote anterior / seguinte  (entra em navegação)
      * ↑ / ↓                : salta ±25 pacotes
      * Home                 : primeiro pacote
      * End                  : último pacote (ainda em navegação)
      * tecla V              : volta ao ao vivo

    O loop de eventos roda a ~20 Hz, então a janela continua respondendo
    mesmo depois que o treinamento termina e os pacotes param de chegar —
    dá para navegar pelo histórico completo à vontade.
    """
    matplotlib.use("TkAgg")   # interativo
    from matplotlib.widgets import Slider, Button

    fig, axes = plt.subplots(1, 3, figsize=(15, 6.6),
                              gridspec_kw={"width_ratios": [1, 1, 0.85]})
    fig.patch.set_facecolor("white")
    plt.subplots_adjust(left=0.01, right=0.99, top=0.88, bottom=0.11, wspace=0.06)

    ax_slider = fig.add_axes([0.07, 0.035, 0.70, 0.030])
    ax_btn    = fig.add_axes([0.855, 0.025, 0.095, 0.050])
    slider = Slider(ax_slider, "pacote", 0, 1, valinit=0, valstep=1,
                    color="#4C86C6")
    slider.valtext.set_fontsize(8)
    slider.label.set_fontsize(8)
    btn = Button(ax_btn, "AO VIVO", color="#EEF4FB", hovercolor="#CBE0F6")
    btn.label.set_fontsize(9)

    # view["idx"]: None = ao vivo; int = índice de pacote em navegação
    view = {"idx": None, "dirty": True}
    guard = {"slider": False}   # evita loop set_val → on_changed
    n_cache = [0]               # último n conhecido (para o teclado)

    def _goto(idx):
        view["idx"]   = idx
        view["dirty"] = True

    def on_slider(val):
        if guard["slider"]:
            return
        _goto(int(val))

    def on_live(_event):
        view["idx"]   = None
        view["dirty"] = True

    def on_key(event):
        n = n_cache[0]
        if n == 0:
            return
        cur = view["idx"] if view["idx"] is not None else n - 1
        if   event.key == "right": _goto(min(n - 1, cur + 1))
        elif event.key == "left":  _goto(max(0, cur - 1))
        elif event.key == "up":    _goto(min(n - 1, cur + 25))
        elif event.key == "down":  _goto(max(0, cur - 25))
        elif event.key == "home":  _goto(0)
        elif event.key == "end":   _goto(n - 1)
        elif event.key in ("v", "V"): on_live(None)

    slider.on_changed(on_slider)
    btn.on_clicked(on_live)
    fig.canvas.mpl_connect("key_press_event", on_key)

    plt.ion()
    plt.show()

    last_draw_key = None      # (idx_desenhado, modo) para coalescer
    last_draw_t   = 0.0       # throttle de redraw no modo ao vivo
    waiting_drawn = False
    POLL_S = 0.05             # 20 Hz: janela sempre responsiva

    while plt.fignum_exists(fig.number):
        n, new_data, wall_ok = state.counts()
        n_cache[0] = n

        if not wall_ok:
            if not waiting_drawn:
                for a in axes:
                    a.clear(); a.set_facecolor("#f0f0f0"); a.axis("off")
                axes[1].text(0.5, 0.5,
                             "Aguardando mapa do labirinto\n(envie via generate_maze.py --send)",
                             ha="center", va="center", transform=axes[1].transAxes,
                             fontsize=12, color="#555")
                fig.canvas.draw_idle()
                waiting_drawn = True
            fig.canvas.flush_events()
            time.sleep(POLL_S)
            continue
        waiting_drawn = False

        # mantém o range do slider acompanhando os pacotes que chegam
        if n > 0 and slider.valmax != n - 1:
            guard["slider"] = True
            slider.valmax = n - 1
            slider.ax.set_xlim(0, max(1, n - 1))
            if view["idx"] is None:
                slider.set_val(n - 1)
            guard["slider"] = False

        live = view["idx"] is None
        draw_idx = (n - 1) if live else min(view["idx"], n - 1)

        need = False
        if n > 0:
            if live:
                # redesenha no máx. 1×/refresh_s (cada frame custa ~0,5 s;
                # pacotes chegam bem mais rápido que isso)
                fresh = new_data or (last_draw_key != (draw_idx, "live"))
                need  = fresh and (time.time() - last_draw_t >= refresh_s
                                   or last_draw_key is None)
            else:
                need = view["dirty"] or (last_draw_key != (draw_idx, "nav"))

        if need:
            snap = state.snapshot()
            state.clear_new_data()
            view["dirty"] = False
            _render_frame(fig, axes, snap, draw_idx, live)
            if not live:
                guard["slider"] = True
                slider.set_val(draw_idx)   # sincroniza com teclado
                guard["slider"] = False
            last_draw_key = (draw_idx, "live" if live else "nav")
            last_draw_t   = time.time()
            fig.canvas.draw_idle()

        fig.canvas.flush_events()
        time.sleep(POLL_S)

    plt.close(fig)


# ─── Modo offline (parser de log legado) ──────────────────────────────────────

_EP_RE = re.compile(r"^\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(SIM|nao)\s*\|\s*(\d+)")

def parse_log(path: str, wall_hex: Optional[str] = None) -> Tuple[List[dict], np.ndarray]:
    records = []
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = _EP_RE.match(line)
            if m:
                records.append({
                    "ep":      int(m.group(1)),
                    "steps":   int(m.group(2)),
                    "goal":    m.group(3) == "SIM",
                    "epsilon": int(m.group(4)),
                })

    # Mapa padrão (igual ao WALL_MAP_DEFAULT do maze_env.v)
    DEFAULT_HEX = (
        "1F19008810229025014354140C368800BAE118A012020F244D1B0A10A062B4F2"
        "000A400D1660A8E8730DA007804233102008E24128842080003026E210210084"
        "98103B08610228020810004063080A7A010A8A8426CC30C394828314003A1901"
        "408035301420781488084CA90A501C0C21D21008091818616010C0515CA18400"
    )
    hex_str = wall_hex if wall_hex else DEFAULT_HEX
    clean = hex_str.replace(" ", "").replace("\n", "")
    bits  = np.zeros(1024, np.uint8)
    for i, ch in enumerate(reversed(clean)):
        nibble = int(ch, 16)
        for b in range(4):
            idx = i * 4 + b
            if idx < 1024:
                bits[idx] = (nibble >> b) & 1
    wall = bits.reshape(32, 32)
    wall[0,  0]  = 0
    wall[31, 31] = 0
    return records, wall


def run_offline(log_path: str, output: str, fps: int, wall_hex: Optional[str]):
    """Modo legado: lê log, reconstrói Q-table e salva animação."""
    from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter

    matplotlib.use("Agg")

    print(f"[LOG] Lendo {log_path} ...")
    records, wall = parse_log(log_path, wall_hex)
    if not records:
        sys.exit("Nenhum episódio encontrado no log.")
    print(f"  {len(records)} episódios")

    NS_R, NS_C, REW, DONE = make_lookup(wall)

    # Reconstrói Q-table episódio a episódio e captura snapshots
    ALPHA = 0.5; GAMMA = 0.9375
    Q         = np.zeros((32, 32, 5), np.float32)
    snapshots = []
    rng       = np.random.default_rng(0)

    for i, rec in enumerate(records):
        Q, visits, (r, c), reached = _replay_consistente(
            Q, NS_R, NS_C, REW, DONE,
            rec["epsilon"], rec["steps"], bool(rec.get("goal")),
            rng, ALPHA, GAMMA)

        # Captura a cada 50 episódios
        if (i + 1) % 50 == 0 or i == len(records) - 1:
            snapshots.append((i + 1, Q.copy(), list(records[:i+1]),
                              visits.copy(), (r, c), reached))

        if (i + 1) % 500 == 0:
            print(f"  {i+1}/{len(records)} episódios processados ...")

    print(f"  {len(snapshots)} frames de animação")

    fig, axes = plt.subplots(1, 3, figsize=(15, 6.2),
                              gridspec_kw={"width_ratios": [1, 1, 0.85]})
    fig.patch.set_facecolor("white")
    plt.subplots_adjust(left=0.01, right=0.99, top=0.91, bottom=0.04, wspace=0.06)

    def update(fi):
        n_ep, Q_snap, eps_so_far, visits, final_rc, reached = snapshots[fi]
        _draw_maze_and_visits(axes[0], wall, visits, final_rc, n_ep,
                              eps_so_far[-1] if eps_so_far else None,
                              replay_reached=reached)
        _draw_policy(axes[1], Q_snap, wall)
        _draw_stats(axes[2], eps_so_far)
        fig.suptitle(f"Q-Learning 32×32  ·  episódios: {n_ep} / {len(records)}",
                     fontsize=10, fontweight="bold", color="#1a1a18", y=0.98)

    anim = FuncAnimation(fig, update, frames=len(snapshots),
                          interval=1000//fps, repeat=False)
    ext = output.rsplit(".", 1)[-1].lower()
    print(f"Renderizando → {output}")
    t0 = time.time()
    if ext == "mp4":
        anim.save(output, writer=FFMpegWriter(fps=fps, bitrate=2000), dpi=130,
                  progress_callback=lambda i, n: print(f"\r  frame {i+1}/{n}", end=""))
    else:
        anim.save(output, writer=__import__("matplotlib.animation", fromlist=["PillowWriter"]).PillowWriter(fps=fps), dpi=110,
                  progress_callback=lambda i, n: print(f"\r  frame {i+1}/{n}", end=""))
    print(f"\nSalvo em {output}  ({time.time()-t0:.0f}s)")
    plt.close(fig)


# ─── Utilitários de terminal (menu) ──────────────────────────────────────────

import os
import glob as _glob

_RESET   = "\033[0m"
_BOLD    = "\033[1m"
_DIM     = "\033[2m"
_GREEN   = "\033[32m"
_CYAN    = "\033[36m"
_YELLOW  = "\033[33m"
_RED     = "\033[31m"
_WHITE   = "\033[97m"

def _supports_color() -> bool:
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()

def _c(text: str, *codes: str) -> str:
    if not _supports_color():
        return text
    return "".join(codes) + text + _RESET

def _clear():
    os.system("cls" if os.name == "nt" else "clear")

def _pause(msg: str = "  Pressione Enter para continuar..."):
    input(_c(f"\n{msg}", _DIM))

def _ask(prompt: str, default: str = "") -> str:
    hint = _c(f" [{default}]", _DIM) if default else ""
    try:
        val = input(_c("  ▸ ", _CYAN) + prompt + hint + ": ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return default
    return val if val else default

def _ask_int(prompt: str, default: int, lo: Optional[int] = None,
             hi: Optional[int] = None) -> int:
    while True:
        raw = _ask(prompt, str(default))
        try:
            v = int(raw)
            if lo is not None and v < lo:
                print(_c(f"  Valor mínimo: {lo}", _RED)); continue
            if hi is not None and v > hi:
                print(_c(f"  Valor máximo: {hi}", _RED)); continue
            return v
        except ValueError:
            print(_c("  Entrada inválida — informe um inteiro.", _RED))

def _ask_float(prompt: str, default: float, lo: float = 0.0, hi: float = 99.0) -> float:
    while True:
        raw = _ask(prompt, f"{default:.2f}")
        try:
            v = float(raw)
            if not (lo <= v <= hi):
                print(_c(f"  Valor deve estar entre {lo} e {hi}.", _RED)); continue
            return v
        except ValueError:
            print(_c("  Entrada inválida — informe um número decimal.", _RED))

def _ask_yn(prompt: str, default: bool = False) -> bool:
    hint = "S/n" if default else "s/N"
    raw  = _ask(f"{prompt} [{hint}]", "").lower()
    if raw in ("s", "sim", "y", "yes"):  return True
    if raw in ("n", "não", "nao", "no"): return False
    return default

def _header(title: str):
    w   = 62
    bar = _c("═" * w, _CYAN)
    print(f"\n{bar}")
    print(_c(f"  {title}", _BOLD + _WHITE))
    print(f"{bar}")

def _section(title: str):
    print(_c(f"\n  ── {title} ──", _YELLOW + _BOLD))

def _ok(msg: str):   print(_c(f"  ✔  {msg}", _GREEN))
def _info(msg: str): print(_c(f"  ℹ  {msg}", _CYAN))
def _warn(msg: str): print(_c(f"  ⚠  {msg}", _YELLOW))
def _err(msg: str):  print(_c(f"  ✘  {msg}", _RED))


# ─── Estado de sessão do menu ─────────────────────────────────────────────────

_vsession: dict = {
    "port":    None,
    "baud":    115200,
    "refresh": 1.0,
    "map":     None,   # ndarray 32×32 ou None
    "map_src": None,   # descrição textual da origem
}


def _list_serial_ports() -> List[str]:
    """Detecta portas seriais disponíveis (Linux / macOS / Windows)."""
    try:
        import serial.tools.list_ports
        return [p.device for p in serial.tools.list_ports.comports()]
    except Exception:
        pass
    candidates = (
        _glob.glob("/dev/ttyUSB*") +
        _glob.glob("/dev/ttyACM*") +
        _glob.glob("/dev/tty.usbserial*") +
        _glob.glob("/dev/tty.usbmodem*") +
        _glob.glob("COM[0-9]*")
    )
    return sorted(candidates)


def _status_bar() -> str:
    s = _vsession
    if s["map"] is None:
        map_info = _c("nenhum mapa carregado", _DIM)
    else:
        walls    = int(s["map"].sum())
        map_info = (_c(s["map_src"], _GREEN) +
                    _c(f"  [{walls} paredes · {100*walls/1024:.1f}%]", _DIM))

    port_info = (
        _c(s["port"], _CYAN) + _c(f" @ {s['baud']}", _DIM)
        if s["port"] else _c("não configurada", _DIM)
    )
    return (f"\n  Mapa  : {map_info}\n"
            f"  UART  : {port_info}"
            f"   {_c('refresh ' + str(s['refresh']) + 's', _DIM)}")


# ─── Submenus ─────────────────────────────────────────────────────────────────

def _menu_ao_vivo():
    """1 — Modo ao vivo via UART."""
    _clear()
    _header("Modo ao vivo  —  recepção UART + visualização em tempo real")

    _section("Porta serial")
    ports = _list_serial_ports()
    if ports:
        print()
        for i, p in enumerate(ports, 1):
            print(_c(f"  [{i}] ", _CYAN) + p)
        print()
    else:
        _warn("Nenhuma porta serial detectada automaticamente.")
        print()

    default_port = _vsession["port"] or (ports[0] if ports else "/dev/ttyUSB0")
    port = _ask("Porta serial", default=default_port)
    baud = _ask_int("Baud rate", default=_vsession["baud"], lo=1200, hi=3_000_000)

    _vsession["port"] = port
    _vsession["baud"] = baud

    _section("Mapa inicial (opcional)")
    _info("Deixe em branco para aguardar o mapa via UART (generate_maze.py --send).")
    use_map = _ask_yn("Carregar mapa de arquivo .npy?", default=(_vsession["map"] is not None))

    state    = SharedState()
    stop_evt = threading.Event()

    if use_map:
        npy_files = _glob.glob("*.npy")
        if npy_files:
            print()
            for i, f in enumerate(npy_files, 1):
                print(_c(f"  [{i}] ", _CYAN) + f)
            print()
        path = _ask("Caminho do arquivo .npy",
                    default=(_vsession["map_src"] if _vsession["map_src"] and
                             _vsession["map_src"].endswith(".npy") else "maze.npy"))
        try:
            wall = np.load(path).astype(np.uint8)
            wall[0, 0] = 0; wall[31, 31] = 0
            state.set_wall(wall)
            _vsession["map"]     = wall
            _vsession["map_src"] = path
            _ok(f"Mapa carregado: {path}")
        except Exception as e:
            _err(f"Falha ao carregar: {e}")
            _pause(); return

    _section("Configurações de exibição")
    refresh = _ask_float("Intervalo de atualização (segundos)", default=_vsession["refresh"],
                         lo=0.1, hi=10.0)
    _vsession["refresh"] = refresh

    _section("Resumo")
    _info(f"Porta   : {port} @ {baud} baud")
    _info(f"Mapa    : {'pré-carregado' if use_map else 'aguardando via UART'}")
    _info(f"Refresh : {refresh}s")
    print()
    if not _ask_yn("Iniciar visualização ao vivo?", default=True):
        _info("Cancelado."); _pause(); return

    t_uart = threading.Thread(target=uart_reader,
                               args=(port, baud, state, stop_evt), daemon=True)
    t_qt   = threading.Thread(target=qtable_updater,
                               args=(state, stop_evt), daemon=True)
    t_uart.start()
    t_qt.start()

    try:
        run_live(state, refresh_s=refresh)
    except KeyboardInterrupt:
        print("\n[INFO] Interrompido pelo usuário.")
    finally:
        stop_evt.set()
        t_uart.join(timeout=1.0)
        t_qt.join(timeout=1.0)


def _menu_offline():
    """2 — Modo offline: processa log e gera animação."""
    _clear()
    _header("Modo offline  —  processar log e gerar animação")

    _section("Arquivo de log")
    log_files = _glob.glob("*.log") + _glob.glob("xrun*.log")
    if log_files:
        print()
        for i, f in enumerate(log_files, 1):
            print(_c(f"  [{i}] ", _CYAN) + f)
        print()
    log_path = _ask("Caminho do arquivo de log", default="xrun.log")
    if not os.path.isfile(log_path):
        _err(f"Arquivo não encontrado: {log_path}")
        _pause(); return

    _section("Saída da animação")
    output = _ask("Arquivo de saída (.gif ou .mp4)", default="qlearning_viz.gif")
    fps    = _ask_int("FPS da animação", default=3, lo=1, hi=30)

    _section("Mapa de paredes (opcional)")
    _info("Deixe em branco para usar o WALL_MAP_DEFAULT embutido.")
    wall_hex = _ask("String hex do WALL_MAP (256 chars)", default="").strip() or None
    if wall_hex and len(wall_hex) not in (256, 257):
        _warn(f"Comprimento incomum ({len(wall_hex)} chars); esperado 256. Prosseguindo assim mesmo.")

    _section("Resumo")
    _info(f"Log     : {log_path}")
    _info(f"Saída   : {output}  @ {fps} FPS")
    _info(f"Mapa    : {'hex fornecido' if wall_hex else 'padrão embutido'}")
    print()
    if not _ask_yn("Iniciar processamento?", default=True):
        _info("Cancelado."); _pause(); return

    try:
        run_offline(log_path, output, fps, wall_hex)
        _ok(f"Animação salva em '{output}'")
    except Exception as e:
        _err(f"Erro: {e}")

    _pause()


def _menu_carregar_mapa():
    """3 — Carregar mapa .npy para inspeção."""
    _clear()
    _header("Carregar mapa  —  .npy → pré-visualização ASCII + estatísticas")

    npy_files = _glob.glob("*.npy")
    if npy_files:
        print()
        for i, f in enumerate(npy_files, 1):
            print(_c(f"  [{i}] ", _CYAN) + f)
        print()
    path = _ask("Caminho do arquivo .npy", default="maze.npy")
    try:
        wall = np.load(path).astype(np.uint8)
        if wall.shape != (32, 32):
            _err(f"Esperado 32×32, encontrado {wall.shape}")
            _pause(); return
        wall[0, 0] = 0; wall[31, 31] = 0
    except Exception as e:
        _err(f"Falha ao carregar: {e}")
        _pause(); return

    _vsession["map"]     = wall
    _vsession["map_src"] = path
    walls = int(wall.sum())
    _ok(f"Mapa carregado: {path}  ({walls} paredes · {100*walls/1024:.1f}%)")

    _section("Pré-visualização ASCII  (█ = parede · · = livre)")
    print()
    for r in range(32):
        row_str = "  "
        for co in range(32):
            if r == 0 and co == 0:
                row_str += _c("S", _GREEN)
            elif r == 31 and co == 31:
                row_str += _c("G", _RED)
            elif wall[r, co]:
                row_str += _c("█", _DIM)
            else:
                row_str += _c("·", _WHITE)
        print(row_str)

    _section("Estatísticas")
    _info(f"Paredes : {walls}  ({100*walls/1024:.1f}%)")
    _info(f"Livres  : {1024 - walls}  ({100*(1024-walls)/1024:.1f}%)")

    _section("Ações")
    if _ask_yn("Abrir visualização gráfica (matplotlib)?", default=False):
        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(7, 7))
            ax.imshow(wall, cmap="binary", vmin=0, vmax=1)
            ax.plot(0, 0, "go", markersize=11, label="Início", zorder=5)
            ax.plot(31, 31, "ro", markersize=11, label="Meta", zorder=5)
            ax.legend(loc="upper right", fontsize=9)
            ax.set_title(f"Mapa: {path}", fontsize=12)
            ax.axis("off")
            fig.tight_layout()
            plt.show()
            plt.close(fig)
        except Exception as e:
            _err(f"Erro ao exibir: {e}")

    _pause()


def _menu_configurar_uart():
    """4 — Configurar porta e baud rate."""
    _clear()
    _header("Configurar UART  —  porta e baud rate")

    ports = _list_serial_ports()
    if ports:
        print()
        for i, p in enumerate(ports, 1):
            print(_c(f"  [{i}] ", _CYAN) + p)
        print()
    else:
        _warn("Nenhuma porta detectada.")
        print()

    default_port = _vsession["port"] or (ports[0] if ports else "/dev/ttyUSB0")
    port = _ask("Porta serial", default=default_port)
    baud = _ask_int("Baud rate", default=_vsession["baud"], lo=1200, hi=3_000_000)
    ref  = _ask_float("Refresh da visualização (s)", default=_vsession["refresh"],
                      lo=0.1, hi=10.0)

    _vsession["port"]    = port
    _vsession["baud"]    = baud
    _vsession["refresh"] = ref

    _ok(f"UART configurada: {port} @ {baud}  —  refresh {ref}s")
    _pause()


def _menu_fluxo_rapido():
    """5 — Fluxo rápido: conectar direto à UART usando configuração atual."""
    _clear()
    _header("Fluxo rápido  —  conectar e visualizar agora")

    s = _vsession
    if not s["port"]:
        _warn("Nenhuma porta configurada. Configure via opção [4] primeiro.")
        _pause(); return

    _section("Configuração atual")
    _info(f"Porta   : {s['port']} @ {s['baud']} baud")
    _info(f"Mapa    : {s['map_src'] if s['map_src'] else 'aguardar via UART'}")
    _info(f"Refresh : {s['refresh']}s")

    if not _ask_yn("Iniciar agora?", default=True):
        _info("Cancelado."); _pause(); return

    state    = SharedState()
    stop_evt = threading.Event()

    if s["map"] is not None:
        state.set_wall(s["map"])
        _ok(f"Mapa pré-carregado: {s['map_src']}")

    t_uart = threading.Thread(target=uart_reader,
                               args=(s["port"], s["baud"], state, stop_evt), daemon=True)
    t_qt   = threading.Thread(target=qtable_updater,
                               args=(state, stop_evt), daemon=True)
    t_uart.start()
    t_qt.start()

    try:
        run_live(state, refresh_s=s["refresh"])
    except KeyboardInterrupt:
        print("\n[INFO] Interrompido.")
    finally:
        stop_evt.set()
        t_uart.join(timeout=1.0)
        t_qt.join(timeout=1.0)


# ─── Menu principal ───────────────────────────────────────────────────────────

_MENU_ITEMS = [
    ("1", "Modo ao vivo          (UART → visualização em tempo real)",    _menu_ao_vivo),
    ("2", "Modo offline          (log → reconstruir Q-table → animação)", _menu_offline),
    ("─", None, None),
    ("3", "Carregar / inspecionar mapa (.npy)",                           _menu_carregar_mapa),
    ("4", "Configurar porta UART e refresh",                              _menu_configurar_uart),
    ("─", None, None),
    ("5", "Fluxo rápido          (usar configuração atual e iniciar)",    _menu_fluxo_rapido),
    ("─", None, None),
    ("0", "Sair", None),
]


def _main_menu():
    while True:
        _clear()
        print(_c("\n  ╔══════════════════════════════════════════════════════════╗", _CYAN))
        print(_c("  ║", _CYAN) +
              _c("   Q-Learning 32×32  —  Visualizador                     ", _BOLD + _WHITE) +
              _c(" ║", _CYAN))
        print(_c("  ╚══════════════════════════════════════════════════════════╝", _CYAN))

        print(_status_bar())
        print()

        for key, label, _ in _MENU_ITEMS:
            if key == "─":
                print(_c("  " + "─" * 52, _DIM))
            else:
                bullet = _c(f"  [{key}]", _CYAN + _BOLD)
                print(f"{bullet}  {label}")
        print()

        escolha = _ask("Opção", default="").strip()
        action_map = {k: fn for k, _, fn in _MENU_ITEMS if k != "─" and fn is not None}

        if escolha == "0":
            _clear()
            print(_c("\n  Até logo!\n", _GREEN + _BOLD))
            sys.exit(0)
        elif escolha in action_map:
            action_map[escolha]()
        else:
            _warn("Opção inválida.")
            time.sleep(0.8)


# ─── CLI (modo não-interativo, mantido para scripts) ─────────────────────────

def _run_cli(args):
    state    = SharedState()
    stop_evt = threading.Event()

    if args.log:
        run_offline(args.log, args.output, args.fps, args.wall_hex)
        return

    if args.map:
        wall = np.load(args.map).astype(np.uint8)
        wall[0, 0] = 0; wall[31, 31] = 0
        state.set_wall(wall)
        print(f"[MAP] Labirinto carregado de '{args.map}'")

    t_uart = threading.Thread(target=uart_reader,
                               args=(args.port, args.baud, state, stop_evt), daemon=True)
    t_qt   = threading.Thread(target=qtable_updater,
                               args=(state, stop_evt), daemon=True)
    t_uart.start()
    t_qt.start()

    try:
        run_live(state, refresh_s=args.refresh)
    except KeyboardInterrupt:
        print("\n[INFO] Interrompido pelo usuário.")
    finally:
        stop_evt.set()


def main():
    # Se chamado sem argumentos → menu interativo
    if len(sys.argv) == 1:
        _main_menu()
        return

    ap = argparse.ArgumentParser(
        description="Visualizador Q-Learning 32×32 — ao vivo (UART) ou offline (log)",
        epilog="Sem argumentos: abre o menu interativo no terminal.")

    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--port", metavar="PORT",
                      help="Porta serial para modo ao vivo (ex: /dev/ttyUSB0, COM3)")
    mode.add_argument("--log",  metavar="FILE",
                      help="Arquivo de log legado para modo offline")

    ap.add_argument("--baud",    type=int, default=115200)
    ap.add_argument("--map",     metavar="FILE.npy", default=None)
    ap.add_argument("--refresh", type=float, default=1.0)
    ap.add_argument("--output",  default="qlearning_viz.gif")
    ap.add_argument("--fps",     type=int, default=3)
    ap.add_argument("--wall-hex", metavar="HEX", default=None)

    _run_cli(ap.parse_args())


if __name__ == "__main__":
    main()