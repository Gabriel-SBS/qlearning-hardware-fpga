"""
generate_maze.py
=================
Gerador e gerenciador de labirintos 32×32 para o acelerador Q-Learning RTL
(qlearning_top_v2). Permite gerar mapas aleatórios, carregar/salvar .npy,
exportar a string hex do parâmetro Verilog WALL_MAP_DEFAULT e enviar o mapa
ao hardware (ou à co-simulação) via UART.

Modos de uso:
  python generate_maze.py          →  menu interativo no terminal
  python generate_maze.py --help   →  ajuda da CLI (modo não-interativo)

Exemplos de CLI:
  python generate_maze.py --seed 42 --save maze.npy --no-plot
  python generate_maze.py --load labirinto_bom.npy --send /dev/ttyUSB0 --start
  python generate_maze.py --seed 42 --send socket://127.0.0.1:5555 --start

Portas aceitas em --send (e no menu):
  /dev/ttyUSB0, COM3          FPGA real
  /tmp/vcom0                  ponte socat (test_fpga_hw_bridge.py, Linux)
  socket://127.0.0.1:5555     hub TCP da co-simulação (run_bridge_win.py)

Protocolo UART de upload (ver qlearning_top_v2.v):
  1. Envia 0xFF  → soft reset no hardware (restaura WALL_MAP_DEFAULT!)
  2. Aguarda 50 ms
  3. Envia 0xB0  → sinaliza início de upload de mapa
  4. Envia 128 bytes do WALL_MAP (byte 0 = bits[7:0], byte 127 = bits[1023:1016])
  5. map_ready sobe no hardware após o 128º byte (confirmação implícita)
  6. Envia 0xA0  → start de treinamento (opcional)

Nota sobre a abertura da porta: _open_porta_sem_flush() neutraliza o
tcflush(TCIFLUSH) do pyserial em PTYs POSIX compartilhados, para não apagar
telemetria ainda não lida pelo qlearning_visualizer.py (a fila de entrada de
um TTY pertence ao dispositivo, não ao file descriptor).

Ferramentas companheiras:
  qlearning_visualizer.py  visualização/telemetria do treinamento
  edit_maze.py             editor gráfico de .npy (clique para alternar parede)
  converter_npy.py         imagem PNG de labirinto → .npy (OpenCV)
  plot_maze.py             plota o WALL_MAP hex hardcoded do maze_env.v

Dependências:
  pip install numpy matplotlib pyserial
"""

import argparse
import os
import sys
import time
import glob

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


# ══════════════════════════════════════════════════════════════════════════════
# Utilitários de terminal
# ══════════════════════════════════════════════════════════════════════════════

# Códigos ANSI
_RESET  = "\033[0m"
_BOLD   = "\033[1m"
_DIM    = "\033[2m"
_GREEN  = "\033[32m"
_CYAN   = "\033[36m"
_YELLOW = "\033[33m"
_RED    = "\033[31m"
_WHITE  = "\033[97m"
_BG_DARK = "\033[48;5;235m"

def _supports_color() -> bool:
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()

def c(text: str, *codes: str) -> str:
    """Aplica códigos ANSI apenas se o terminal suportar."""
    if not _supports_color():
        return text
    return "".join(codes) + text + _RESET

def clear():
    os.system("cls" if os.name == "nt" else "clear")

def pause(msg: str = "  Pressione Enter para continuar..."):
    input(c(f"\n{msg}", _DIM))

def ask(prompt: str, default: str = "") -> str:
    """Input com valor padrão exibido."""
    default_hint = c(f" [{default}]", _DIM) if default else ""
    try:
        val = input(c("  ▸ ", _CYAN) + prompt + default_hint + ": ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return default
    return val if val else default

def ask_int(prompt: str, default: int, lo: int | None = None, hi: int | None = None) -> int:
    while True:
        raw = ask(prompt, str(default))
        try:
            v = int(raw)
            if lo is not None and v < lo:
                print(c(f"  Valor mínimo: {lo}", _RED)); continue
            if hi is not None and v > hi:
                print(c(f"  Valor máximo: {hi}", _RED)); continue
            return v
        except ValueError:
            print(c("  Entrada inválida, tente um número inteiro.", _RED))

def ask_float(prompt: str, default: float, lo: float = 0.0, hi: float = 1.0) -> float:
    while True:
        raw = ask(prompt, f"{default:.2f}")
        try:
            v = float(raw)
            if not (lo <= v <= hi):
                print(c(f"  Valor deve estar entre {lo} e {hi}.", _RED)); continue
            return v
        except ValueError:
            print(c("  Entrada inválida, tente um número decimal.", _RED))

def ask_yn(prompt: str, default: bool = False) -> bool:
    hint = "S/n" if default else "s/N"
    raw = ask(f"{prompt} [{hint}]", "").lower()
    if raw in ("s", "sim", "y", "yes"):
        return True
    if raw in ("n", "não", "nao", "no"):
        return False
    return default

def header(title: str):
    w = 62
    bar = c("═" * w, _CYAN)
    print(f"\n{bar}")
    print(c(f"  {title}", _BOLD + _WHITE))
    print(f"{bar}")

def section(title: str):
    print(c(f"\n  ── {title} ──", _YELLOW + _BOLD))

def ok(msg: str):
    print(c(f"  ✔  {msg}", _GREEN))

def info(msg: str):
    print(c(f"  ℹ  {msg}", _CYAN))

def warn(msg: str):
    print(c(f"  ⚠  {msg}", _YELLOW))

def err(msg: str):
    print(c(f"  ✘  {msg}", _RED))

def progress_bar(current: int, total: int, width: int = 38) -> str:
    filled = int(width * current / total)
    bar    = "█" * filled + "░" * (width - filled)
    pct    = 100 * current // total
    return c(f"[{bar}]", _CYAN) + c(f" {pct:3d}%", _BOLD)


# ══════════════════════════════════════════════════════════════════════════════
# Lógica de labirinto
# ══════════════════════════════════════════════════════════════════════════════

def generate_maze(wall_prob: float = 0.27, seed: int | None = None) -> np.ndarray:
    rng  = np.random.default_rng(seed)
    maze = rng.choice([0, 1], size=(32, 32),
                      p=[1 - wall_prob, wall_prob]).astype(np.uint8)
    maze[0,  0]  = 0
    maze[31, 31] = 0
    return maze

def load_maze(path: str) -> np.ndarray:
    maze = np.load(path).astype(np.uint8)
    if maze.shape != (32, 32):
        raise ValueError(f"Esperado 32×32, encontrado {maze.shape}")
    maze[0,  0]  = 0
    maze[31, 31] = 0
    return maze

def maze_to_bytes(maze: np.ndarray) -> bytes:
    bits = [maze[bit_idx // 32, bit_idx % 32] for bit_idx in range(1024)]
    result = bytearray(128)
    for byte_idx in range(128):
        byte_val = 0
        for b in range(8):
            byte_val |= (int(bits[byte_idx * 8 + b]) << b)
        result[byte_idx] = byte_val
    return bytes(result)

def maze_to_hex(maze: np.ndarray) -> str:
    raw = maze_to_bytes(maze)
    val = int.from_bytes(reversed(raw), byteorder="big")
    return hex(val)[2:].upper().zfill(256)

def bytes_to_maze(data: bytes) -> np.ndarray:
    maze = np.zeros((32, 32), dtype=np.uint8)
    for byte_idx, byte_val in enumerate(data):
        for b in range(8):
            bit_pos = byte_idx * 8 + b
            maze[bit_pos // 32, bit_pos % 32] = (byte_val >> b) & 1
    return maze

def maze_stats(maze: np.ndarray) -> dict:
    walls = int(maze.sum())
    return {
        "paredes": walls,
        "livres":  1024 - walls,
        "pct":     f"{100 * walls / 1024:.1f}%",
    }


# ══════════════════════════════════════════════════════════════════════════════
# Visualização
# ══════════════════════════════════════════════════════════════════════════════

def plot_maze(maze: np.ndarray, title: str, filename: str | None = None,
              show: bool = True) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(maze, cmap="binary", vmin=0, vmax=1)
    ax.plot(0,  0,  "go", markersize=11, label="Início (0,0)",  zorder=5)
    ax.plot(31, 31, "ro", markersize=11, label="Meta  (31,31)", zorder=5)
    ax.grid(True, which="both", color="lightgrey", linewidth=0.5)
    ax.set_xticks(np.arange(-0.5, 32, 1)); ax.set_xticklabels([])
    ax.set_yticks(np.arange(-0.5, 32, 1)); ax.set_yticklabels([])
    ax.legend(loc="upper right", fontsize=9)
    ax.set_title(title, fontsize=13)
    fig.tight_layout()
    if filename:
        fig.savefig(filename, bbox_inches="tight", dpi=150)
        ok(f"Imagem salva em '{filename}'")
    if show:
        plt.show()
    plt.close(fig)

def print_ascii_maze(maze: np.ndarray):
    """Imprime o labirinto 32×32 como arte ASCII no terminal."""
    section("Pré-visualização ASCII (█ = parede, · = livre)")
    print()
    for row in range(32):
        row_str = "  "
        for col in range(32):
            if row == 0 and col == 0:
                row_str += c("S", _GREEN)
            elif row == 31 and col == 31:
                row_str += c("G", _RED)
            elif maze[row, col]:
                row_str += c("█", _DIM)
            else:
                row_str += c("·", _WHITE)
        print(row_str)
    print()


# ══════════════════════════════════════════════════════════════════════════════
# UART
# ══════════════════════════════════════════════════════════════════════════════

def _get_serial():
    try:
        import serial
        import serial.tools.list_ports
        return serial, serial.tools.list_ports
    except ImportError:
        return None, None

def list_serial_ports() -> list[str]:
    _, list_ports = _get_serial()
    if list_ports is None:
        return []
    return [p.device for p in list_ports.comports()]

def _open_porta_sem_flush(serial_mod, port: str, baud: int, timeout: float):
    """Abre a porta serial de forma portátil e SEM efeitos colaterais.

    Três casos:
    1. URL pyserial (ex.: socket://127.0.0.1:5555) — modo Windows/co-simulação
       via hub TCP (test_fpga_hw_bridge_win.py). Cada cliente tem sua própria
       conexão, então não existe fila compartilhada para proteger.
    2. Porta POSIX (/dev/ttyUSB0, /tmp/vcom0) — neutraliza o tcflush(TCIFLUSH)
       que o Serial.open() do pyserial executa: no Linux a fila de entrada de
       um TTY/PTY pertence ao DISPOSITIVO, não ao file descriptor, e o flush
       apagaria bytes de telemetria ainda não lidos pelo
       qlearning_visualizer.py quando os dois compartilham o mesmo PTY.
    3. Porta Windows (COM3 etc.) — o SO já garante acesso exclusivo, sem
       fila compartilhada; abre normalmente.
    """
    if "://" in port:
        # baud é irrelevante num transporte TCP, mas o handler aceita a URL pura
        return serial_mod.serial_for_url(port, timeout=timeout)

    ser = serial_mod.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    if os.name == "posix":
        # Neutraliza apenas o flush automático do open(); reset_input_buffer()
        # público continua disponível se algum dia for chamado de propósito.
        ser._reset_input_buffer = lambda: None
    ser.open()
    return ser


def send_maze_uart(maze: np.ndarray, port: str, baud: int,
                   send_start: bool, verbose: bool = True) -> bool:
    serial_mod, _ = _get_serial()
    if serial_mod is None:
        err("pyserial não encontrado. Instale com: pip install pyserial")
        return False

    payload = maze_to_bytes(maze)

    try:
        if verbose:
            info(f"Abrindo {port} @ {baud} baud ...")
        with _open_porta_sem_flush(serial_mod, port, baud, timeout=2) as ser:
            time.sleep(0.1)

            # 1. Soft reset
            if verbose: info("Enviando 0xFF  →  soft reset ...")
            ser.write(bytes([0xFF]))
            time.sleep(0.05)

            # 2. Início de upload
            if verbose: info("Enviando 0xB0  →  início de upload de mapa ...")
            ser.write(bytes([0xB0]))
            time.sleep(0.005)

            # 3. 128 bytes do mapa com barra de progresso
            CHUNK = 16
            if verbose: print()
            for i in range(0, 128, CHUNK):
                ser.write(payload[i:i+CHUNK])
                time.sleep(CHUNK * 10 / baud + 0.002)
                if verbose:
                    sent = i + CHUNK
                    bar  = progress_bar(min(sent, 128), 128)
                    print(f"\r  Enviando mapa  {bar}  {min(sent,128):3d}/128 bytes",
                          end="", flush=True)
            if verbose:
                print(f"\r  Enviando mapa  {progress_bar(128, 128)}  128/128 bytes")
                print()
                ok("Mapa enviado com sucesso!")

            time.sleep(0.02)

            # 4. Start (opcional)
            if send_start:
                if verbose: info("Enviando 0xA0  →  start de treinamento ...")
                ser.write(bytes([0xA0]))
                if verbose: ok("Treinamento iniciado no hardware.")

        return True

    except serial_mod.SerialException as exc:
        err(f"Erro UART: {exc}")
        return False


# ══════════════════════════════════════════════════════════════════════════════
# Sub-menus
# ══════════════════════════════════════════════════════════════════════════════

# Estado persistente durante a sessão
_session = {
    "maze":     None,   # ndarray atual
    "source":   "",     # descrição da origem
    "port":     "",     # última porta UART usada
    "baud":     115200,
}


# ── 1. Gerar novo labirinto ───────────────────────────────────────────────────

def menu_gerar():
    clear()
    header("Gerar novo labirinto aleatório")

    section("Parâmetros")
    wall_prob = ask_float("Probabilidade de parede (0.0–0.60)", default=0.27,
                          lo=0.0, hi=0.60)
    use_seed  = ask_yn("Usar semente fixa (reprodutível)?", default=False)
    seed      = None
    if use_seed:
        seed = ask_int("Semente (inteiro)", default=42)

    section("Gerando")
    maze = generate_maze(wall_prob=wall_prob, seed=seed)
    st   = maze_stats(maze)
    ok(f"Labirinto gerado  —  {st['paredes']} paredes ({st['pct']}), {st['livres']} células livres")

    _session["maze"]   = maze
    _session["source"] = f"gerado  wall_prob={wall_prob:.2f}  seed={seed}"

    # Pré-visualização ASCII
    show_ascii = ask_yn("Exibir pré-visualização ASCII?", default=True)
    if show_ascii:
        print_ascii_maze(maze)

    pause()


# ── 2. Carregar labirinto de arquivo ─────────────────────────────────────────

def menu_carregar():
    clear()
    header("Carregar labirinto de arquivo .npy")

    # Lista arquivos .npy na pasta atual
    npys = sorted(glob.glob("*.npy"))
    if npys:
        section("Arquivos encontrados na pasta atual")
        for i, f in enumerate(npys, 1):
            print(c(f"  [{i}] ", _CYAN) + f)
        print()

    path = ask("Caminho do arquivo .npy", default=npys[0] if npys else "maze.npy")
    if not os.path.exists(path):
        err(f"Arquivo não encontrado: {path}")
        pause(); return

    try:
        maze = load_maze(path)
    except Exception as exc:
        err(f"Erro ao carregar: {exc}")
        pause(); return

    st = maze_stats(maze)
    ok(f"Carregado '{path}'  —  {st['paredes']} paredes ({st['pct']})")

    _session["maze"]   = maze
    _session["source"] = f"arquivo '{path}'"

    show_ascii = ask_yn("Exibir pré-visualização ASCII?", default=True)
    if show_ascii:
        print_ascii_maze(maze)

    pause()


# ── 3. Visualizar labirinto atual ─────────────────────────────────────────────

def menu_visualizar():
    clear()
    header("Visualizar labirinto")

    if _session["maze"] is None:
        warn("Nenhum labirinto carregado. Gere ou carregue um primeiro.")
        pause(); return

    maze = _session["maze"]

    section("Opções de visualização")
    print(c("  [1]", _CYAN) + "  Janela gráfica (Matplotlib)")
    print(c("  [2]", _CYAN) + "  ASCII no terminal")
    print(c("  [3]", _CYAN) + "  Ambos")
    escolha = ask("Escolha", default="1")

    if escolha in ("1", "3"):
        fname = ask("Nome do arquivo para salvar imagem (Enter = não salvar)", default="")
        fname = fname if fname else None
        title = f"Labirinto 32×32  ({_session['source']})"
        plot_maze(maze, title=title, filename=fname, show=True)

    if escolha in ("2", "3"):
        print_ascii_maze(maze)

    pause()


# ── 4. Exportar hex Verilog ──────────────────────────────────────────────────

def menu_exportar_hex():
    clear()
    header("Exportar string hex para Verilog")

    if _session["maze"] is None:
        warn("Nenhum labirinto carregado. Gere ou carregue um primeiro.")
        pause(); return

    hex_str = maze_to_hex(_session["maze"])

    section("Parâmetro Verilog")
    print()
    print(c("  parameter [1023:0] WALL_MAP_DEFAULT = ", _DIM)
          + c(f"1024'h{hex_str}", _GREEN) + c(";", _DIM))
    print()

    # Verificação round-trip
    raw       = maze_to_bytes(_session["maze"])
    recovered = bytes_to_maze(raw)
    if np.array_equal(_session["maze"], recovered):
        ok("Verificação round-trip  maze→bytes→maze  OK")
    else:
        err("Verificação round-trip FALHOU — não use este hex!")

    # Salvar em arquivo de texto
    save = ask_yn("Salvar em arquivo .txt?", default=False)
    if save:
        fname = ask("Nome do arquivo", default="wall_map.txt")
        with open(fname, "w") as f:
            f.write(f"parameter [1023:0] WALL_MAP_DEFAULT = 1024'h{hex_str};\n")
        ok(f"Salvo em '{fname}'")

    pause()


# ── 5. Salvar labirinto em .npy ───────────────────────────────────────────────

def menu_salvar_npy():
    clear()
    header("Salvar labirinto em arquivo .npy")

    if _session["maze"] is None:
        warn("Nenhum labirinto carregado. Gere ou carregue um primeiro.")
        pause(); return

    fname = ask("Nome do arquivo", default="maze.npy")
    if not fname.endswith(".npy"):
        fname += ".npy"

    if os.path.exists(fname):
        if not ask_yn(f"'{fname}' já existe. Sobrescrever?", default=False):
            info("Operação cancelada.")
            pause(); return

    np.save(fname, _session["maze"])
    ok(f"Labirinto salvo em '{fname}'")
    pause()


# ── 6. Configurar UART ────────────────────────────────────────────────────────

def menu_configurar_uart():
    clear()
    header("Configurar porta UART")

    serial_mod, _ = _get_serial()
    if serial_mod is None:
        warn("pyserial não instalado. Instale com: pip install pyserial")
        pause(); return

    section("Portas disponíveis no sistema")
    ports = list_serial_ports()
    if ports:
        for i, p in enumerate(ports, 1):
            marker = c(" ◄ atual", _GREEN) if p == _session["port"] else ""
            print(c(f"  [{i}] ", _CYAN) + p + marker)
    else:
        warn("Nenhuma porta serial detectada.")

    print()
    default_port = _session["port"] if _session["port"] else (ports[0] if ports else "/dev/ttyUSB0")
    port = ask("Porta serial", default=default_port)
    baud = ask_int("Baud rate", default=_session["baud"],
                   lo=1200, hi=3000000)

    _session["port"] = port
    _session["baud"] = baud
    ok(f"Configurado: {port} @ {baud} baud")
    pause()


# ── 7. Enviar ao hardware ─────────────────────────────────────────────────────

def menu_enviar():
    clear()
    header("Enviar mapa ao hardware via UART")

    if _session["maze"] is None:
        warn("Nenhum labirinto carregado. Gere ou carregue um primeiro.")
        pause(); return

    serial_mod, _ = _get_serial()
    if serial_mod is None:
        warn("pyserial não instalado. Instale com: pip install pyserial")
        pause(); return

    section("Destino UART")

    # Mostra portas disponíveis
    ports = list_serial_ports()
    if ports:
        for i, p in enumerate(ports, 1):
            marker = c(" ◄ padrão", _GREEN) if p == _session["port"] else ""
            print(c(f"  [{i}] ", _CYAN) + p + marker)
        print()

    default_port = _session["port"] if _session["port"] else (ports[0] if ports else "/dev/ttyUSB0")
    port = ask("Porta serial", default=default_port)
    baud = ask_int("Baud rate", default=_session["baud"], lo=1200, hi=3000000)

    _session["port"] = port
    _session["baud"] = baud

    section("Opções de envio")
    send_start = ask_yn("Enviar 0xA0 (iniciar treinamento) após o upload?", default=False)

    st = maze_stats(_session["maze"])
    section("Resumo")
    info(f"Origem  : {_session['source']}")
    info(f"Mapa    : {st['paredes']} paredes ({st['pct']})")
    info(f"Destino : {port} @ {baud} baud")
    info(f"Start   : {'sim' if send_start else 'não'}")
    print()

    if not ask_yn("Confirmar envio?", default=True):
        info("Operação cancelada.")
        pause(); return

    print()
    success = send_maze_uart(_session["maze"], port=port, baud=baud,
                             send_start=send_start, verbose=True)
    if success:
        ok("Operação concluída com sucesso.")
    else:
        err("Falha no envio. Verifique a conexão e tente novamente.")

    pause()


# ── 8. Fluxo rápido: gerar + enviar ─────────────────────────────────────────

def menu_fluxo_rapido():
    clear()
    header("Fluxo rápido  —  Gerar → Visualizar → Enviar")

    section("1 · Geração do labirinto")
    wall_prob = ask_float("Probabilidade de parede", default=0.27, lo=0.0, hi=0.60)
    use_seed  = ask_yn("Usar semente fixa?", default=False)
    seed      = ask_int("Semente", default=42) if use_seed else None

    maze = generate_maze(wall_prob=wall_prob, seed=seed)
    st   = maze_stats(maze)
    ok(f"{st['paredes']} paredes ({st['pct']}), {st['livres']} livres")

    _session["maze"]   = maze
    _session["source"] = f"gerado  wall_prob={wall_prob:.2f}  seed={seed}"

    # Pré-visualização rápida ASCII
    print_ascii_maze(maze)

    section("2 · Visualização gráfica (opcional)")
    if ask_yn("Abrir janela gráfica?", default=False):
        plot_maze(maze, title=f"Labirinto 32×32  (wall_prob={wall_prob:.2f})",
                  filename=None, show=True)

    section("3 · Envio UART")
    serial_mod, _ = _get_serial()
    if serial_mod is None:
        warn("pyserial não instalado — pulando envio UART.")
    else:
        ports = list_serial_ports()
        if ports:
            for i, p in enumerate(ports, 1):
                print(c(f"  [{i}] ", _CYAN) + p)
            print()
        default_port = _session["port"] if _session["port"] else (ports[0] if ports else "/dev/ttyUSB0")
        port       = ask("Porta serial", default=default_port)
        baud       = ask_int("Baud rate", default=_session["baud"], lo=1200, hi=3000000)
        send_start = ask_yn("Iniciar treinamento automaticamente após upload?", default=True)

        _session["port"] = port
        _session["baud"] = baud

        print()
        send_maze_uart(maze, port=port, baud=baud,
                       send_start=send_start, verbose=True)

    pause()


# ══════════════════════════════════════════════════════════════════════════════
# Menu principal
# ══════════════════════════════════════════════════════════════════════════════

def _status_bar() -> str:
    if _session["maze"] is None:
        maze_info = c("nenhum labirinto carregado", _DIM)
    else:
        st = maze_stats(_session["maze"])
        maze_info = (c(_session["source"], _GREEN) +
                     c(f"  [{st['paredes']} paredes · {st['pct']}]", _DIM))

    port_info = (c(_session["port"], _CYAN) + c(f" @ {_session['baud']}", _DIM)
                 if _session["port"] else c("não configurada", _DIM))

    return (f"\n  Labirinto : {maze_info}\n"
            f"  UART      : {port_info}")


MENU_ITEMS = [
    ("1", "Gerar novo labirinto aleatório",              menu_gerar),
    ("2", "Carregar labirinto de arquivo .npy",          menu_carregar),
    ("3", "Visualizar labirinto atual",                  menu_visualizar),
    ("4", "Exportar parâmetro Verilog (hex)",            menu_exportar_hex),
    ("5", "Salvar labirinto em .npy",                    menu_salvar_npy),
    ("─", None,                                          None),
    ("6", "Configurar porta UART",                       menu_configurar_uart),
    ("7", "Enviar mapa ao hardware via UART",            menu_enviar),
    ("─", None,                                          None),
    ("8", "Fluxo rápido  (Gerar → Visualizar → Enviar)", menu_fluxo_rapido),
    ("─", None,                                          None),
    ("0", "Sair",                                        None),
]


def main_menu():
    while True:
        clear()
        # Cabeçalho
        print(c("\n  ╔══════════════════════════════════════════════════════════╗", _CYAN))
        print(c("  ║", _CYAN) + c("   Q-Learning 32×32  —  Gerador de Labirintos            ", _BOLD + _WHITE) + c(" ║", _CYAN))
        print(c("  ╚══════════════════════════════════════════════════════════╝", _CYAN))

        # Status atual da sessão
        print(_status_bar())

        # Itens do menu
        print()
        for key, label, _ in MENU_ITEMS:
            if key == "─":
                print(c("  " + "─" * 52, _DIM))
            else:
                bullet = c(f"  [{key}]", _CYAN + _BOLD)
                print(f"{bullet}  {label}")
        print()

        escolha = ask("Opção", default="").strip()

        # Despacha
        action_map = {key: fn for key, _, fn in MENU_ITEMS if key != "─" and fn is not None}

        if escolha == "0":
            clear()
            print(c("\n  Até logo!\n", _GREEN + _BOLD))
            sys.exit(0)
        elif escolha in action_map:
            action_map[escolha]()
        else:
            warn("Opção inválida.")
            time.sleep(0.8)


# ══════════════════════════════════════════════════════════════════════════════
# CLI (modo não-interativo, mantido para uso em scripts)
# ══════════════════════════════════════════════════════════════════════════════

def run_cli(args):
    """Executa o fluxo original por linha de comando (sem menu)."""
    # Gera ou carrega
    if args.load:
        print(f"[MAZE] Carregando '{args.load}' ...")
        maze = load_maze(args.load)
        source_desc = f"carregado de {args.load}"
    else:
        print(f"[MAZE] Gerando labirinto (wall_prob={args.wall_prob}, seed={args.seed}) ...")
        maze = generate_maze(wall_prob=args.wall_prob, seed=args.seed)
        source_desc = f"{int(args.wall_prob*100)}% paredes, seed={args.seed}"

    st = maze_stats(maze)
    print(f"[MAZE] {st['paredes']} paredes ({st['pct']}), {st['livres']} livres")

    hex_str = maze_to_hex(maze)
    print("\n" + "="*66)
    print("Parâmetro Verilog:")
    print(f"  parameter [1023:0] WALL_MAP_DEFAULT = 1024'h{hex_str};")
    print("="*66 + "\n")

    raw = maze_to_bytes(maze)
    assert np.array_equal(maze, bytes_to_maze(raw)), "Round-trip falhou!"

    if args.save:
        np.save(args.save, maze)
        print(f"[SAVE] '{args.save}' salvo.")

    if not args.no_plot:
        plot_maze(maze, title=f"Labirinto 32×32  ({source_desc})",
                  filename="labirinto_gerado.png", show=True)

    if args.send:
        send_maze_uart(maze, port=args.send, baud=args.baud,
                       send_start=args.start, verbose=True)
    else:
        print("[INFO] Use --send <PORTA> para enviar ao hardware.")


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="Gerador de labirintos 32×32 para Q-Learning RTL",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Sem argumentos: abre o menu interativo no terminal.")

    src = ap.add_mutually_exclusive_group()
    src.add_argument("--wall-prob", type=float, default=0.27, metavar="P")
    src.add_argument("--load",      metavar="FILE.npy")

    ap.add_argument("--seed",    type=int,   default=None)
    ap.add_argument("--save",    metavar="FILE.npy", default=None)
    ap.add_argument("--send",    metavar="PORT",      default=None)
    ap.add_argument("--baud",    type=int,   default=115200)
    ap.add_argument("--start",   action="store_true")
    ap.add_argument("--no-plot", action="store_true")

    # Se nenhum argumento relevante foi passado → menu interativo
    import sys as _sys
    if len(_sys.argv) == 1:
        main_menu()
    else:
        run_cli(ap.parse_args())


if __name__ == "__main__":
    main()