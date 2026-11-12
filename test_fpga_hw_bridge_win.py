"""
test_fpga_hw_bridge_win.py — "Modo FPGA" multiplataforma (Windows/Linux/macOS).

Mesma ideia do test_fpga_hw_bridge.py, mas SEM socat/PTY: a própria simulação
sobe um HUB TCP em localhost. Qualquer script pyserial conecta usando a URL
`socket://127.0.0.1:5555` — o pyserial trata isso como uma porta serial comum
(mesma API: read/write/in_waiting/timeout), então generate_maze.py e
qlearning_visualizer.py funcionam sem mudanças de lógica:

    python generate_maze.py            (porta: socket://127.0.0.1:5555)
    python qlearning_visualizer.py --port socket://127.0.0.1:5555

Vantagens sobre o par de PTYs do Linux:
  * Funciona no Windows nativamente (não precisa de com0com nem driver).
  * Cada cliente tem SUA conexão TCP — o hub replica a saída do DUT para
    todos e mescla as entradas. Acabou o problema de dois processos
    disputando a mesma fila de entrada de um TTY compartilhado.
  * Não existe symlink/PTY órfão: se a simulação morrer, os clientes
    recebem erro de conexão explícito na hora.

Arquitetura (threads + cocotb):

    generate_maze.py / qlearning_visualizer.py   (N clientes)
              │  pyserial socket://127.0.0.1:5555
              ▼
      HUB TCP (este arquivo, thread _accept_thread)
              │  1 thread leitora por cliente → rx_queue (mesclada)
              ▼
   cocotb coroutine (polling por ciclo) → UartDriver.send → pinos do DUT
              │
              ▼  UartMonitor → on_byte → tx_queue
      _writer_thread → broadcast para TODOS os clientes conectados

Variáveis de ambiente:
    SIM_TCP_HOST      (padrão 127.0.0.1)
    SIM_TCP_PORT      (padrão 5555)
    CLKS_PER_BIT      (padrão 4)
    BRIDGE_MAX_CYCLES (padrão 0 = roda para sempre; útil em CI/testes)
"""

import os
import queue
import socket
import sys
import threading

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, os.path.dirname(__file__))
from uart_bfm import UartDriver, UartMonitor

CLKS_PER_BIT = int(os.environ.get("CLKS_PER_BIT", "4"))
CLK_PERIOD_NS = 20  # 50 MHz
TCP_HOST = os.environ.get("SIM_TCP_HOST", "127.0.0.1")
TCP_PORT = int(os.environ.get("SIM_TCP_PORT", "5555"))
MAX_CYCLES = int(os.environ.get("BRIDGE_MAX_CYCLES", "0"))  # 0 = infinito


class TcpHub:
    """Servidor TCP que faz o papel do par socat: aceita N clientes pyserial
    (socket://), mescla tudo que eles enviam em rx_queue e replica cada byte
    do DUT (tx_queue) para todos os clientes conectados."""

    def __init__(self, host: str, port: int,
                 rx_queue: "queue.Queue[int]", stop_evt: threading.Event):
        self.rx_queue = rx_queue
        self.stop_evt = stop_evt
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()

        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(8)
        self.srv.settimeout(0.5)
        print(f"[hub] escutando em socket://{host}:{port} — aponte "
              f"generate_maze.py e qlearning_visualizer.py para essa URL",
              flush=True)
        threading.Thread(target=self._accept_thread, daemon=True).start()

    # ── aceitação de clientes ────────────────────────────────────────────
    def _accept_thread(self):
        n = 0
        while not self.stop_evt.is_set():
            try:
                conn, addr = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            n += 1
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._clients.append(conn)
            print(f"[hub] cliente #{n} conectado: {addr}", flush=True)
            threading.Thread(target=self._client_reader,
                             args=(conn, n), daemon=True).start()

    # ── 1 thread leitora por cliente → rx_queue mesclada ────────────────
    def _client_reader(self, conn: socket.socket, cid: int):
        conn.settimeout(0.5)
        total = 0
        while not self.stop_evt.is_set():
            try:
                data = conn.recv(256)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:  # cliente fechou (ex.: generate_maze terminou)
                break
            for b in data:
                total += 1
                self.rx_queue.put(b)
            # Eco para os OUTROS clientes: emula um barramento compartilhado.
            # É assim que o qlearning_visualizer.py "escuta" o upload de mapa
            # (0xB0 + 128 bytes) feito pelo generate_maze.py e consegue
            # desenhar o labirinto — sem isso, só o DUT veria esses bytes.
            self._broadcast_bytes(data, exclude=conn)
            print(f"[hub] cliente #{cid}: {len(data)} byte(s) recebidos "
                  f"(total {total})", flush=True)
        with self._lock:
            if conn in self._clients:
                self._clients.remove(conn)
        try:
            conn.close()
        except OSError:
            pass
        print(f"[hub] cliente #{cid} desconectado", flush=True)

    # ── broadcast DUT → todos os clientes ────────────────────────────────
    def broadcast(self, b: int):
        self._broadcast_bytes(bytes([b]))

    def _broadcast_bytes(self, data: bytes, exclude: socket.socket = None):
        dead = []
        with self._lock:
            for conn in self._clients:
                if conn is exclude:
                    continue
                try:
                    conn.sendall(data)
                except OSError:
                    dead.append(conn)
            for conn in dead:
                self._clients.remove(conn)
                try:
                    conn.close()
                except OSError:
                    pass

    def close(self):
        try:
            self.srv.close()
        except OSError:
            pass
        with self._lock:
            for conn in self._clients:
                try:
                    conn.close()
                except OSError:
                    pass
            self._clients.clear()


def _writer_thread(hub: TcpHub, byte_queue: "queue.Queue[int]",
                   stop_evt: threading.Event):
    """Drena bytes decodificados do DUT e replica para todos os clientes."""
    while not stop_evt.is_set():
        try:
            b = byte_queue.get(timeout=0.2)
        except queue.Empty:
            continue
        hub.broadcast(b)
        print(f"[writer_thread] byte 0x{b:02X} replicado aos clientes", flush=True)


@cocotb.test()
async def fpga_hw_bridge_win(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.i_Rx_Serial.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    uart_driver = UartDriver(dut, dut.clk, dut.i_Rx_Serial, CLKS_PER_BIT)
    uart_monitor = UartMonitor(dut, dut.clk, dut.o_Tx_Serial, CLKS_PER_BIT)

    rx_queue: "queue.Queue[int]" = queue.Queue()   # scripts externos -> DUT
    tx_queue: "queue.Queue[int]" = queue.Queue()   # DUT -> scripts externos
    stop_evt = threading.Event()

    hub = TcpHub(TCP_HOST, TCP_PORT, rx_queue, stop_evt)
    t_writer = threading.Thread(target=_writer_thread,
                                args=(hub, tx_queue, stop_evt), daemon=True)
    t_writer.start()

    def on_dut_byte(b: int):
        dut._log.info(f"[bridge] DUT emitiu byte via o_Tx_Serial: 0x{b:02X}")
        tx_queue.put(b)

    uart_monitor.on_byte = on_dut_byte
    cocotb.start_soon(uart_monitor.start())

    dut._log.info("Bridge TCP pronto. Aguardando conexões de "
                  "generate_maze.py / qlearning_visualizer.py ...")

    try:
        heartbeat = 0
        while MAX_CYCLES == 0 or heartbeat < MAX_CYCLES:
            await RisingEdge(dut.clk)
            heartbeat += 1
            if heartbeat % 500000 == 0:
                dut._log.info(f"[bridge] heartbeat, ainda esperando bytes "
                              f"(ciclo {heartbeat})")
            try:
                b = rx_queue.get_nowait()
            except queue.Empty:
                continue
            dut._log.info(f"[bridge] byte tirado da rx_queue: 0x{b:02X} — "
                          f"enviando ao DUT ...")
            await uart_driver.send(b)
            dut._log.info(f"[bridge] byte 0x{b:02X} entregue ao DUT com sucesso")
    finally:
        stop_evt.set()
        hub.close()