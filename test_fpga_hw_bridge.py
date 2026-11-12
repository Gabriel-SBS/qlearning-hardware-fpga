"""
test_fpga_hw_bridge.py — "Modo FPGA" com scripts REAIS e inalterados.

Diferente de test_fpga_mode.py (que importa generate_maze.py/qlearning_visualizer.py
como bibliotecas Python), este teste abre uma porta serial virtual (PTY criado
via socat, ver start_virtual_fpga.sh) e faz o bridge byte a byte para os pinos
do DUT — a simulação passa a se comportar, do ponto de vista de qualquer
processo pyserial externo, como se fosse a FPGA de verdade.

Isso permite rodar, sem tocar em uma linha de código:
    python3 generate_maze.py --send /tmp/vcom0 --start
    python3 qlearning_visualizer.py --port /tmp/vcom0

enquanto este teste roda do outro lado (/tmp/vcom1).

Arquitetura (threads + cocotb):

    generate_maze.py / qlearning_visualizer.py
              │  pyserial real (bytes crus)
              ▼
        /tmp/vcom0  <──socat──>  /tmp/vcom1
                                      │  pyserial (aberto por este arquivo)
                                      ▼
                          _reader_thread (bloqueante, OS thread)
                                      │  queue.Queue thread-safe
                                      ▼
                       cocotb coroutine faz polling a cada ciclo de clock
                                      │
                                      ▼  UartDriver.send(byte) -> pinos do DUT
                                DUT (qlearning_top_v2, simulado)
                                      │  UartMonitor -> on_byte callback
                                      ▼  queue.Queue thread-safe
                          _writer_thread (bloqueante, OS thread)
                                      │  pyserial real (bytes crus)
                                      ▼
                                 /tmp/vcom1  ──socat──>  /tmp/vcom0
"""

import os
import queue
import sys
import threading
import time

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

sys.path.insert(0, os.path.dirname(__file__))
from uart_bfm import UartDriver, UartMonitor

CLKS_PER_BIT = int(os.environ.get("CLKS_PER_BIT", "4"))
CLK_PERIOD_NS = 20  # 50 MHz
SERIAL_PORT = os.environ.get("SIM_SERIAL_PORT", "/tmp/vcom1")
SERIAL_BAUD = int(os.environ.get("SIM_SERIAL_BAUD", "115200"))


class _SerialHolder:
    """Dono único do handle pyserial do lado /tmp/vcom1, com reconexão.

    Problema que isto resolve: /tmp/vcom1 é um SYMLINK para um /dev/pts/N
    criado pelo socat. Se o start_virtual_fpga.sh for reiniciado com a
    simulação viva, o symlink passa a apontar para um pts NOVO, enquanto o
    file descriptor desta simulação continua preso ao pts ANTIGO (órfão).
    Nesse estado, read() levanta SerialException/EIO — e a versão antiga
    deste arquivo simplesmente deixava a thread leitora MORRER com uma
    única linha de log, produzindo o clássico "heartbeat, ainda esperando
    bytes" eterno, enquanto generate_maze.py escrevia com sucesso no par
    novo. Agora: qualquer erro de I/O fecha o handle e reabre o symlink
    (resolvendo o alvo de novo), religando a ponte ao par vivo.
    """

    def __init__(self, port: str, baud: int):
        import serial as _serial
        self._serial_mod = _serial
        self._port = port
        self._baud = baud
        self._lock = threading.Lock()
        self.ser = None
        self._open(first=True)

    def _open(self, first: bool = False):
        real = os.path.realpath(self._port)
        self.ser = self._serial_mod.Serial(self._port, self._baud, timeout=0.2)
        tag = "aberto" if first else "REABERTO"
        print(f"[bridge/serial] {self._port} {tag} (alvo real: {real})", flush=True)

    def reopen(self, stop_evt: threading.Event, who: str, exc: Exception):
        with self._lock:
            print(
                f"[{who}] *** ERRO DE I/O na porta serial: {exc}\n"
                f"[{who}] *** Causa típica: o par socat (start_virtual_fpga.sh) "
                f"morreu ou foi REINICIADO e este processo ficou preso num PTY "
                f"órfão. Tentando religar em {self._port} ...",
                flush=True,
            )
            try:
                self.ser.close()
            except Exception:
                pass
            while not stop_evt.is_set():
                time.sleep(1.0)
                try:
                    self._open()
                    return
                except Exception:
                    continue


def _reader_thread(holder: "_SerialHolder", byte_queue: "queue.Queue[int]",
                   stop_evt: threading.Event):
    """OS thread bloqueante: lê 1 byte por vez da serial real e empilha.
    Nunca morre por erro de I/O — reabre a porta e continua."""
    n = 0
    while not stop_evt.is_set():
        try:
            data = holder.ser.read(1)  # bloqueia até 1 byte ou timeout
        except Exception as exc:
            holder.reopen(stop_evt, "reader_thread", exc)
            continue
        if data:
            n += 1
            print(f"[reader_thread] byte #{n} recebido do SO: 0x{data[0]:02X}", flush=True)
            byte_queue.put(data[0])


def _writer_thread(holder: "_SerialHolder", byte_queue: "queue.Queue[int]",
                   stop_evt: threading.Event):
    """OS thread bloqueante: drena bytes decodificados do DUT e escreve na
    serial real. Em erro de I/O, reabre e reenvia o byte (não descarta)."""
    while not stop_evt.is_set():
        try:
            b = byte_queue.get(timeout=0.2)
        except queue.Empty:
            continue
        while not stop_evt.is_set():
            try:
                holder.ser.write(bytes([b]))
                holder.ser.flush()
                print(f"[writer_thread] byte 0x{b:02X} escrito no SO (vcom1)", flush=True)
                break
            except Exception as exc:
                holder.reopen(stop_evt, "writer_thread", exc)


@cocotb.test()
async def fpga_hw_bridge(dut):
    import serial  # pyserial — só é importado aqui, no lado "simulador"

    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.i_Rx_Serial.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    uart_driver = UartDriver(dut, dut.clk, dut.i_Rx_Serial, CLKS_PER_BIT)
    uart_monitor = UartMonitor(dut, dut.clk, dut.o_Tx_Serial, CLKS_PER_BIT)

    dut._log.info(f"Abrindo porta serial virtual: {SERIAL_PORT} @ {SERIAL_BAUD}")
    holder = _SerialHolder(SERIAL_PORT, SERIAL_BAUD)

    rx_queue: "queue.Queue[int]" = queue.Queue()   # bytes vindos do script externo -> DUT
    tx_queue: "queue.Queue[int]" = queue.Queue()   # bytes vindos do DUT -> script externo
    stop_evt = threading.Event()

    t_reader = threading.Thread(target=_reader_thread, args=(holder, rx_queue, stop_evt), daemon=True)
    t_writer = threading.Thread(target=_writer_thread, args=(holder, tx_queue, stop_evt), daemon=True)
    t_reader.start()
    t_writer.start()

    def _symlink_watchdog():
        """Detecta troca do par socat mesmo SEM erro de leitura pendente:
        se o alvo real do symlink /tmp/vcom1 mudar em relação ao fd aberto,
        força a religação. (Sem isso, um fd órfão que só recebe timeouts
        — sem EIO — esperaria para sempre em silêncio.)"""
        import stat as _stat
        while not stop_evt.is_set():
            time.sleep(2.0)
            try:
                fd_dev = os.fstat(holder.ser.fd).st_rdev
                ln_dev = os.stat(os.path.realpath(SERIAL_PORT)).st_rdev
                if fd_dev != ln_dev:
                    raise OSError(
                        f"symlink {SERIAL_PORT} agora aponta para outro PTY "
                        f"(par socat foi recriado)")
            except Exception as exc:
                holder.reopen(stop_evt, "symlink_watchdog", exc)

    threading.Thread(target=_symlink_watchdog, daemon=True).start()

    def on_dut_byte(b: int):
        # chamado dentro da coroutine do monitor (mesma thread da simulação);
        # só empilha — quem escreve na serial de verdade é a t_writer.
        dut._log.info(f"[bridge] DUT emitiu byte via o_Tx_Serial: 0x{b:02X}")
        tx_queue.put(b)

    uart_monitor.on_byte = on_dut_byte
    cocotb.start_soon(uart_monitor.start())

    dut._log.info("Bridge pronto. Aguardando bytes de generate_maze.py / qlearning_visualizer.py ...")

    # Loop principal: a cada ciclo de clock, verifica se chegou byte novo do
    # script externo e injeta no DUT via UartDriver (bit-bang síncrono ao clk
    # simulado). O envio de 1 byte trava este loop por ~10*CLKS_PER_BIT
    # ciclos, exatamente como aconteceria numa UART real.
    try:
        heartbeat = 0
        while True:
            await RisingEdge(dut.clk)
            heartbeat += 1
            if heartbeat % 500000 == 0:
                dut._log.info(f"[bridge] heartbeat, ainda esperando bytes (ciclo {heartbeat})")
            try:
                b = rx_queue.get_nowait()
            except queue.Empty:
                continue
            dut._log.info(f"[bridge] byte tirado da rx_queue: 0x{b:02X} — enviando ao DUT ...")
            await uart_driver.send(b)
            dut._log.info(f"[bridge] byte 0x{b:02X} entregue ao DUT com sucesso")
    finally:
        stop_evt.set()
        holder.ser.close()