"""
uart_bfm.py — Bus Functional Model de UART 8N1 para cocotb.

Espelha exatamente o protocolo de uart_tx.v / uart_rx.v:
    * 1 start bit (0)
    * 8 bits de dado, LSB primeiro
    * 1 stop bit (1)
    * idle = 1

`UartDriver` é injetado num sinal de entrada do DUT (i_Rx_Serial) e
`UartMonitor` observa um sinal de saída do DUT (o_Tx_Serial), decodificando
bytes conforme eles saem — sem nenhum arquivo intermediário: tudo é
entregue como bytes Python, em tempo real, via callback.
"""

from collections import deque

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge, Timer, Event, ReadOnly
from cocotb.clock import Clock


class _SimpleQueue:
    """Fila async mínima baseada em cocotb.Event — evita depender do event
    loop asyncio nativo, que no cocotb 2.x roda desacoplado de
    asyncio.get_running_loop() e quebra asyncio.Queue()."""

    def __init__(self):
        self._items = deque()
        self._event = Event()

    async def put(self, item):
        self._items.append(item)
        self._event.set()

    async def get(self):
        while not self._items:
            self._event.clear()
            await self._event.wait()
        return self._items.popleft()


class UartDriver:
    """Injeta bytes no pino serial de entrada do DUT (ex.: dut.i_Rx_Serial)."""

    def __init__(self, dut, clk, rx_signal, clks_per_bit: int):
        self.dut = dut
        self.clk = clk
        self.rx_signal = rx_signal
        self.clks_per_bit = clks_per_bit
        self.rx_signal.value = 1  # idle = mark

    async def send(self, byte: int):
        """await uart_driver.send(0xB0) — envia 1 byte, bloqueia até o fim do stop bit."""
        assert 0 <= byte <= 0xFF

        # start bit
        self.rx_signal.value = 0
        await ClockCycles(self.clk, self.clks_per_bit)

        # 8 bits de dado, LSB primeiro
        for bit_idx in range(8):
            self.rx_signal.value = (byte >> bit_idx) & 1
            await ClockCycles(self.clk, self.clks_per_bit)

        # stop bit
        self.rx_signal.value = 1
        await ClockCycles(self.clk, self.clks_per_bit)

    async def send_many(self, data: bytes):
        for b in data:
            await self.send(b)


class UartMonitor:
    """
    Observa um pino serial de saída do DUT (ex.: dut.o_Tx_Serial) e decodifica
    bytes conforme eles chegam, empilhando-os numa asyncio.Queue interna.

    Uso:
        mon = UartMonitor(dut, dut.clk, dut.o_Tx_Serial, CLKS_PER_BIT)
        cocotb.start_soon(mon.start())
        byte = await mon.get_byte()
    """

    def __init__(self, dut, clk, tx_signal, clks_per_bit: int, on_byte=None):
        self.dut = dut
        self.clk = clk
        self.tx_signal = tx_signal
        self.clks_per_bit = clks_per_bit
        self.on_byte = on_byte          # callback síncrono opcional: on_byte(int)
        self._queue = _SimpleQueue()

    async def start(self):
        """Coroutine de fundo: fica sempre esperando a borda de descida (start bit)."""
        while True:
            # idle -> aguarda start bit (queda de 1 para 0)
            await self._wait_falling(self.tx_signal)

            # centraliza a amostragem no meio de cada bit
            await ClockCycles(self.clk, self.clks_per_bit // 2)
            # confirma que ainda é start bit
            if await self._sample() != 0:
                continue

            await ClockCycles(self.clk, self.clks_per_bit)  # pula pro meio do bit 0

            byte_val = 0
            for bit_idx in range(8):
                byte_val |= (await self._sample() & 1) << bit_idx
                await ClockCycles(self.clk, self.clks_per_bit)

            # aqui estamos no meio do stop bit; não precisa checar, só entregar
            await self._queue.put(byte_val)
            if self.on_byte is not None:
                self.on_byte(byte_val)

    async def _sample(self) -> int:
        """Lê o sinal só depois que os deltas de simulação assentaram
        (fase ReadOnly), evitando ler valores pré-atualização logo após
        uma borda de clock — a causa raiz da corrupção de bytes observada
        ao ler diretamente após RisingEdge/ClockCycles."""
        await ReadOnly()
        return int(self.tx_signal.value)

    async def _wait_falling(self, sig):
        prev = await self._sample()
        while True:
            await RisingEdge(self.clk)
            cur = await self._sample()
            if prev == 1 and cur == 0:
                return
            prev = cur

    async def get_byte(self, timeout_cycles=None):
        return await self._queue.get()

    async def get_packet(self, n: int):
        """Espera n bytes e devolve como `bytes`."""
        out = bytearray()
        for _ in range(n):
            out.append(await self.get_byte())
        return bytes(out)
