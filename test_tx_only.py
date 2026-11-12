import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, ReadOnly

from uart_bfm import UartMonitor

CLKS_PER_BIT = 8


@cocotb.test()
async def tx_only(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.rst_n.value = 0
    dut.i_Tx_DV.value = 0
    dut.i_Tx_Byte.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    monitor = UartMonitor(dut, dut.clk, dut.o_Tx_Serial, CLKS_PER_BIT)
    cocotb.start_soon(monitor.start())
    await ClockCycles(dut.clk, 3)

    test_bytes = [0x00, 0x5F, 0x87, 0x00, 0x89, 0x87, 0x01, 0x3D]

    async def sender():
        for b in test_bytes:
            dut.i_Tx_Byte.value = b
            dut.i_Tx_DV.value = 1
            await RisingEdge(dut.clk)
            dut.i_Tx_DV.value = 0
            # aguarda o_Tx_Done, exatamente como a FSM do top faz (back-to-back)
            while int(dut.o_Tx_Done.value) == 0:
                await RisingEdge(dut.clk)

    cocotb.start_soon(sender())

    from cocotb.triggers import Timer

    got = []
    for i in range(len(test_bytes)):
        try:
            got.append(await cocotb.triggers.with_timeout(monitor.get_byte(), 2000, "ns"))
        except cocotb.triggers.SimTimeoutError:
            dut._log.error(f"TIMEOUT waiting for byte {i}, got so far={got}")
            break

    dut._log.info(f"sent={[hex(x) for x in test_bytes]}")
    dut._log.info(f"got ={[hex(x) for x in got]}")
    assert got == test_bytes, f"MISMATCH: sent={test_bytes} got={got}"
