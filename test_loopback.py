import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from uart_bfm import UartDriver, UartMonitor

CLKS_PER_BIT = 4


@cocotb.test()
async def loopback(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.rst_n.value = 1
    dut.i_Rx_Serial.value = 1
    await ClockCycles(dut.clk, 5)

    driver = UartDriver(dut, dut.clk, dut.i_Rx_Serial, CLKS_PER_BIT)
    monitor = UartMonitor(dut, dut.clk, dut.i_Rx_Serial, CLKS_PER_BIT)
    cocotb.start_soon(monitor.start())
    await ClockCycles(dut.clk, 3)

    test_bytes = [0xB0, 0x00, 0xFF, 0x55, 0xAA, 0x87, 0x01, 0x3D]

    async def sender():
        for b in test_bytes:
            await driver.send(b)

    cocotb.start_soon(sender())

    got = []
    for i in range(len(test_bytes)):
        try:
            got.append(await cocotb.triggers.with_timeout(monitor.get_byte(), 5000, "ns"))
        except cocotb.triggers.SimTimeoutError:
            dut._log.error(f"TIMEOUT waiting for byte {i}, got so far={got}")
            break

    dut._log.info(f"sent={[hex(x) for x in test_bytes]}")
    dut._log.info(f"got ={[hex(x) for x in got]}")
    assert got == test_bytes, f"MISMATCH: sent={test_bytes} got={got}"
