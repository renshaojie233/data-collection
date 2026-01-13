#!/usr/bin/env python3
"""Check current gripper status with correct bit parsing"""

from pymodbus.client import ModbusSerialClient
from pymodbus import FramerType

client = ModbusSerialClient(
    port='/dev/ttyUSB0',
    framer=FramerType.RTU,
    baudrate=115200,
    timeout=1
)

client.connect()

result = client.read_holding_registers(0x07D0, count=3, device_id=0x09)

if not result.isError():
    reg0 = result.registers[0]
    print(f"Register 0: 0x{reg0:04X} = {reg0:016b}")
    print()

    # Correct bit positions for Robotiq 2F-85
    # High byte (bits 15-8)
    gACT = (reg0 >> 15) & 0x01   # bit 15
    gGTO = (reg0 >> 11) & 0x01   # bit 11
    gSTA = (reg0 >> 8) & 0x03    # bits 9-8
    gOBJ = (reg0 >> 6) & 0x03    # bits 7-6

    # Low byte (bits 7-0)
    gFLT = (reg0 >> 0) & 0x0F    # bits 3-0

    print(f"gACT (Gripper Activated): {gACT}")
    print(f"gGTO (Go To): {gGTO}")
    print(f"gSTA (Status): {gSTA} ", end="")
    if gSTA == 0: print("(Reset)")
    elif gSTA == 1: print("(Activating)")
    elif gSTA == 3: print("(✓ ACTIVATED!)")

    print(f"gOBJ (Object detection): {gOBJ}")
    print(f"gFLT (Fault): 0x{gFLT:X}")
    print()
    print(f"Position: {result.registers[1] & 0xFF}")
    print(f"Current: {result.registers[2] & 0xFF}")

client.close()
