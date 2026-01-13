#!/usr/bin/env python3
"""Complete activation sequence for Robotiq 2F-85"""

from pymodbus.client import ModbusSerialClient
from pymodbus import FramerType
import time

client = ModbusSerialClient(
    port='/dev/ttyUSB0',
    framer=FramerType.RTU,
    baudrate=115200,
    timeout=1
)

client.connect()
print("Connected\n")

def read_status():
    result = client.read_holding_registers(0x07D0, count=3, device_id=0x09)
    if not result.isError():
        reg0 = result.registers[0]
        gACT = (reg0 >> 15) & 0x01
        gSTA = (reg0 >> 8) & 0x03
        return reg0, gACT, gSTA
    return None, None, None

# Step 1: Reset
print("1. Reset...")
client.write_registers(0x03E8, [0x0000, 0x0000, 0x0000], device_id=0x09)
time.sleep(0.5)

# Step 2: Activate (rACT=1)
print("2. Set rACT=1...")
client.write_registers(0x03E8, [0x0100, 0x0000, 0x0000], device_id=0x09)
time.sleep(0.5)

# Check status
reg0, gACT, gSTA = read_status()
print(f"   Status after rACT: 0x{reg0:04X}, gSTA={gSTA}")

# Step 3: Set rGTO=1 to complete activation
print("3. Set rGTO=1 (Go To)...")
# rACT=1 (bit 0) + rGTO=1 (bit 3) = 0x09
client.write_registers(0x03E8, [0x0900, 0x0000, 0x00FF], device_id=0x09)

# Wait for activation
print("4. Waiting for activation...")
for i in range(30):
    time.sleep(0.2)
    reg0, gACT, gSTA = read_status()
    print(f"   [{i+1}] Status=0x{reg0:04X}, gSTA={gSTA}", end="")

    if gSTA == 3:
        print(" ← ✓ ACTIVATED!")
        break
    elif gSTA == 1:
        print(" (activating...)")
    elif gSTA == 0:
        print(" (reset)")
    else:
        print(f" (unknown)")
else:
    print("\n   ✗ Timeout")

# Final check
print("\n5. Final status:")
result = client.read_holding_registers(0x07D0, count=3, device_id=0x09)
if not result.isError():
    reg0 = result.registers[0]
    print(f"   Register 0: 0x{reg0:04X}")
    gACT = (reg0 >> 15) & 0x01
    gSTA = (reg0 >> 8) & 0x03
    gOBJ = (reg0 >> 6) & 0x03

    print(f"   gACT: {gACT}")
    print(f"   gSTA: {gSTA} {'✓ ACTIVATED' if gSTA == 3 else ''}")
    print(f"   Position: {result.registers[1] & 0xFF}")

client.close()
