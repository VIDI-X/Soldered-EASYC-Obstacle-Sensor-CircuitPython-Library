# probe_obstacle_i2c.py — mapping helper for Soldered Obstacle Sensor (easyC)
# Moves through a window of registers, samples them while you wave a hand,
# and suggests analog/digital registers for the driver.
import time
import board
from ObstacleSensor_I2C import ObstacleSensor_I2C

ADDR = 0x30  # change if your I2C scan shows a different address
i2c = board.I2C()
s = ObstacleSensor_I2C(i2c, address=ADDR, active_low=True)

print("Probing address 0x%02X. Wave your hand in front of the sensor for ~3 seconds…" % ADDR)
a_reg, d_reg = s.auto_detect_map(window=(0x00, 0x10), duration_s=3.0, sample_delay=0.04)
print("Suggested map -> analog_reg: 0x%02X, digital_reg: 0x%02X" % (a_reg, d_reg))

print("Live test with the suggested map. Press Ctrl+C to stop.")
while True:
    ar = s.read_analog_raw()
    dg = s.read_digital()
    print("Analog(raw):", ar, "| Digital:", dg)
    time.sleep(0.15)
