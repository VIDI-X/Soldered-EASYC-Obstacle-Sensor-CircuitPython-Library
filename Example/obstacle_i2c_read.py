# code.py — VIDI X + Soldered Obstacle Sensor (easyC, I2C)
# Minimal, ASCII-only demo. Auto-detects register map, then prints readings.
import time
import board

from ObstacleSensor_I2C import ObstacleSensor_I2C

I2C_ADDR = 0x30        # change if your I2C scan shows a different address
DETECT_S = 2.0         # seconds for auto-detect sampling
AVG_SAMPLES = 4        # simple averaging for the analog value

i2c = board.I2C()      # or: busio.I2C(board.GPIO32, board.GPIO33) on VIDI X
sensor = ObstacleSensor_I2C(i2c, address=I2C_ADDR, active_low=True, avg_samples=AVG_SAMPLES)

print("Detecting register map... wave a hand in front of the sensor.")
a_reg, d_reg = sensor.auto_detect_map(duration_s=DETECT_S)
print("Using analog_reg=0x%02X, digital_reg=0x%02X" % (a_reg, d_reg))

# Quick 2-point calibration to produce a 0..100 percent proximity value
print("Calibration step 1: keep area clear (no obstacle).")
time.sleep(1.0)
far_raw = sensor.read_analog_raw()

print("Calibration step 2: hold an obstacle close (2-3 cm).")
time.sleep(1.0)
close_raw = sensor.read_analog_raw()

print("\nLive. Press Ctrl+C to stop.\n")
while True:
    digital = sensor.read_digital()       # True when obstacle is detected
    analog_raw = sensor.read_analog_raw() # 0..65535
    print("Obstacle:", digital, "| Analog(raw):", analog_raw)
    time.sleep(0.12)
