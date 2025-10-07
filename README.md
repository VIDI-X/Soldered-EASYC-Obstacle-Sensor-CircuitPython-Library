# Soldered-EASYC-Obstacle-Sensor-CircuitPython-Library
CircuitPython Library for Obstacle Sensor with I2C (EASYC,VIDIIC,qwiic) from Soldered

---

# VIDI X Obstacle Sensor (VIDIIC, easyC, I²C, qwiic) — CircuitPython

CircuitPython driver and examples for the **Soldered Obstacle Sensor with easyC** (TCRT5000 over I²C). Works out-of-the-box on **VIDI X (ESP32)** and other CircuitPython boards with I²C.
Product page: [https://soldered.com/product/obstacle-sensor-with-easyc/](https://soldered.com/product/obstacle-sensor-with-easyc/)

![DSC_3391-Edit-768x512](https://github.com/user-attachments/assets/97edae0e-86e1-40a9-bb65-be01d091b8cd)

## What’s in this repo

* `ObstacleSensor_I2C.py` — the driver: reads the sensor’s **digital obstacle flag** and **analog intensity** over I²C. Includes a robust read path (repeated-start with STOP fallback) and a small **auto-detect** that finds the right register map while you wave your hand. 
* `obstacle_i2c_read.py` — a minimal example. It **auto-detects** the map, prints the digital flag and raw analog value continuously. Great for a first lesson. 
* `probe_obstacle_i2c.py` — a mapping helper you can run once. It scans a small window of registers while you move your hand, **suggests** `analog_reg` and `digital_reg`, then live-tests them. Handy if your module uses a different firmware mapping. 

## Requirements

* CircuitPython on your VIDI X (or another supported board).
* The `adafruit_bus_device` library in `CIRCUITPY/lib/` (driver uses `I2CDevice`).
* Wiring: easyC → VIDI X I²C. Default 7-bit I²C address is often **0x30** (change in code if your scan shows otherwise). 

## Quick start (copy–paste)

Save this as `test-code.py` on **CIRCUITPY** next to `ObstacleSensor_I2C.py`:

```python
import time, board
from ObstacleSensor_I2C import ObstacleSensor_I2C

I2C_ADDR = 0x30     # change if your I2C scan shows a different address
i2c = board.I2C()   # on VIDI X, board.I2C() is fine

sensor = ObstacleSensor_I2C(i2c, address=I2C_ADDR, active_low=True, avg_samples=4)

print("Detecting register map... wave a hand in front of the sensor.")
a_reg, d_reg = sensor.auto_detect_map(duration_s=2.0)  # quick 2s sampling
print("Using analog_reg=0x%02X, digital_reg=0x%02X" % (a_reg, d_reg))

while True:
    digital = sensor.read_digital()       # True when obstacle detected
    analog_raw = sensor.read_analog_raw() # 0..65535 (firmware-defined scale)
    print("Obstacle:", digital, "| Analog(raw):", analog_raw)
    time.sleep(0.12)
```

That’s it—students can already see the numbers jump as they move an object.

If you prefer, you can run the included example scripts directly (`obstacle_i2c_read.py` for a minimal demo or `probe_obstacle_i2c.py` to explicitly see the suggested mapping).  

## Locking the register map (optional but faster)

After the first auto-detect run prints something like:

```
Using analog_reg=0x01, digital_reg=0x03
```

you can **hard-code** those in the constructor to skip detection on future runs:

```python
sensor = ObstacleSensor_I2C(
    i2c,
    address=0x30,
    analog_reg=0x01,   # from auto-detect
    digital_reg=0x03,  # from auto-detect
    active_low=True,
    avg_samples=4
)
```

## API overview (driver)

From `ObstacleSensor_I2C.py` you’ll typically use: 

* `read_digital()` → `True` when an obstacle is detected (with `active_low=True`, which matches most boards where DO goes LOW on detection).
* `read_analog_raw()` → 16-bit intensity (0..65535).
* `read_analog_mv(full_scale_mv=3300.0)` → rough millivolt estimate if you want a “voltage-like” number.
* `reconfigure_map(analog_reg=…, digital_reg=…)` → change registers at runtime.
* `auto_detect_map(duration_s=…, window=(0x00, 0x10))` → watch which registers change while you wave your hand; returns `(analog_reg, digital_reg)`.
* `dump_first_bytes(n=8)` → quick peek at the first bytes for debugging.

## Tips for classrooms

* Start with the minimal example and ask students to **predict** when the digital flag flips, then turn the trim-pot on the board to change the threshold.
* Use the analog value to discuss **“more/less light”** (IR reflectivity) and show how it changes with distance and surface color.
* Challenge: have students implement a two-point calibration (far/close) and map raw analog to **0–100% proximity**—or adapt the example to print a percentage.

## Troubleshooting

* **No change in numbers?** Run `probe_obstacle_i2c.py`. It will sample registers and suggest the correct map for your firmware, then live-test it. 
* **All zeros or constant max?** Double-check the I²C address with a scanner. Change `address=` in the constructor if needed.
* **Digital logic inverted?** Set `active_low=False` so `read_digital()` returns `True` when DO is HIGH instead.
* **Occasional I/O errors (NACK)?** The driver already retries with a STOP-then-READ fallback, which fixes most strict devices. If needed, slow your loop slightly (e.g., `time.sleep(0.12)`).
* **Other errors** Reset the VIDI X Device
  
## Credits & license

* **Driver**: VIDI X Team — CircuitPython I²C port inspired by Soldered’s Arduino library; product: Soldered Obstacle Sensor with easyC. MIT License. 
* **Examples**: designed for short classroom demos; see `obstacle_i2c_read.py` and `probe_obstacle_i2c.py`.  
