# ObstacleSensor_I2C.py
# AUTHOR: VIDI X Team — CircuitPython I2C driver for Soldered Obstacle Sensor (easyC, ATtiny404)
# LICENSE: MIT
#
# DESCRIPTION
#   Driver for Soldered's Obstacle sensor with easyC (TCRT5000 + ATtiny404 over I2C).
#   The firmware exposes a digital "obstacle" state and an analog intensity value.
#   Register map can vary across revisions, so you can override it in the constructor
#   or call auto_detect_map() to let the driver infer the likely registers.
#
# DEFAULTS
#   address     = 0x30        # typical easyC address (edit if your scan shows different)
#   analog_reg  = 0x01        # 16-bit little-endian
#   digital_reg = 0x03        # 8-bit
#
# QUICK TEST
#   import board, time
#   from ObstacleSensor_I2C import ObstacleSensor_I2C
#   i2c = board.I2C()  # or: busio.I2C(board.GPIO32, board.GPIO33) on VIDI X
#   s = ObstacleSensor_I2C(i2c, address=0x30, active_low=True)
#   s.auto_detect_map(duration_s=3.0)  # wave your hand during detection
#   while True:
#       print("Digital:", s.read_digital(), "Analog:", s.read_analog_raw())
#       time.sleep(0.1)
#
import time
from adafruit_bus_device.i2c_device import I2CDevice

class ObstacleSensor_I2C:
    def __init__(self, i2c, *, address=0x30, analog_reg=0x01, digital_reg=0x03, active_low=True, avg_samples=1):
        self._dev = I2CDevice(i2c, address)
        self.address = int(address) & 0x7F
        self._analog_reg = int(analog_reg) & 0xFF
        self._digital_reg = int(digital_reg) & 0xFF
        self._active_low = bool(active_low)
        self._avg = max(1, int(avg_samples))

    # ----- Low-level helpers -----
    def _write_then_readinto_retry(self, wbuf, rbuf):
        # Try repeated-start first.
        try:
            with self._dev as i2c:
                i2c.write_then_readinto(wbuf, rbuf)
            return
        except OSError:
            # Fall back to STOP between ops: separate write() then readinto().
            time.sleep(0.0015)
            with self._dev as i2c:
                i2c.write(wbuf)
                time.sleep(0.001)
                i2c.readinto(rbuf)

    def _read_u8(self, reg):
        b = bytearray(1)
        self._write_then_readinto_retry(bytes([reg & 0xFF]), b)
        return b[0]

    def _read_u16le(self, reg):
        b = bytearray(2)
        self._write_then_readinto_retry(bytes([reg & 0xFF]), b)
        return b[0] | (b[1] << 8)

    # ----- Public API -----
    def read_analog_raw(self):
        if self._avg <= 1:
            return self._read_u16le(self._analog_reg)
        total = 0
        for _ in range(self._avg):
            total += self._read_u16le(self._analog_reg)
        return int(total / self._avg)

    def read_digital_raw(self):
        return self._read_u8(self._digital_reg)

    def read_digital(self):
        raw = self.read_digital_raw()
        bit_is_high = (raw != 0)
        return (not bit_is_high) if self._active_low else bit_is_high

    def read_analog_mv(self, full_scale_mv=3300.0):
        raw = self.read_analog_raw()
        if raw is None:
            return None
        return float(raw) * (float(full_scale_mv) / 65535.0)

    # ----- Map utilities -----
    def reconfigure_map(self, *, analog_reg=None, digital_reg=None):
        if analog_reg is not None:
            self._analog_reg = int(analog_reg) & 0xFF
        if digital_reg is not None:
            self._digital_reg = int(digital_reg) & 0xFF

    def dump_first_bytes(self, n=8):
        out = bytearray()
        for reg in range(n):
            out += bytes([self._read_u8(reg)])
        return bytes(out)

    def auto_detect_map(self, *, window=(0x00, 0x10), duration_s=3.0, sample_delay=0.04):
        """Heuristically detect which registers change when you move an object.
        Wave your hand in front of the sensor while this runs.
        Chooses:
          - digital_reg: the 8-bit register with strongest binary toggle
          - analog_reg: the 16-bit LE pair (r, r+1) with the largest variance
        """
        start_reg, end_reg = int(window[0]) & 0xFF, int(window[1]) & 0xFF
        if end_reg <= start_reg:
            end_reg = start_reg + 0x10

        # Collect samples
        regs = list(range(start_reg, end_reg))
        mins = {r: 255 for r in regs}
        maxs = {r: 0   for r in regs}
        toggles = {r: 0 for r in regs}
        count = 0
        t_end = time.monotonic() + float(duration_s)
        prev = {}

        while time.monotonic() < t_end:
            for r in regs:
                v = self._read_u8(r)
                if v < mins[r]: mins[r] = v
                if v > maxs[r]: maxs[r] = v
                if r in prev and v != prev[r]:
                    toggles[r] += 1
                prev[r] = v
            count += 1
            time.sleep(sample_delay)

        # Pick digital: the register with the largest number of toggles,
        # and preferably a small set of distinct values (0/1 or 0/255).
        best_d, best_score = None, -1
        for r in regs:
            span = maxs[r] - mins[r]
            score = toggles[r] * 10 - span  # favor binary-ish behavior
            if score > best_score:
                best_score = score
                best_d = r

        # Pick analog: among adjacent pairs, choose the pair with largest combined span.
        best_a, best_span = None, -1
        for r in regs:
            if (r + 1) in regs:
                span_pair = (maxs[r] - mins[r]) + (maxs[r+1] - mins[r+1])
                if span_pair > best_span:
                    best_span = span_pair
                    best_a = r

        if best_d is not None:
            self._digital_reg = best_d & 0xFF
        if best_a is not None:
            self._analog_reg = best_a & 0xFF

        return (self._analog_reg, self._digital_reg)

    def __repr__(self):
        return "<ObstacleSensor_I2C addr=0x%02X analog=0x%02X digital=0x%02X active_low=%s avg=%d>" % (
            self.address, self._analog_reg, self._digital_reg, self._active_low, self._avg
        )
