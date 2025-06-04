from machine import Pin, I2C, ADC
import time

# TCS34725 constants
TCS34725_ADDRESS = 0x29
COMMAND_BIT = 0x80

ENABLE = 0x00
ATIME = 0x01
CONTROL = 0x0F
ID = 0x12
CDATA = 0x14

ENABLE_PON = 0x01
ENABLE_AEN = 0x02

GAIN_1X = 0x00
GAIN_4X = 0x01
GAIN_16X = 0x02

# PCA9685 constants
PCA9685_ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

# Stepper outputs (map to stepper driver inputs)
STEPPERI_IN1 = 10
STEPPERI_IN2 = 11
STEPPERI_IN3 = 12
STEPPERI_IN4 = 13

STEPPER_HOME_SENSOR_PIN = 26
STEPPER_HOME_THRESHOLD = 25000

# Ball toggle angles
MIN_ANGLE = 0
MAX_ANGLE = 55


class Motor:
    stepms = 10

    # Do be defined by subclasses
    maxpos = 0
    states = []

    def __init__(self, p1, p2, p3, p4, stepms=None):
        self.pins = [p1, p2, p3, p4]

        if stepms is not None:
            self.stepms = stepms

        self._state = 0
        self._pos = 0

    def __repr__(self):
        return '<{} @ {}>'.format(
            self.__class__.__name__,
            self.pos,
        )

    @property
    def pos(self):
        return self._pos

    @classmethod
    def frompins(cls, *pins, **kwargs):
        return cls(*[Pin(pin, Pin.OUT) for pin in pins],
                   **kwargs)

    def reset(self):
        self._pos = 0
        self.pins_off()

    def _step(self, dir):
        state = self.states[self._state]

        for i, val in enumerate(state):
            self.pins[i].value(val)

        self._state = (self._state + dir) % len(self.states)
        self._pos = (self._pos + dir) % self.maxpos

    def pins_off(self):
        state = self.states[self._state]

        for i, val in enumerate(state):
            self.pins[i].value(0)

    def step(self, steps):
        dir = 1 if steps >= 0 else -1
        steps = abs(steps)

        for _ in range(steps):
            t_start = time.ticks_ms()

            self._step(dir)

            t_end = time.ticks_ms()
            t_delta = time.ticks_diff(t_end, t_start)
            time.sleep_ms(self.stepms - t_delta)

    def step_until(self, target, dir=None):
        if target < 0 or target > self.maxpos:
            raise ValueError(target)

        if dir is None:
            dir = 1 if target > self._pos else -1
            if abs(target - self._pos) > self.maxpos/2:
                dir = -dir

        while True:
            if self._pos == target:
                break
            self.step(dir)

        self.pins_off()

    def step_until_angle(self, angle, dir=None):
        if angle < 0 or angle > 360:
            raise ValueError(angle)

        target = int(angle / 360 * self.maxpos)
        self.step_until(target, dir=dir)

    def step_degrees(self, degrees):
        if degrees < 0 or degrees > 360:
            raise ValueError("Degrees should be between 0 and 360")

        steps_to_take = int(degrees / 360 * self.maxpos)

        self.zero()  # Ignore the current position, start from zero
        self.step(steps_to_take)


class HalfStepMotor(Motor):
    stepms = 3
    maxpos = 4096
    states = [
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1],
        [1, 0, 0, 1],
    ]


def pca9685_init():
    i2c_1.writeto_mem(PCA9685_ADDR, MODE1, b'\x00')  # normal mode
    pca9685_set_pwm_freq(50)  # 50Hz for servo


def pca9685_set_pwm_freq(freq_hz):
    prescale_val = int(25000000.0 / (4096 * freq_hz) - 1)
    old_mode = i2c_1.readfrom_mem(PCA9685_ADDR, MODE1, 1)[0]
    sleep_mode = (old_mode & 0x7F) | 0x10
    i2c_1.writeto_mem(PCA9685_ADDR, MODE1, bytes([sleep_mode]))  # go to sleep
    i2c_1.writeto_mem(PCA9685_ADDR, PRESCALE, bytes(
        [prescale_val]))  # set prescale
    i2c_1.writeto_mem(PCA9685_ADDR, MODE1, bytes([old_mode]))  # wake up
    time.sleep_ms(5)
    i2c_1.writeto_mem(PCA9685_ADDR, MODE1, bytes(
        [old_mode | 0xA1]))  # auto-increment


def pca9685_set_pwm(channel, on, off):
    reg = LED0_ON_L + 4 * channel
    data = bytes([on & 0xFF, on >> 8, off & 0xFF, off >> 8])
    i2c_1.writeto_mem(PCA9685_ADDR, reg, data)

# --- Servo Control ---


def pca9685_set_servo_us(channel, us):
    # 1 tick = (1/50Hz) / 4096 = ~4.88us
    ticks = int(us * 4096 / 20000)
    pca9685_set_pwm(channel, 0, ticks)


def pca9685_set_servo_angle(channel, angle):
    min_us = 500
    max_us = 2500
    delta_us = max_us - min_us
    us_per_degree = delta_us / 180
    us = angle * us_per_degree + min_us
    pca9685_set_servo_us(channel, us)


def tcs34725_write8(reg, val):
    i2c_0.writeto_mem(TCS34725_ADDRESS, COMMAND_BIT | reg, bytes([val]))


def tcs34725_read8(reg):
    return int.from_bytes(i2c_0.readfrom_mem(TCS34725_ADDRESS, COMMAND_BIT | reg, 1), 'little')


def tcs34725_read16(reg):
    data = i2c_0.readfrom_mem(TCS34725_ADDRESS, COMMAND_BIT | reg, 2)
    return data[1] << 8 | data[0]


def tcs34725_init():
    sensor_id = tcs34725_read8(ID)

    tcs34725_write8(ENABLE, ENABLE_PON)
    time.sleep(0.01)
    tcs34725_write8(ENABLE, ENABLE_PON | ENABLE_AEN)

    tcs34725_write8(ATIME, 0x00)         # Max integration time
    tcs34725_write8(CONTROL, GAIN_4X)    # Set gain


def tcs34725_read_colors():
    time.sleep(0.7)  # Wait for integration
    c = tcs34725_read16(CDATA)
    r = tcs34725_read16(CDATA + 2)
    g = tcs34725_read16(CDATA + 4)
    b = tcs34725_read16(CDATA + 6)
    return r, g, b, c

def interpolate_angle(start, end):  
    if start == end:
        return
    
    a = start
    dir = -1    
    
    if start < end:
        dir = 1
        
    while a != end:
        pca9685_set_servo_angle(0, a)
        a = a + dir
        time.sleep(0.01)
        
def next_ball():
    interpolate_angle(MAX_ANGLE, MIN_ANGLE)
    #pca9685_set_servo_angle(0, 0)
    time.sleep(0.5)
    interpolate_angle(MIN_ANGLE, MAX_ANGLE)
    #pca9685_set_servo_angle(0, 40)


color_ranges = {
    ("red",   ((0.7, 0.9), (0.1, 0.2), (0.1, 0.2))),
    ("green", ((0.3, 0.5), (0.4, 0.6), (0.1, 0.3))),
    ("blue",  ((0.1, 0.3), (0.3, 0.5), (0.3, 0.5))),
    ("pink",   ((0.5, 0.7), (0.15, 0.25), (0.15, 0.25 ))),
    ("purple", ((0.3, 0.5), (0.2, 0.4), (0.2, 0.4))),
    ("yellow",  ((0.45, 0.65), (0.25, 0.4), (0.1, 0.2)))
}


def get_color_name(r, g, b):
    for name, ((rmin, rmax), (gmin, gmax), (bmin, bmax)) in color_ranges:
        if rmin <= r <= rmax and gmin <= g <= gmax and bmin <= b <= bmax:
            return name
    return "unknown"


# i2c_0 setup on GPIO5 (SCL), GPIO4 (SDA)
i2c_0 = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

# Setup I2C on GPIO16 (SCL), GPIO17 (SDA)
i2c_1 = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)

# Initialize the stepper motor
stepper_motor = HalfStepMotor.frompins(
    STEPPERI_IN1, STEPPERI_IN2, STEPPERI_IN3, STEPPERI_IN4)

stepper_home_sensor = ADC(Pin(STEPPER_HOME_SENSOR_PIN))


def home_stepper():
    # Move past current home position if already at home (it might be slightly ajar)
    while stepper_is_home():
        stepper_motor.step(100)

    while not stepper_is_home():
        # Move clockwise until home triggered
        stepper_motor.step(1)

    # Home is position 0
    stepper_motor.reset()


def stepper_is_home():
    stepper_sensor_value = stepper_home_sensor.read_u16()
    is_home = stepper_sensor_value <= STEPPER_HOME_THRESHOLD
    return is_home


# Start main program
pca9685_init()
tcs34725_init()
stepper_is_home_prev = stepper_is_home()

try:
    home_stepper()
    
    pca9685_set_servo_angle(MIN_ANGLE, MAX_ANGLE)

    while True:
        stepper_sensor_value = stepper_home_sensor.read_u16()

        r, g, b, c = tcs34725_read_colors()

        ball_detected = c > 3000
        
        if ball_detected:
            time.sleep(1)
            r_raw, g_raw, b_raw, c = tcs34725_read_colors()

            r = r_raw / c
            g = g_raw / c
            b = b_raw / c

            color = get_color_name(r, g, b)

            if color != "unknown":
                print(f"{color}")
                if color == "red" or color == "pink" or color == "purple":
                    stepper_motor.step_until(4096 - 170, -1)
                else:
                    stepper_motor.step_until(170, 1)
                #print("{}, {}, {}, {}".format(r, g, b, c))
                
            else:
              print("Unknown color")
              print("{}, {}, {}, {}".format(r, g, b, c))
              stepper_motor.step_until(205)

            next_ball()
        
        time.sleep(0.3)
except KeyboardInterrupt:
    print("Execution stopped.")

