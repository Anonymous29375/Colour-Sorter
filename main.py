from machine import Pin, I2C, ADC
import time

# The color sensor level for detecting the presence of a ball
BALL_DETECTED_THRESHOLD = 3000

# I2C pins
I2C_0_SDA = 4
I2C_0_SCL = 5
I2C_1_SDA = 18
I2C_1_SCL = 19

# Hall effect sensor pin number
STEPPER_HOME_SENSOR_PIN = 26

# The level threshold to detect magnet near hall effect sensor
STEPPER_HOME_THRESHOLD = 25000

# Ball toggle angles
BALL_TOGGLE_MIN_ANGLE = 0
BALL_TOGGLE_MAX_ANGLE = 55

# Stepper position to direct to each tray
TRAY_1_DIRECT_POS = 4096 - 170
TRAY_2_DIRECT_POS = 170

# Stepper directions (clockwise/counterclockwise)
STEPPER_CCW = -1
STEPPER_CW = 1


class StepperMotor:
    # Stepper outputs (map to stepper driver inputs)
    STEPPER_IN1 = 10
    STEPPER_IN2 = 11
    STEPPER_IN3 = 12
    STEPPER_IN4 = 13

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

    def __init__(self):
        self.pins = [Pin(self.STEPPER_IN1, Pin.OUT), Pin(self.STEPPER_IN2, Pin.OUT), Pin(self.STEPPER_IN3, Pin.OUT), Pin(self.STEPPER_IN4, Pin.OUT)]
        self._state = 0
        self._pos = 0

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

class Pca9685:
    DEVICE_ADDR = 0x40
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, i2c):
        self.i2c = i2c
        self.i2c.writeto_mem(self.DEVICE_ADDR, self.MODE1, b'\x00')  # normal mode
        self.set_pwm_freq(50)  # 50Hz for servo


    def set_pwm_freq(self, freq_hz):
        prescale_val = int(25000000.0 / (4096 * freq_hz) - 1)
        old_mode = self.i2c.readfrom_mem(self.DEVICE_ADDR, self.MODE1, 1)[0]
        sleep_mode = (old_mode & 0x7F) | 0x10
        self.i2c.writeto_mem(self.DEVICE_ADDR, self.MODE1, bytes([sleep_mode]))  # go to sleep
        self.i2c.writeto_mem(self.DEVICE_ADDR, self.PRESCALE, bytes(
            [prescale_val]))  # set prescale
        self.i2c.writeto_mem(self.DEVICE_ADDR, self.MODE1, bytes([old_mode]))  # wake up
        time.sleep_ms(5)
        self.i2c.writeto_mem(self.DEVICE_ADDR, self.MODE1, bytes(
            [old_mode | 0xA1]))  # auto-increment


    def set_pwm(self, channel, on, off):
        reg = self.LED0_ON_L + 4 * channel
        data = bytes([on & 0xFF, on >> 8, off & 0xFF, off >> 8])
        self.i2c.writeto_mem(self.DEVICE_ADDR, reg, data)

    def set_servo_us(self, channel, us):
        # 1 tick = (1/50Hz) / 4096 = ~4.88us
        ticks = int(us * 4096 / 20000)
        self.set_pwm(channel, 0, ticks)


    def set_servo_angle(self, channel, angle):
        min_us = 500
        max_us = 2500
        delta_us = max_us - min_us
        us_per_degree = delta_us / 180
        us = angle * us_per_degree + min_us
        self.set_servo_us(channel, us)


class Tcs34725:
    DEVICE_ADDRESS = 0x29
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

    def __init__(self, i2c):
        self.i2c = i2c

        self.write8(self.ENABLE, self.ENABLE_PON)
        time.sleep(0.01)
        self.write8(self.ENABLE, self.ENABLE_PON | self.ENABLE_AEN)

        self.write8(self.ATIME, 0x00)         # Max integration time
        self.write8(self.CONTROL, self.GAIN_4X)    # Set gain

    def write8(self, reg, val):
        self.i2c.writeto_mem(self.DEVICE_ADDRESS, self.COMMAND_BIT | reg, bytes([val]))

    def read16(self, reg):
        data = self.i2c.readfrom_mem(self.DEVICE_ADDRESS, self.COMMAND_BIT | reg, 2)
        return data[1] << 8 | data[0]


    def read_colors(self):
        time.sleep(0.7)  # Wait for integration
        c = self.read16(self.CDATA)
        r = self.read16(self.CDATA + 2)
        g = self.read16(self.CDATA + 4)
        b = self.read16(self.CDATA + 6)
        return r, g, b, c


# i2c_0 setup
i2c_0 = I2C(0, scl=Pin(I2C_0_SCL), sda=Pin(I2C_0_SDA), freq=400000)

# Create colour sensor instance
colour_sensor = Tcs34725(i2c_0)

# Setup I2C
i2c_1 = I2C(1, scl=Pin(I2C_1_SCL), sda=Pin(I2C_1_SDA), freq=400000)

# Create servo driver isntance
servo_driver = Pca9685(i2c_1)

# The colour ranges for matching each ball colour
# The smoothness means that they reflect a lot of light and so the RGB values are not
# what are expected. They were worked out by testing each ball and setting close ranges
color_ranges = {
    ("red",   ((0.7, 0.9), (0.1, 0.2), (0.1, 0.2))),
    ("green", ((0.3, 0.5), (0.4, 0.6), (0.1, 0.3))),
    ("blue",  ((0.1, 0.3), (0.3, 0.5), (0.3, 0.5))),
    ("pink",   ((0.5, 0.7), (0.15, 0.25), (0.15, 0.25 ))),
    ("purple", ((0.3, 0.5), (0.2, 0.4), (0.2, 0.4))),
    ("yellow",  ((0.45, 0.65), (0.25, 0.4), (0.1, 0.2)))
}

def interpolate_angle(start, end):  
    if start == end:
        return
    
    a = start
    dir = -1    
    
    if start < end:
        dir = 1
        
    while a != end:
        servo_driver.set_servo_angle(0, a)
        a = a + dir
        time.sleep(0.01)
        
def next_ball():
    interpolate_angle(BALL_TOGGLE_MAX_ANGLE, BALL_TOGGLE_MIN_ANGLE)
    time.sleep(0.5)
    interpolate_angle(BALL_TOGGLE_MIN_ANGLE, BALL_TOGGLE_MAX_ANGLE)


def get_color_name(r, g, b):
    for name, ((rmin, rmax), (gmin, gmax), (bmin, bmax)) in color_ranges:
        if rmin <= r <= rmax and gmin <= g <= gmax and bmin <= b <= bmax:
            return name
    return "unknown"


# Initialize the stepper motor
stepper_motor = StepperMotor()

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
try:
    # On boot home the director arm stepper
    home_stepper()
    
    servo_driver.set_servo_angle(BALL_TOGGLE_MIN_ANGLE, BALL_TOGGLE_MAX_ANGLE)

    while True:
        # Get the raw component of each color along with the colour total
        r_raw, g_raw, b_raw, c = colour_sensor.read_colors()

        # Has a ball been detected
        ball_detected = c > BALL_DETECTED_THRESHOLD
        
        if ball_detected:
            # Convert each colour to a percetage of total colour to get correct ratio
            r = r_raw / c
            g = g_raw / c
            b = b_raw / c

            # Get the ccolour name for the specified colour ratios
            color = get_color_name(r, g, b)

            if color != "unknown":
                if color == "red" or color == "pink" or color == "purple":
                    # All balls that are of a red-ish color move to tray 1
                    stepper_motor.step_until(TRAY_1_DIRECT_POS, STEPPER_CCW)
                
                else: # Green, yellow and blue
                    # All balls that are not a red-ish color move to tray 2
                    stepper_motor.step_until(TRAY_2_DIRECT_POS, STEPPER_CW)
                
            else:
              # If the colour was not detected then report it as unknown
              print("Unknown color")
              print("{}, {}, {}, {}".format(r, g, b, c))

            # Release the ball from the colour sensor and get next ball if present
            next_ball()
        
        time.sleep(0.3)
except KeyboardInterrupt:
    print("Execution stopped.")


