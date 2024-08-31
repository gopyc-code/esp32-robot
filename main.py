from machine import Pin, PWM, Timer, SoftI2C
import bluetooth
from ble_uart_peripheral import BLEUART
from tcs34725 import TCS34725
from time import sleep


class RGB_Sensor(object):
    def __init__(self, sda, scl, freq=400000, per=1000):
        i2c = SoftI2C(sda=Pin(sda), scl=Pin(scl), freq=freq)
        self.tcs = TCS34725(i2c)
        self.timer = Timer(1)
        self.timer.init(mode=Timer.PERIODIC, period=per, callback=self.print_rgb)

    def print_rgb(self, m=None):
        print(self.tcs.read('rgb'))


class Grip(object):
    def __init__(self, pin, freq=50, per=100):
        self.servo = PWM(Pin(pin, Pin.OUT), freq=freq)
        self.per = per
        
        self.timer = Timer(4)
        self.timer.init(mode=Timer.ONE_SHOT, period=self.per, callback=self.stop)
    
    def release(self, m=None):
        self.servo.duty(40)
        
    def catch(self, m=None):
        self.servo.duty(100)

    def stop(self, m=None):
        self.servo.duty(69)
        
    def handle_uart_code(self, code):
        CODES_CONTROL = {
            '!B11:': self.release,
            '!B219': self.catch,
            '!B318': self.stop,
            '!B10;': self.stop,
            '!B20:': self.stop,
            '!B309': self.stop
        }
        
        if code in CODES_CONTROL.keys():
            self.timer.deinit()
            self.timer.init(mode=Timer.PERIODIC, period=self.per, callback=CODES_CONTROL[code])


class Motor(object):
    MAX_ADC = 1000
    MIN_ADC = 0
    
    def __init__(self, pin1, pin2, dif=50, freq=50):
        self.dif = dif
        self.pwm1 = PWM(Pin(pin1, Pin.OUT), freq=freq, duty=0)
        self.pwm2 = PWM(Pin(pin2, Pin.OUT), freq=freq, duty=0)
    
    def forward(self):
        self.pwm1.duty(min(self.pwm1.duty() + self.dif, self.MAX_ADC))
        self.pwm2.duty(max(self.pwm2.duty() - self.dif, self.MIN_ADC))

    def back(self):
        self.pwm2.duty(min(self.pwm2.duty() + self.dif, self.MAX_ADC))
        self.pwm1.duty(max(self.pwm1.duty() - self.dif, self.MIN_ADC))
    
    def retard(self):
        self.pwm1.duty(max(self.pwm1.duty() - self.dif, self.MIN_ADC))
        self.pwm2.duty(max(self.pwm2.duty() - self.dif, self.MIN_ADC))
        
    def stop(self):
        self.pwm2.duty(self.MIN_ADC)
        self.pwm1.duty(self.MIN_ADC)


class Chassis(object):
    def __init__(self, motors, per=100):
        self.motors = motors
        self.NMOTORS = len(self.motors)
        self.per = per
        
        self.timer = Timer(0)
        self.timer.init(mode=Timer.ONE_SHOT, period=self.per, callback=self.stop)

    def left(self, m=None):
        for i in range(self.NMOTORS // 2):
            self.motors[i].back()
        for i in range(self.NMOTORS // 2, self.NMOTORS, 1):
            self.motors[i].forward()

    def right(self, m=None):
        for i in range(self.NMOTORS // 2):
            self.motors[i].forward()
        for i in range(self.NMOTORS // 2, self.NMOTORS, 1):
            self.motors[i].back()

    def forward(self, m=None):
        for i in range(self.NMOTORS):
            self.motors[i].forward()
      
    def back(self, m=None):
        for i in range(self.NMOTORS):
            self.motors[i].back()
            
    def retard(self, m=None):
        for i in range(self.NMOTORS * 2):
            self.motors[i].retard()
      
    def stop(self, m=None):
        for i in range(self.NMOTORS):
            self.motors[i].stop()
    
    def handle_uart_code(self, code):
        CODES_CONTROL = {
            '!B516': self.forward,
            '!B813': self.right,
            '!B714': self.left,
            '!B615': self.back,
            '!B417': self.retard,
            '!B507': self.stop,
            '!B804': self.stop,
            '!B705': self.stop,
            '!B606': self.stop,
            '!B408': self.retard
        }
        
        if code in CODES_CONTROL.keys():
            self.timer.deinit()
            self.timer.init(mode=Timer.PERIODIC, period=self.per, callback=CODES_CONTROL[code])


class Handler(object):
    def __init__(self, aims):
        self.aims = aims
        self.uart = BLEUART(bluetooth.BLE()) 
        self.uart.irq(handler=self.handle)
        self.uart.close()
    
    def handle(self):
        Pin(2, Pin.OUT).value(1)
        
        code = self.uart.read().decode("utf-8") 
        for aim in self.aims:
            aim.handle_uart_code(code)
        
        Pin(2, Pin.OUT).value(0)
        
        
"""
   Scheme of robot

        grip
       (   )
     +--\_/--+
     | sensor|
  +-------------+
0-|motor4 motor2|-0
  |    \  /     |
  |    ESP32    |
  |    /  \     |
0-|motor3 motor1|-0
  +-------------+


Bluefruit Connect (buttons):

1 - release grip
2 - catch
3 - stop servo (for debug)
4 - stop motors (for debug)
"""

rgb_sensor = RGB_Sensor(19, 22)
chassis = Chassis(
    [
        Motor(25, 26),
        Motor(14, 27),
        Motor(17, 5),
        Motor(16, 4)
    ]
)
grip = Grip(33)

handler = Handler([chassis, grip])
handler.handle()
