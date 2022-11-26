from pyb import UART
import time

led_alarm = pyb.LED(1)

uart_6 = UART(6, 115200)
#uart_6.init(115200, bits=8, parity=None, stop=1)

uart_1 = UART(1, 115200)
#uart_1.init(115200, bits=8, parity=None, stop=1)


class LidarMiniTFPlus:
    def __init__(self, uart):
        self.uart = uart
        self.uart.init(115200, bits=8, parity=None, stop=1,read_buf_len=64)
        self.distance_slow_m = 0.0
        self.distance_slow_m_increment_plus = 1E-3
        self.distance_slow_m_increment_minus = 1E-4
        self.distance_m = 0.0
        self.detect_at_ticks_ms = None
        self.timeout_ms = 5000
        self.signal_strength = None
        self.temperature_C = None
        self.detected_flag_intern = False
        self.missaligned = False

    def try_to_get_new_measurement(self):
        if self.missaligned:
            if self.uart.any() > 0:
                self.uart.read(1) #trash one
                self.missaligned = False
                return False
        temp = bytes()
        if self.uart.any() > 8:
            temp += self.uart.read(9)
            if not (temp[0] == 0x59 and temp[1] == 0x59):
                # missaligned, ignore and shift by one
                self.missaligned = True
                print('missaligned')
                return False
            self.distance_m   = (temp[2] + temp[3] * 256 ) / 100.0   #Get distance value  
            self.signal_strength    = temp[4] + temp[5] * 256            #Get Strength value  
            self.temperature_C = (temp[6] + temp[7]* 256)/8-256      #Get IC temperature value  
            if self.distance_m > self.distance_slow_m:
                self.distance_slow_m += self.distance_slow_m_increment_plus
            else:
                self.distance_slow_m -= self.distance_slow_m_increment_minus
            if self.distance_slow_m - self.distance_m > 0.1: # Es ist ein Gegenstand im Strahl
                if not self.detected_flag_intern: # Falls nicht bereits das letzte mal aufgetreten
                    self.detect_at_ticks_ms = time.ticks_ms()
                    self.detected_flag_intern = True
                    return True
                else:
                    self.detected_flag_intern = False
                    return False

    def do_extra_missalign(self): # for testing
        while self.uart.read(1) == None: # none: timeout vom lesen
            time.sleep_us(1)

print('hugo')

lidar_6 = LidarMiniTFPlus(uart_6)
lidar_1 = LidarMiniTFPlus(uart_1)

monitor_at_ms = time.ticks_ms()
while True:
    lidar_6.try_to_get_new_measurement()
    lidar_1.try_to_get_new_measurement()
    if time.ticks_diff(monitor_at_ms, time.ticks_ms()) <0:
        monitor_at_ms = time.ticks_add(monitor_at_ms, 1000)
        print(lidar_1.distance_m, lidar_6.distance_m,)



