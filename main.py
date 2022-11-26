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
        self.signal_strength = None
        self.temperature_C = None
        self.detected_last = False
        self.misaligned = False

    def try_to_get_new_measurement(self):
        """
        Returns True when object detected
        """
        if self.misaligned:
            if self.uart.any() > 0:
                self.uart.read(1) #trash one
                self.misaligned = False
            return False

        if self.uart.any() < 9:
            return False

        buf = self.uart.read(9)
        
        if buf[0] != 0x59 or buf[1] != 0x59:
            # misaligned, ignore and shift by one
            self.misaligned = True
            return False

        checksum = sum(buf[0:8]) % 256
        if buf[8] != checksum:
            # misaligned, ignore and shift by one
            self.misaligned = True
            return False
        
        self.distance_m   = (buf[2] + buf[3] * 256 ) / 100.0   #Get distance value  
        self.signal_strength    = buf[4] + buf[5] * 256            #Get Strength value  
        self.temperature_C = (buf[6] + buf[7]* 256)/8-256      #Get IC buferature value  
        if self.distance_m > self.distance_slow_m:
            self.distance_slow_m += self.distance_slow_m_increment_plus
        else:
            self.distance_slow_m -= self.distance_slow_m_increment_minus
            
        if self.distance_slow_m - self.distance_m < 0.1: # Es ist kein Gegenstand im Strahl
            self.detected_last = False
            return False

        if self.detected_last == False: # Falls nicht bereits das letzte mal detektiert
            self.detect_at_ticks_ms = time.ticks_ms()
            self.detected_last = True
            print('.')
            return True
        return False
                

    def do_extra_misalign(self): # for testing
        while self.uart.read(1) == None: # none: timeout vom lesen
            time.sleep_us(1)

print('hugo')

lidar_6 = LidarMiniTFPlus(uart_6)
lidar_1 = LidarMiniTFPlus(uart_1)

monitor_at_ms = time.ticks_ms()

delaytime_us = 0
while True:
    lidar_6_detected = lidar_6.try_to_get_new_measurement()
    lidar_1_detected = lidar_1.try_to_get_new_measurement()
    if lidar_1.misaligned or lidar_6.misaligned:
        delaytime_us = 0
    time.sleep_us(delaytime_us)
    if time.ticks_diff(monitor_at_ms, time.ticks_ms()) <0:
        monitor_at_ms = time.ticks_add(monitor_at_ms, 1000)
        print(lidar_1.distance_m, lidar_1.detect_at_ticks_ms, lidar_6.distance_m,lidar_6.detect_at_ticks_ms, 'delaytime_us %d'%delaytime_us)
        delaytime_us += 1000



