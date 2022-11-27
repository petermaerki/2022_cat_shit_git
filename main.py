from pyb import UART
import time

led_alarm = pyb.LED(1)

uart_6 = UART(6, 115200)
#uart_6.init(115200, bits=8, parity=None, stop=1)

uart_1 = UART(1, 115200)
#uart_1.init(115200, bits=8, parity=None, stop=1)


class LidarMiniTFPlus:
    def __init__(self, uart, name):
        self.name = name
        self._misaligned = False
        self._uart = uart
        self._uart.init(115200, bits=8, parity=None, stop=1,read_buf_len=64)
        self.distance_slow_m = 0.0
        self._distance_slow_m_increment_plus = 1E-3
        self._distance_slow_m_increment_minus = 1E-4
        self.distance_m = 0.0
        self.signal_strength = None
        self.temperature_C = None
        self.detected = False
        self.detected_edge_processed = False
        self.detected_ticks_ms = 0

    def process_input_data(self):
        """
        Process input data.
        Does nothing if misaligned or not sufficient data.
        """
        if self._misaligned:
            if self._uart.any() == 0:
                return
            self._uart.read(1)
            self._misaligned = False

        if self._uart.any() < 9:
            return

        buf = self._uart.read(9)
        
        if buf[0] != 0x59 or buf[1] != 0x59:
            # misaligned, ignore and shift by one
            self._misaligned = True
            return

        checksum = sum(buf[0:8]) % 256
        if buf[8] != checksum:
            # misaligned, ignore and shift by one
            self._misaligned = True
            return
        
        self.distance_m   = (buf[2] + buf[3] * 256 ) / 100.0   #Get distance value  
        self.signal_strength    = buf[4] + buf[5] * 256            #Get Strength value  
        self.temperature_C = (buf[6] + buf[7]* 256)/8-256      #Get IC buferature value
        if self.distance_slow_m < 0.1: # start up
            self.distance_slow_m = self.distance_m
        if self.distance_m > self.distance_slow_m:
            self.distance_slow_m += self._distance_slow_m_increment_plus
        else:
            self.distance_slow_m -= self._distance_slow_m_increment_minus
            
        detected_actual = self.distance_slow_m - self.distance_m > 0.1 #and self.distance_slow_m > 0.3
        deteced_edge = detected_actual != self.detected
        self.detected = detected_actual

        if deteced_edge:
            if detected_actual:
                self.detected_edge_processed = True
                self.detected_ticks_ms = time.ticks_ms()
                # print('.')
    
    def edge_detected(self):
        """
        Return True if a edge was observed.
        Will only return once True for every edge.
        """
        tmp = self.detected_edge_processed
        self.detected_edge_processed = False
        return tmp

    def get_status(self):
        delay_ms = time.ticks_diff(time.ticks_ms(), self.detected_ticks_ms)
        return "%s: %0.2fm (%0.2fm) %6dms" % (self.name, self.distance_m, self.distance_slow_m, delay_ms)


lidar_C6 = LidarMiniTFPlus(uart_6, "lidar_C6")
lidar_B1 = LidarMiniTFPlus(uart_1, "lidar_B1")

monitor_at_ms = time.ticks_ms() + 50

delaytime_us = 0
start_ticks_ms = time.ticks_ms()

lidars = (lidar_C6, lidar_B1)

while True:
    for lidar in lidars:
        lidar.process_input_data()
    
        if lidar.edge_detected():
            name = lidar.name
            print('detected: %s' % name)
    
        if time.ticks_diff(monitor_at_ms, time.ticks_ms()) < 0:
            monitor_at_ms = time.ticks_add(monitor_at_ms, 1000)
            status = ", ".join(map(LidarMiniTFPlus.get_status, lidars))
            print(status)
# 
#     if False:
#         lidar_C6_detected = lidar_C6.try_to_get_new_measurement()
#         lidar_B1_detected = lidar_B1.try_to_get_new_measurement()
#         if lidar_B1.misaligned or lidar_C6.misaligned:
#             delaytime_us = 0
#         time.sleep_us(delaytime_us)
#         if time.ticks_diff(monitor_at_ms, time.ticks_ms()) <0:
#             monitor_at_ms = time.ticks_add(monitor_at_ms, 1000)
#             print(lidar_B1.distance_m, lidar_B1.detected_ticks_ms, lidar_C6.distance_m,lidar_C6.detected_ticks_ms, 'delaytime_us %d'%delaytime_us)
#             delaytime_us += 1000



