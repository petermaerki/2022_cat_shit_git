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
        self._distance_slow_m = 0.0
        self._distance_slow_m_increment_plus = 1E-3
        self._distance_slow_m_increment_minus = 1E-4
        self.distance_m = 0.0
        self.signal_strength = None
        self.temperature_C = None
        self.detected = False
        self.detected_edge_processed = True
        self.detected_ticks_ms = 0
        self._detected_edge = False

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
        if self.distance_m > self._distance_slow_m:
            self._distance_slow_m += self._distance_slow_m_increment_plus
        else:
            self._distance_slow_m -= self._distance_slow_m_increment_minus
            
        detected_actual = self._distance_slow_m - self.distance_m < 0.1
        deteced_edge = detected_actual != self.detected
        self.detected = detected_actual

        if deteced_edge: # Falls nicht bereits das letzte mal detektiert
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

    def do_extra_misalign(self): # for testing
        while self._uart.read(1) == None: # none: timeout vom lesen
            time.sleep_us(1)

    def get_status(self):
        delay_ms = time.ticks_diff(time.ticks_ms(), self.detected_ticks_ms)
        return "%s: %0.2fm %6dms" % (self.name, self.distance_m, delay_ms)

print('hugo')

lidar_6 = LidarMiniTFPlus(uart_6, "lidar_6")
lidar_1 = LidarMiniTFPlus(uart_1, "lidar_1")

monitor_at_ms = time.ticks_ms()

delaytime_us = 0
start_ticks_ms = time.ticks_ms()

lidars = (lidar_6, lidar_1)

while True:
    for lidar in lidars:
        lidar.process_input_data()
    
        if lidar.edge_detected():
            print(lidar.name)
    
        if time.ticks_diff(monitor_at_ms, time.ticks_ms()) <0:
            monitor_at_ms = time.ticks_add(monitor_at_ms, 1000)
            status = ", ".join(map(LidarMiniTFPlus.get_status, lidars))
            print(status)
# 
#     if False:
#         lidar_6_detected = lidar_6.try_to_get_new_measurement()
#         lidar_1_detected = lidar_1.try_to_get_new_measurement()
#         if lidar_1.misaligned or lidar_6.misaligned:
#             delaytime_us = 0
#         time.sleep_us(delaytime_us)
#         if time.ticks_diff(monitor_at_ms, time.ticks_ms()) <0:
#             monitor_at_ms = time.ticks_add(monitor_at_ms, 1000)
#             print(lidar_1.distance_m, lidar_1.detected_ticks_ms, lidar_6.distance_m,lidar_6.detected_ticks_ms, 'delaytime_us %d'%delaytime_us)
#             delaytime_us += 1000



