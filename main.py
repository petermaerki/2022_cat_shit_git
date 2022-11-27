from pyb import UART
import time

led_alarm = pyb.LED(1)

uart_6 = UART(6, 115200)
uart_1 = UART(1, 115200)
uart_3 = UART(3, 115200)


class LidarMiniTFPlus:
    def __init__(self, uart, name):
        self.name = name
        self._misaligned = False
        self._uart = uart
        self._uart.init(115200, bits=8, parity=None, stop=1,read_buf_len=64)
        self.distance_slow_m = 0.0
        self._distance_slow_m_increment = {True: 1E-3, False: -1E-4}
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
        #if self.name == 'lidar_A': # todo: remove when Lidar A is available
        #    return
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
        self.distance_slow_m += self._distance_slow_m_increment[self.distance_m > self.distance_slow_m]
            
        detected_actual = self.distance_slow_m - self.distance_m > 0.1 #and self.distance_slow_m > 0.3
        deteced_edge = detected_actual != self.detected
        self.detected = detected_actual

        if deteced_edge:
            if detected_actual:
                self.detected_edge_processed = True
                self.detected_ticks_ms = time.ticks_ms()
    
    def edge_detected(self):
        """
        Return True if a edge was observed.
        Will only return once True for every edge.
        """
        tmp = self.detected_edge_processed
        self.detected_edge_processed = False
        return tmp

    def get_status(self):
        return "%s: %0.2fm (%0.2fm) %6dms" % (self.name, self.distance_m, self.distance_slow_m, self.get_delay_ms())

    def get_delay_ms(self):
        delay_ms = time.ticks_diff(time.ticks_ms(), self.detected_ticks_ms)
        return delay_ms

lidar_A = LidarMiniTFPlus(uart_3, "lidar_A")
lidar_B = LidarMiniTFPlus(uart_1, "lidar_B")
lidar_C = LidarMiniTFPlus(uart_6, "lidar_C")

monitor_at_ms = time.ticks_ms() + 50

delaytime_us = 0
start_ticks_ms = time.ticks_ms()

lidars = (lidar_A, lidar_B, lidar_C)

long_ago_forget_ms = 5000

def decision_A():
    if not lidar_A.edge_detected():
        return
    print('A detected')
    if lidar_C.get_delay_ms() < long_ago_forget_ms and lidar_B.get_delay_ms() < long_ago_forget_ms and lidar_C.get_delay_ms() > lidar_B.get_delay_ms():
        print('Car exit')
        return
    if lidar_C.get_delay_ms() > long_ago_forget_ms and lidar_B.get_delay_ms() > long_ago_forget_ms:
        print('Car at lidar A.')
        return
    if lidar_C.get_delay_ms() > long_ago_forget_ms and lidar_B.get_delay_ms() < long_ago_forget_ms:
        print('Strange, B, then A but not C. Should not happen.')
        return
    print('Assert, should never get here')

def decision_B():
    if not lidar_B.edge_detected():
        return
    print('B detected')
    if lidar_C.get_delay_ms() < long_ago_forget_ms:
        print('C was detected before, something exit.')
        return
    if lidar_A.get_delay_ms() < long_ago_forget_ms:
        print('A was detected before. Car enter.')
        return
    print('Cat enter at Lidar B?')

def decision_C():
    if not lidar_C.edge_detected():
        return
    print('C detected')
    if lidar_A.get_delay_ms() < long_ago_forget_ms:
        print('A was detected => car entering')
        return
    if lidar_B.get_delay_ms() > long_ago_forget_ms:
        print('B was not detected before => something exit.')
        return
    print('=> Cat entering, spray water')

while True:
    for lidar in lidars:
        lidar.process_input_data()
    
    decision_A()
    decision_B()
    decision_C()
    
    if time.ticks_diff(monitor_at_ms, time.ticks_ms()) < 0:
        monitor_at_ms = time.ticks_add(monitor_at_ms, 1000)
        status = ", ".join(map(LidarMiniTFPlus.get_status, lidars))
        print(status)




