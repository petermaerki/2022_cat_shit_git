from pyb import UART
import time

led_alarm = pyb.LED(1)

uart_6 = UART(6, 115200)
#uart_6.init(115200, bits=8, parity=None, stop=1)

uart_1 = UART(1, 115200)
#uart_1.init(115200, bits=8, parity=None, stop=1)

class Distance:
    def __init__(self, uart):
        self.uart = uart
        self.uart.init(115200, bits=8, parity=None, stop=1,read_buf_len=64)
        self.distance_slow_m = self.get_distance()
        self.distance_slow_m_increment_plus = 1E-3
        self.distance_slow_m_increment_minus = 1E-4
        self.alarm_start_ms = 0
        self.timeout_ms = 5000

    def get_distance(self):
        temp = bytes()
        while True:
            if self.uart.any() > 0:
                temp += self.uart.read(9)
                if temp[0] == 0x59 and temp[1] == 0x59 :
                    distance   = (temp[2] + temp[3] * 256 ) / 100.0   #Get distance value  
                    #strength    = temp[4] + temp[5] * 256            #Get Strength value  
                    #temperature= (temp[6] + temp[7]* 256)/8-256      #Get IC temperature value  
                    #print("%0.3f m strength %5d  %5d℃"%(distance,strength,temperature))
                    return distance
                else:
                    # missalign
                    #print('.', end = '')
                    self.uart.read(1) #shift by one
                    temp = bytes()


    def do_measure(self):
        while True:
            self.distance_now_m = self.get_distance()
            if self.distance_now_m > self.distance_slow_m:
                self.distance_slow_m += self.distance_slow_m_increment_plus
            else:
                self.distance_slow_m -= self.distance_slow_m_increment_minus
            #print('distance_now_m %2.3f m, distance_slow_m %2.3f m, difference % 2.3f m,counter %d, ' % (self.distance_now_m, self.distance_slow_m, self.distance_now_m - self.distance_slow_m, self.counter) )
            alarm = self.distance_slow_m - self.distance_now_m > 0.1
            if alarm and self.alarm_start_ms == 0 :
                self.alarm_start_ms = time.ticks_ms()
            if time.ticks_diff(time.ticks_ms(), self.alarm_start_ms) > self.timeout_ms:
                self.alarm_start_ms = 0
            return alarm
    
        
lidar_6 = Distance(uart_6)
lidar_1 = Distance(uart_1)

time_next_print_ms = time.ticks_ms()

alarm_6_start_ms = 0
alarm_1_start_ms = 0

while True:
    lidar_6.do_measure()
    lidar_1.do_measure()
    if time.ticks_diff(time.ticks_ms(), time_next_print_ms) > 0:
        time_next_print_ms = time.ticks_add(time_next_print_ms, 1000)
        if lidar_6.alarm_start_ms or lidar_1.alarm_start_ms:
            print (lidar_6.alarm_start_ms, lidar_1.alarm_start_ms)
        if lidar_6.alarm_start_ms and lidar_1.alarm_start_ms:
            print (time.ticks_diff(lidar_6.alarm_start_ms, lidar_1.alarm_start_ms))
    
    
"""

def get_distance():
    temp = bytes()
    while True:
        if uart.any() > 0:
            temp += uart.read(9)
            if temp[0] == 0x59 and temp[1] == 0x59 :
                distance   = (temp[2] + temp[3] * 256 ) / 100.0   #Get distance value  
                #strength    = temp[4] + temp[5] * 256            #Get Strength value  
                #temperature= (temp[6] + temp[7]* 256)/8-256      #Get IC temperature value  
                #print("%0.3f m strength %5d  %5d℃"%(distance,strength,temperature))
                return distance
            else:
                print('seriell Muell empfangen %s', temp)

distance_slow_m = get_distance()
distance_slow_m_increment_plus = 1E-3
distance_slow_m_increment_minus = 1E-4
counter = 0
while True:
    distance_now_m = get_distance()
    if distance_now_m > distance_slow_m:
        distance_slow_m += distance_slow_m_increment_plus
    else:
        distance_slow_m -= distance_slow_m_increment_minus
    print('distance_now_m %2.3f m, distance_slow_m %2.3f m, difference % 2.3f m,counter %d, ' % (distance_now_m, distance_slow_m, distance_now_m - distance_slow_m, counter) )
    counter += 1
    alarm = distance_slow_m - distance_now_m > 0.1
    if alarm:
        print('alarm')
        led_alarm.on()
    else:
        led_alarm.off()
    
"""
