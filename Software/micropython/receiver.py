from machine import Pin, SPI
from nrf24l01 import NRF24L01
import time
import struct

spi = SPI(
    1,
    baudrate=4000000,
    sck=Pin(18),
    mosi=Pin(23),
    miso=Pin(19)
)

csn = Pin(5, Pin.OUT)
ce = Pin(4, Pin.OUT)

nrf = NRF24L01(spi, csn, ce, payload_size=32)
nrf.set_channel(100)

address = b"ABCDE"

nrf.open_rx_pipe(0, address)
nrf.start_listening()

#print("Receiver listening...")

last_x1 = 0
last_y1 = 0
last_sel1 = 1
last_x2 = 0
last_y2 = 0
last_sel2 = 1
last_packet_time = time.ticks_ms()


def unpack():
    global last_x1, last_y1, last_sel1
    global last_x2, last_y2, last_sel2
    global last_packet_time
    
    if nrf.any():
        
        packet = nrf.recv()

        x1, y1, sel1, x2, y2, sel2 = struct.unpack("<hhBhhB", packet)
        
        last_x1 = x1
        last_y1 = y1
        last_sel1 = sel1
        last_x2 = x2
        last_y2 = y2
        last_sel2 = sel2
        last_packet_time = time.ticks_ms()
        
    else:
        x1 = last_x1
        y1 = last_y1
        sel1 = last_sel1
        x2 = last_x2
        y2 = last_y2
        sel2 = last_sel2
        
    elapsed = time.ticks_diff(time.ticks_ms(), last_packet_time)
    
    if elapsed > 100:
        x1 = 0
        y1 = 0 #throttle
        sel1 = 0
        x2 = 0
        y2 = 0
        sel2 = 0
        
    return(x1, y1, sel1, x2, y2, sel2)

last_print = time.ticks_ms()


#while True:
    #uncalibrated_yaw, throttle, sel1, uncalibrated_roll, uncalibrated_pitch, sel2 = unpack()

    #if time.ticks_diff(time.ticks_ms(), last_print) > 50:
        #print(uncalibrated_yaw, throttle, sel1, uncalibrated_roll, uncalibrated_pitch, sel2)
        #last_print = time.ticks_ms()
    
    
        
        #x1 *= 0.03
        #y1 *= 0.03
        

        
    #return(x1, y1, sel1, desired_roll, desired_pitch, sel2)
        
       # print(x1, y1, sel1, x2, y2, sel2)
