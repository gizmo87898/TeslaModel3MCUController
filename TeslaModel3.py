import time
import can
import random 
import socket
import struct
import select 
import threading
import tkinter as tk
import win_precise_time as wpt
from datetime import datetime

#Initalize and connect to CANbuses
vehicle_bus = can.interface.Bus(channel='com10', bustype='slcan', bitrate=500000)
chassis_bus = can.interface.Bus(channel='com11', bustype='slcan', bitrate=500000)
private_bus = can.interface.Bus(channel='com12', bustype='slcan', bitrate=500000)

#Connect to Beam.NG socket from Outgauge
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 4444))
    
# Timers for sending messages
start_time_100ms = time.time()
start_time_10ms = time.time()
start_time_5s = time.time()

#ID for bruteforcing
id_counter = 0

#Counters for messages
counter_8bit_100ms = 0
counter_4bit_100ms = 0


#Inital values from Outgauge
ignition = True # Ignition switch
speed = 0 # Vehicle speed in MPH
gear = "P" # Current gear
coolant_temp = 90 # C
oil_temp = 90 # C
battery = 100 # 0 to 100
lv_battery = 12 # V
drive_mode = 2 # sport/comfort/etc
front_left = 30 # PSI
front_right = 30 # PSI
rear_left = 30 # PSI
rear_right = 30 # PSI

left_directional = False
right_directional = False
tc_active = False
abs_active = False
parking_brake = False

highbeam = False
foglight = False
lowbeam = False 
parking_lights = False

hood = False
trunk = False
door_fl = False
door_fr = False
door_rl = False
door_rr = False

airbag = False
seatbelt = False


def gui_thread():
    root = tk.Tk()
    root.title("Tesla Steering Wheel Controls")
    update_button = tk.Button(root, text="BC", command=lambda: None)
    update_button.pack(pady=10)
    update_button.bind("<ButtonPress>", on_button_press)
    update_button.bind("<ButtonRelease>", on_button_release)
    root.mainloop()

# Start the GUI thread
gui_thread = threading.Thread(target=gui_thread)
gui_thread.start()

#Recieve from all the buses
def receive():
    while True:
        vehicle_message = vehicle_bus.recv()
        chassis_message = chassis_bus.recv()
        private_message = private_bus.recv()

        print("VEH: " + vehicle_message)
        print("CHA: " + chassis_message)
        print("PRI: " = private_message)

receive = threading.Thread(target=receive)    
receive.start()

#Main loop
while True:
    current_time = time.time()
    
    #Read from the socket if there is data to be read
    ready_to_read, _, _ = select.select([sock], [], [], 0)
    if sock in ready_to_read:
        data, _ = sock.recvfrom(256)
        packet = struct.unpack('I4sHc2c7f2I3f16s16si', data)
        speed = int(packet[6]*2.5) #convert speed to km/h
        coolant_temp = int(packet[9])
        oil_temp = int(packet[12])
        fuel = int(packet[10]*100)
        gear = packet[4]
        left_directional = False
        right_directional = False
        highbeam = False
        abs = False
        battery = False
        tc = False
        handbrake = False
        shiftlight = False
        ignition = False
        lowpressure = False
        check_engine = False
        foglight = False
        lowbeam = False
        
        if (packet[14]>>0)&1:
            shiftlight = True
        if (packet[14]>>1)&1:
            highbeam = True
        if (packet[14]>>2)&1:
            handbrake = True
        if (packet[14]>>4)&1:
            tc_active = True
        if (packet[14]>>10)&1:
            abs_active = True
        if (packet[14]>>5)&1:
            left_directional = True
        if (packet[14]>>6)&1:
            right_directional = True
        if (packet[14]>>11)&1:
            ignition = True
        if (packet[14]>>12)&1:
            lowpressure = True
        if (packet[14]>>13)&1:
            check_engine = True
        if (packet[14]>>14)&1:
            foglight = True
        if (packet[14]>>15)&1:
            lowbeam = True

    # Send each message every 100ms
    elapsed_time_100ms = current_time - start_time_100ms
    if elapsed_time_100ms >= 0.1:
        date = datetime.now()
        messages_100ms_vehicle = [
            
            can.Message(arbitration_id=0x1, data=[ # none
                0,0,0,0,0,0,0,0], is_extended_id=False), 
            
            can.Message(arbitration_id=id_counter, data=[
                random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255)], is_extended_id=False),
        ]
        messages_100ms_chassis = [
            
            can.Message(arbitration_id=0x1, data=[ # none
                0,0,0,0,0,0,0,0], is_extended_id=False), 
            
            can.Message(arbitration_id=id_counter, data=[
                random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255)], is_extended_id=False),
        ]
        messages_100ms_private = [
            
            can.Message(arbitration_id=0x1, data=[ # none
                0,0,0,0,0,0,0,0], is_extended_id=False), 
            
            can.Message(arbitration_id=id_counter, data=[
                random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255)], is_extended_id=False),
        ]
        #Update checksums and counters here
        counter_8bit_100ms = (counter_8bit + 1) % 256
        counter_4bit_100ms = (counter_4bit_100ms + 1) % 15

        # Send Messages
        for message in messages_100ms_vehicle:
            vehicle_bus.send(message)
            #print("SEND 100ms: VEH: " + message)
            wpt.sleep(0.001)
        for message in messages_100ms_chassis:
            vehicle_bus.send(message)
            #print("SEND 100ms: CHA: " + message)
            wpt.sleep(0.001)
        for message in messages_100ms_private:
            vehicle_bus.send(message)
            #print("SEND 100ms: PRI: " + message)
            wpt.sleep(0.001)
        start_time_100ms = time.time()


    # Execute code every 10ms
    elapsed_time_10ms = current_time - start_time_10ms
    if elapsed_time_10ms >= 0.01:  # 10ms
        messages_10ms_vehicle = [
            can.Message(arbitration_id=0x1, data=[ # none
                0,0,0,0,0,0,0,0], is_extended_id=False),    
        ]
        messages_10ms_chassis = [
            can.Message(arbitration_id=0x1, data=[ # none
                0,0,0,0,0,0,0,0], is_extended_id=False),    
        ]
        messages_10ms_private = [
            can.Message(arbitration_id=0x1, data=[ # none
                0,0,0,0,0,0,0,0], is_extended_id=False),    
        ]

        for message in messages_10ms_vehicle:
            bus.send(message)
            #print("SEND 10ms: VEH: " + message)
            wpt.sleep(0.001)
        for message in messages_10ms_chassis:
            bus.send(message)
            #print("SEND 10ms: CHA: " + message)
            wpt.sleep(0.001)
        for message in messages_10ms_private:
            bus.send(message)
            #print("SEND 10ms: PRI: " + message)
            wpt.sleep(0.001)
        start_time_10ms = time.time()

    # Execute code every 5s
    elapsed_time_5s = current_time - start_time_5s
    if elapsed_time_5s >= 5:
        id_counter += 1
        print(hex(id_counter))
        if id_counter == 0x7ff:
            id_counter = 0

        start_time_5s = time.time()


sock.close()

