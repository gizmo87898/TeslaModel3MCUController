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


# Timers for sending messages
start_time_100ms = time.time()
start_time_20ms = time.time()
start_time_5s = time.time()

#ID for bruteforcing
#unknown ids:
#0x1f9 - makes the mcu say "tap keycard to drive"

#CHECK 0x257, should be speed

#working an unadded yet: (VEHICLE BUS)
#0x212 - charging stuff
#0x128 - speed limiter and maybe cruise control
#0x243 - sets climate control status on mcu
#0x249 - sets wiper status
#0x25e - brings up charging menu
#0x2b6 - parking brake hold
#0x2b7? - speed, showed 27mph at some point
#0x2e1 - frunk status, also makes the mcu restart (not fully) after a few seconds of it being sent
#0x304 - incompatible drive system software detected error message
#0x31e - charging error messages
#0x320 - general error messages: pull over now, acceleration is slower,unable to drive, charging not enough
#0x321 - outside temp
#0x338 - more charging errors
#0x339 - keyless driving enabled message
#0x340 - more general error messages: wipers stopping, pull over safely (showing on left on car), update required, lv battery cable loose
#0x342/3 - if there is a current "stop vehicle" fault, this message makes it show red at the bottom too
#0x352 - battery percentage and tempurature (snow icon next to percentage)
#0x356 - motor errors: motor overheating, power reduced, 
#0x357 - more motor errors: pull over safely, okay to drive, reduced power
#0x35a - powertrain errors: okay to drive but needs service, service now, 
#0x35b - more errors: gearbox fluid service, adaptive air suspension fault
#0x360 - more errors: manual door released used, dyno mode, secure second row seats, steering wheel buttons unavailable, seats motors used too much
#0x367 - more errors: hold n for neutral, parking brake not applied, pull over safely with red flashing, power reduced
#0x368 - more errors: pull over now: unable to drive with red flashing, cruise control disabled
#0x36b - DYNO MODE, more errors: odometer invalid, tire tread low, tap keycard to drive, autopark aborted
#0x36e - more errors: auto hold disabled/unavailable, adaptive vehicle ride, wait for system to power up, vehicle auto shifted to N, stability control disabled, high brake temp
#0x37f - charing errors: unabel to charge with chademo adapter, dc fast charging unavailable, charging fault, charger temp high
#0x384 + 0x385 - incompatible drive system software detected fault
#0x3a1 - fasten seatbelt messages
#0x3a4 - powerwall errors: unable to charge powerwall level too high, power conversion needs service,
#0x3a5 - motor errors: motor overheating, power reduced
#0x3aa - more errors: vehicle shutting down, unable to charge, vehicle config mismatch
#0x3b5 - more errors: power limited, okay to drive, power reduced
#0x3b6 - SETS ODOMETER
#0x3be - charger errors: unable to dc fast charge, charger fault, charger temp too high
#0x3c0 - manual ac off, cabin heating disabled, secure second row, manual door release used
#0x3c2 - steering wheel controls i think - shows mic icon, adjusts autopilot car in front distance, driver profile adjusted, volume
#0x3c5 - powertrain errors: okay to drive, requires service
#0x3c8 - transport mode: parkbrake not set, ebrake applied, braking degraded
#0x3e2 - trailer light: makes it go red/yellow
#0x3e3 - brake light/rear fog light status: makes the car show brake lights and foglight icon
#0x3e5 - more errors: adaptive air suspension, gearbox fluid service - seems the same as 0x35b
#0x3e8 - transport mode: parkbrake not set, vehicle will remain in N, dyno mode (not full screen text)
#0x3e9 - folding mirrors
#0x3f5 - light status: highbeam, parking lights, directionals, foglights, lowbeams
#0x3f9 - more errors: walk-away unavailable, use keycard, dyno mode, tpms fault, key battery low, update reservation
#0x46c - charging errors: wall charger not configured, temp too high, 
#CHASSIS BUS:
#0x145 - ABS light
#0x329 - Autopilot faults, AEB in progress, AEB unavailable
#0x350 - Autopark
#0x35d power brake reduced
#0x369,36a, 36c - take control immidately, vehicle departing lane
#0x36f - tpms stuff - new wheels detected, low tire pressure, tpms fault
#0x370 - Steering assist fault
#0x371 - safety restarint system faults
#0x389 - blinking yellow steering wheel icon nexto to gear indicator
#0x3d5 - ABS disabled message, brake fluid low message
#0x3f1 - srs faults, airbag light DO NOT SEND THIS IT MAKES THE WHOLE MCU FREEZE FOR A FEW SECS
#0x3f4 - Camera calibration
#0x439 - autosteer/cruisecontrol faults
#0x43a - Autopark, and auto lane change unavailable
#0x449 - autopilot errors: lane departure avoidance, autopilot unavailable, gps antenna error
#0x459 autopilot errors: gps antenna disconnected, update required, autopilot unavailable, autopilot computer needs rebooting
#0x479 - cameras dirty autocleaning, FSD unavailable at current location, autosilot camera requires service, cabin camera unavailable, 

id_counter = 0x31c
test_timer = 5
test_counter = 0

#Counters for messages
counter_8bit_100ms = 0
counter_4bit_100ms = 0


#Inital values from Outgauge
ignition = True # Ignition switch
speed = 0 # Vehicle speed in MPH
gear = 1 # Current gear: 1 = P, 2 = R, 3 = N, 4 = D

outsideTemp = 72
#0 = -40F
#0x21/33 = -10F
#0x2c/44 = 0F
#0x50/80 = 32F
#0x7c/124 = 72F
#0x9b/155 = 100F
#0xb2/178 = 120F
#0xff/255 = 190F

front_left_psi = 30 # PSI
front_right_psi = 30 # PSI
rear_left_psi = 30 # PSI
rear_right_psi = 30 # PSI

wheelspeedFL = 30
wheelspeedFR = 30
wheelspeedRR = 30
wheelspeedRL = 30
wheelspeedCounter = 0
wheelspeedChecksum = 0

rawFL = int(wheelspeedFL / 0.04)
rawFR = int(wheelspeedFR / 0.04)
rawRL = int(wheelspeedRL / 0.04)
rawRR = int(wheelspeedRR / 0.04)


chargeHoursRemaining = 48  # minutes
rawBattCurrent = 200.0  # Amps, -340 = full regen, 0-10 = middle, 650 = full power, this is for the bar at the top that shows regen/power
battVoltage = 400.0  # Volts

#24 = 0%
#25 = 1%
#50 = 34%
#75 = 67%
#85 = 80%
#99.7 = 100%
battPercentage = 100 # this shows actual battery percentage at the top

#0x352
nominalFullPackEnergy = 100 # kWh
nominalEnergyRemaining = battPercentage # kWh
expectedEnergyRemaining = battPercentage # kWh
idealEnergyRemaining = battPercentage # kWh
energyToChargeComplete = 2 # kWh
energyBuffer = 3 # kWh
fullChargeComplete = False



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


#def gui_thread():
#    root = tk.Tk()
#    root.title("Tesla Steering Wheel Controls")
#    update_button = tk.Button(root, text="BC", command=lambda: None)
#    update_button.pack(pady=10)
#    update_button.bind("<ButtonPress>", on_button_press)
#    update_button.bind("<ButtonRelease>", on_button_release)
#    root.mainloop()

# Start the GUI thread
#gui_thread = threading.Thread(target=gui_thread)
#gui_thread.start()

#Recieve from all the buses
def cha_receive():
    while True:
        chassis_message = chassis_bus.recv()

        #print("CHA: " + str(chassis_message))

cha_receive = threading.Thread(target=cha_receive)    
cha_receive.start()
def veh_receive():
    while True:
        vehicle_message = vehicle_bus.recv()

        #print("VEH: " + str(vehicle_message))

veh_receive = threading.Thread(target=veh_receive)    
veh_receive.start()

    
#Main loop
while True:
    current_time = time.time()
    # Scaling and offset calculations
    chargeHoursRemaining_scaled = int(chargeHoursRemaining)
    rawBattCurrent_scaled = int((rawBattCurrent + 822) / -0.05)
    smoothBattCurrent_scaled = int(rawBattCurrent / -0.1)
    battVoltage_scaled = int(battVoltage / 0.01)

    # Send each message every 100ms
    elapsed_time_100ms = current_time - start_time_100ms
    if elapsed_time_100ms >= 0.1:
        date = datetime.now()
        messages_100ms_vehicle = [
            can.Message(arbitration_id=0x118, data=[ # Drive system status (gear)
                0xFA,0x61,gear*32,0x00,0x00,0x88,0x70,0xFB], is_extended_id=False), #3rd byte is gear
            
            can.Message(arbitration_id=0x3b6, data=[ # Set Odometer, 0 = 0mi, 0x25,0x03 = 1mi, 0x6f,0x09 = 2mi, 0xb8,0x0f = 3mi
                0xb8,0x0f,0,0], is_extended_id=False), 

            can.Message(arbitration_id=0x321, data=[ # Vehicle Controller Front Sensors - Outside temp, brake fluid, coolant level, washer fluid level
                0x28,0x82,0xa8,int((outsideTemp+40)*1.11),0x01, int((outsideTemp+40)*1.11),0x30,0xa6], is_extended_id=False), 

            can.Message(arbitration_id=0x221, data=[ # Vehicle Controller Front LV Power State
                ignition*96,0,0,0,0,0,0,0], is_extended_id=False), 

            can.Message(arbitration_id=0x212, data=[ # BMS Status - Makes MCU show charging - 2nd byte: 8 - Ready to charge, 16 - Starting to charge, 24 - chargine complete, 32 - green charging bar, 40 - charging stopped, 48 - no message
                0xb9,16,0x93,0x0c,0x01,0xff,0x3f,0x01], is_extended_id=False), 

            #can.Message(arbitration_id=0x2e1, data=[ # Vehicle Controller Front, this one is a multiplex message and i havent added that yet so its commented out. Controls frunk status
            #    0b001111,0x33,0x00,0x00,0xA4,0x1A,0xA1,0x09], is_extended_id=False), 

            
            can.Message(arbitration_id=0x352, data=[ # HV Battery Status
                (((
                    (int(round(nominalFullPackEnergy * 10))      & ((1 << 11) - 1)) << 0  |  # 0|11
                    (int(round(nominalEnergyRemaining * 10))     & ((1 << 11) - 1)) << 11 |  # 11|11
                    (int(round(expectedEnergyRemaining * 10))    & ((1 << 11) - 1)) << 22 |  # 22|11
                    (int(round(idealEnergyRemaining * 10))       & ((1 << 11) - 1)) << 33 |  # 33|11
                    (int(round(energyToChargeComplete * 10))     & ((1 << 11) - 1)) << 44 |  # 44|11
                    (int(round(energyBuffer * 10))               & ((1 << 8)  - 1)) << 55 |  # 55|8
                    ((1 if fullChargeComplete else 0)            & 0x1)              << 63    # 63|1
                ) >> (8 * i)) & 0xFF) for i in range(8)
            ], is_extended_id=False), 


            can.Message(arbitration_id=0x132, data = [ #rawbattcurrent: -340 = full regen, 
                battVoltage_scaled & 0xFF, (battVoltage_scaled >> 8) & 0xFF,  # 0-16: BattVoltage
                smoothBattCurrent_scaled & 0xFF, (smoothBattCurrent_scaled >> 8) & 0xFF,  # 16-32: SmoothBattCurrent
                rawBattCurrent_scaled & 0xFF, (rawBattCurrent_scaled >> 8) & 0xFF,  # 32-48: RawBattCurrent
                chargeHoursRemaining_scaled & 0xFF, (chargeHoursRemaining_scaled >> 8) & 0x0F  # 48-60: ChargeHoursRemaining
            ], is_extended_id=False),

            can.Message(arbitration_id=0x102, data=[ # Door status left
                0x22,0x33,0x00,0x00,0xA4,0x1A,0xA1,0x09], is_extended_id=False), 

            can.Message(arbitration_id=0x122, data=[ # Door status left2
                0x00,0x00,0x12,0x12,0xd2,0x00], is_extended_id=False), 

            can.Message(arbitration_id=0x103, data=[ # Door status right
                0x22,0x33,0x00,0x00,0xAa,0x1c,0x21,0x32], is_extended_id=False), 
            
            can.Message(arbitration_id=id_counter, data=[
                random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255)], is_extended_id=False)
        ]

        messages_100ms_chassis = [
            can.Message(
                arbitration_id=0x145,data=[0,0,0,0,0,0,0,0],is_extended_id=False # This needs a bunch of variables added, but it at least gets the ABS light off for now.
            ),
            can.Message(
                arbitration_id=0x175,
                data=[
                    (rawFL & 0xFF),                             # Byte 1: first 8 bits of FL
                    ((rawFL >> 8) & 0x1F) | ((rawFR & 0x7) << 5),  # Byte 2: last 5 bits of FL + first 3 bits of FR
                    (rawFR >> 3) & 0xFF,                         # Byte 3: middle 8 bits of FR (bits 16-23)
                    ((rawFR >> 11) & 0x3) | ((rawRL & 0x3F) << 2),  # Byte 4: last 2 bits of FR + first 6 bits of RL
                    ((rawRL >> 6) & 0x7F) | ((rawRR & 0x1) << 7),  # Byte 5: last 7 bits of RL + first bit of RR
                    (rawRR >> 8) & 0xFF,                         # Byte 6: middle 8 bits of RR (bits 40-47)
                    ((rawRR >> 15) & 0xF) | ((wheelspeedCounter & 0xF) << 4),  # Byte 7: last 4 bits of RR + counter (bits 52-55)
                    (wheelspeedChecksum & 0xFF)                  # Byte 8: checksum (bits 56-63)
                ],
                is_extended_id=False
            ),
            can.Message(arbitration_id=id_counter, data=[
                random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255)], is_extended_id=False),
        ]
        

        
        #Update checksums and counters here
        counter_8bit_100ms = (counter_8bit_100ms + 1) % 256
        counter_4bit_100ms = (counter_4bit_100ms + 1) % 15

        # Send Messages
        for message in messages_100ms_vehicle:
            vehicle_bus.send(message)
            #print("SEND 100ms: VEH: " + message)
            wpt.sleep(0.001)
        for message in messages_100ms_chassis:
            chassis_bus.send(message)
            #print("SEND 100ms: CHA: " + message)
            wpt.sleep(0.001)

        start_time_100ms = time.time()


    # Execute code every 20ms
    elapsed_time_20ms = current_time - start_time_20ms
    if elapsed_time_20ms >= 0.02:  # 20ms

        messages_20ms_vehicle = [
            can.Message(arbitration_id=0x257, data=[ # Vehicle Speed -- TODO: this should be the one that displays on the MCU
                random.randint(0,255),random.randint(0,255),0x22,0x04,0x02,0x38,0x11,0x01], is_extended_id=False),    
        ]
        messages_20ms_chassis = [
            #can.Message(arbitration_id=0x1, data=[ # none
            #    0,0,0,0,0,0,0,0], is_extended_id=False),    
        ]

        for message in messages_20ms_vehicle:
            vehicle_bus.send(message)
            #print("SEND 20ms: VEH: " + message)
            wpt.sleep(0.001)
        for message in messages_20ms_chassis:
            chassis_bus.send(message)
            #print("SEND 20ms: CHA: " + message)
            wpt.sleep(0.001)

        start_time_20ms = time.time()

    # Execute code every 5s
    elapsed_time_5s = current_time - start_time_5s
    if elapsed_time_5s >= test_timer:
        id_counter += 1
        test_counter += 1
        print(test_counter)
        print(hex(id_counter))
        if id_counter == 0x7ff:
            id_counter = 0

        start_time_5s = time.time()


sock.close()

