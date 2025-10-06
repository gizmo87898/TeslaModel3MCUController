import time
import can
import random 
import threading
import tkinter as tk
import win_precise_time as wpt
from datetime import datetime
from collections import deque

#Initalize and connect to CANbuses
vehicle_bus = can.interface.Bus(channel='COM10', bustype='slcan', bitrate=500000, write_timeout=1.0)
chassis_bus = can.interface.Bus(channel='COM11', bustype='slcan', bitrate=500000, write_timeout=1.0)

# Timers for sending messages
start_time_100ms = time.time()
start_time_20ms = time.time()
start_time_5s = time.time()

unknown_vehicle_msgs = deque(maxlen=50)  # holds can.Message objects
unknown_chassis_msgs = deque(maxlen=50)  # holds can.Message objects

#ID for bruteforcing
#unknown ids:
#0x1f9 - makes the mcu say "tap keycard to drive"

#CHECK 0x257, should be speed

#working an unadded yet: (VEHICLE BUS)
#0x128 - Speedlimiter and cruise control
#0x212 - charging stuff
#0x228 - parking brake right (EPB Right Status)
#0x243 - sets climate control status on mcu (Vehicle Controller Right HVAC Status)
#0x249 - sets wiper status (SCCM Left Stalk)
#0x25e - brings up charging menu
#0x2b6 - parking brake hold (DI Chassis Control Status)
#0x2b7? - speed, showed 27mph at some point
#0x2e1 - frunk status, also makes the mcu restart (not fully) after a few seconds of it being sent (Vehicle Controller Front Status)
#0x304 - incompatible drive system software detected error message
#0x31e - charging error messages
#0x320 - general error messages: pull over now, acceleration is slower,unable to drive, charging not enough
#0x321 - outside temp (Vehicle Controller Front Sensors)
#0x338 - more charging errors
#0x339 - keyless driving enabled message + vehicle locked/unlocked status
#0x340 - more general error messages: wipers stopping, pull over safely (showing on left on car), update required, lv battery cable loose
#0x342/3 - if there is a current "stop vehicle" fault, this message makes it show red at the bottom too
#0x352 - battery percentage and tempurature (snow icon next to percentage) (BMS Energy Status)
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
#0x3a1 - fasten seatbelt messages (Vehicle Controller Front Vehicle Status)
#0x3a4 - powerwall errors: unable to charge powerwall level too high, power conversion needs service,
#0x3a5 - motor errors: motor overheating, power reduced
#0x3aa - more errors: vehicle shutting down, unable to charge, vehicle config mismatch
#0x3b5 - more errors: power limited, okay to drive, power reduced
#0x3b6 - SETS ODOMETER
#0x3be - charger errors: unable to dc fast charge, charger fault, charger temp too high
#0x3c0 - manual ac off, cabin heating disabled, secure second row, manual door release used
#0x3c2 - steering wheel controls - shows mic icon, adjusts autopilot car in front distance, driver profile adjusted, volume (Vehicle Controller Left Switch Status)
#0x3c5 - powertrain errors: okay to drive, requires service
#0x3c8 - transport mode: parkbrake not set, ebrake applied, braking degraded
#0x3e2 - trailer light: makes it go red/yellow (Vehicle Controller Left Light Status)
#0x3e3 - brake light/rear fog light status: makes the car show brake lights and foglight icon (Vehicle Controller Right Light Status)
#0x3e5 - more errors: adaptive air suspension, gearbox fluid service - seems the same as 0x35b
#0x3e8 - transport mode: parkbrake not set, vehicle will remain in N, dyno mode (not full screen text)
#0x3e9 - folding mirrors (DAS Body Controls)
#0x3f5 - light status: highbeam, parking lights, directionals, foglights, lowbeams (Vehicle Controller Front Lighting)
#0x3f9 - more errors: walk-away unavailable, use keycard, dyno mode, tpms fault, key battery low, update reservation
#0x46c - charging errors: wall charger not configured, temp too high, 
#CHASSIS BUS:
#0x145 - ABS light (ESP Status)
#0x329 - Autopilot faults, AEB in progress, AEB unavailable
#0x350 - Autopark
#0x35d power brake reduced
#0x369,36a, 36c - take control immidately, vehicle departing lane
#0x36f - tpms stuff - new wheels detected, low tire pressure, tpms fault
#0x370 - Steering assist fault
#0x371 - safety restarint system faults
#0x389 - blinking yellow steering wheel icon nexto to gear indicator (DAS Status 2)
#0x3d5 - ABS disabled message, brake fluid low message
#0x3f1 - srs faults, airbag light DO NOT SEND THIS IT MAKES THE WHOLE MCU FREEZE FOR A FEW SECS
#0x3f4 - Camera calibration
#0x439 - autosteer/cruisecontrol faults
#0x43a - Autopark, and auto lane change unavailable
#0x449 - autopilot errors: lane departure avoidance, autopilot unavailable, gps antenna error
#0x459 autopilot errors: gps antenna disconnected, update required, autopilot unavailable, autopilot computer needs rebooting
#0x479 - cameras dirty autocleaning, FSD unavailable at current location, autosilot camera requires service, cabin camera unavailable, 

id_counter = 0x0
test_timer = 1

test_counter = 0

ui_status_message = can.Message(arbitration_id=0x353, data=[0,0,0,0,0,0,0,0], is_extended_id=False)
ui_range_message = can.Message(arbitration_id=0x33A, data=[0,0,0,0,0,0,0,0], is_extended_id=False)
ui_charge_message = can.Message(arbitration_id=0x333, data=[0,0,0,0,0], is_extended_id=False)
ui_odo_message = can.Message(arbitration_id=0x3F3, data=[0,0,0], is_extended_id=False)
gps_latlong_message = can.Message(arbitration_id=0x04F, data=[0,0,0,0,0,0,0,0], is_extended_id=False)
utc_time_message = can.Message(arbitration_id=0x318, data=[0,0,0,0,0,0,0,0], is_extended_id=False)
ui_car_config_message = can.Message(arbitration_id=0x7FF, data=[0,0,0,0,0,0,0,0], is_extended_id=False)

vin_mux_last = None
vin_segments = {16: None, 17: None, 18: None}  # VINA/VINB/VINC by mux value
vin_string = ""
swc_frame_bytes = None  # latest SWC payload to send from main loop (or None)

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

def gui_thread():
    # ----- Window -----
    window = tk.Tk()
    window.title("Tesla UI Decode")

    # ====== Root grid (4 columns, single top row + bottom unknowns) ======
    root = tk.Frame(window)
    root.pack(fill="both", expand=True, padx=8, pady=8)
    for c in range(4):
        root.columnconfigure(c, weight=1)

    # ---------- LEFT CLUSTER (cols 0-1): SWC on top, info columns right under it ----------
    left_cluster = tk.Frame(root)
    left_cluster.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=(0,8))
    left_cluster.columnconfigure(0, weight=1)
    left_cluster.columnconfigure(1, weight=1)

    # SWC spans both subcolumns inside left_cluster
    frame_swc = tk.LabelFrame(left_cluster, text="Steering Wheel Controls (0x3C2)", padx=8, pady=8)
    frame_swc.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0,8))

    # Two info columns directly under SWC (no gap now)
    col0  = tk.Frame(left_cluster); col0.grid(row=1, column=0, sticky="nsew", padx=(0,8))
    col1  = tk.Frame(left_cluster); col1.grid(row=1, column=1, sticky="nsew")

    # ---------- RIGHT SIDE (cols 2 & 3): Car Config split ----------
    frame_cfg_a = tk.LabelFrame(root, text="Car Config (0x7FF) — m1 + m2", padx=8, pady=8)
    frame_cfg_a.grid(row=0, column=2, sticky="nsew", padx=(0,8))
    frame_cfg_b = tk.LabelFrame(root, text="Car Config (0x7FF) — m3 + m4+", padx=8, pady=8)
    frame_cfg_b.grid(row=0, column=3, sticky="nsew")

    text_cfg_a = tk.StringVar(value="Waiting for 0x7FF (m1–m2)...")
    text_cfg_b = tk.StringVar(value="Waiting for 0x7FF (m3+)...")
    tk.Label(frame_cfg_a, textvariable=text_cfg_a, justify="left", anchor="nw").pack(fill="both", expand=True)
    tk.Label(frame_cfg_b, textvariable=text_cfg_b, justify="left", anchor="nw").pack(fill="both", expand=True)

    # ---------- SWC controls ----------
    btns = {}

    # Left knob cluster
    tk.Label(frame_swc, text="Left Knob").grid(row=0, column=0, pady=(0,6))
    left_panel = tk.Frame(frame_swc)
    left_panel.grid(row=1, column=0, sticky="ew", padx=(0,12))
    for c in range(3):
        left_panel.columnconfigure(c, weight=1)

    # Right knob cluster
    tk.Label(frame_swc, text="Right Knob").grid(row=0, column=1, pady=(0,6))
    right_panel = tk.Frame(frame_swc)
    right_panel.grid(row=1, column=1, sticky="ew")
    for c in range(3):
        right_panel.columnconfigure(c, weight=1)

    def make_swc_panel(panel, side):
        btns[(side,"up")]    = tk.Button(panel, text="Up")
        btns[(side,"left")]  = tk.Button(panel, text="Left")
        btns[(side,"click")] = tk.Button(panel, text="Click")
        btns[(side,"right")] = tk.Button(panel, text="Right")
        btns[(side,"down")]  = tk.Button(panel, text="Down")
        btns[(side,"up")].grid(   row=0, column=1, sticky="ew", padx=4, pady=2)
        btns[(side,"left")].grid( row=1, column=0, sticky="ew", padx=4, pady=2)
        btns[(side,"click")].grid(row=1, column=1, sticky="ew", padx=4, pady=2)
        btns[(side,"right")].grid(row=1, column=2, sticky="ew", padx=4, pady=2)
        btns[(side,"down")].grid( row=2, column=1, sticky="ew", padx=4, pady=2)

    make_swc_panel(left_panel,  "left")
    make_swc_panel(right_panel, "right")

    # ---------- INFO PANES under SWC ----------
    # COL 0
    frame_ui = tk.LabelFrame(col0, text="UI Status (0x353)", padx=8, pady=8)
    frame_ui.pack(fill="x", padx=0, pady=(0,8))
    text_ui = tk.StringVar(value="Waiting for 0x353...")
    tk.Label(frame_ui, textvariable=text_ui, justify="left", anchor="w").pack(fill="x")

    frame_range = tk.LabelFrame(col0, text="Range / SOC (0x33A)", padx=8, pady=8)
    frame_range.pack(fill="x", padx=0, pady=(0,0))
    text_range = tk.StringVar(value="Waiting for 0x33A...")
    tk.Label(frame_range, textvariable=text_range, justify="left", anchor="w").pack(fill="x")

    # COL 1
    frame_charge = tk.LabelFrame(col1, text="Charge Request (0x333)", padx=8, pady=8)
    frame_charge.pack(fill="x", padx=0, pady=(0,8))
    text_charge = tk.StringVar(value="Waiting for 0x333...")
    tk.Label(frame_charge, textvariable=text_charge, justify="left", anchor="w").pack(fill="x")

    frame_odo = tk.LabelFrame(col1, text="Odometer (0x3F3)", padx=8, pady=8)
    frame_odo.pack(fill="x", padx=0, pady=(0,8))
    text_odo = tk.StringVar(value="Waiting for 0x3F3...")
    tk.Label(frame_odo, textvariable=text_odo, justify="left", anchor="w").pack(fill="x")

    frame_gps = tk.LabelFrame(col1, text="GPS Lat/Long (0x04F)", padx=8, pady=8)
    frame_gps.pack(fill="x", padx=0, pady=(0,0))
    text_gps = tk.StringVar(value="Waiting for 0x04F...")
    tk.Label(frame_gps, textvariable=text_gps, justify="left", anchor="w").pack(fill="x")

    frame_time = tk.LabelFrame(col1, text="System Time UTC (0x318)", padx=8, pady=8)
    frame_time.pack(fill="x", padx=0, pady=(8,0))
    text_time = tk.StringVar(value="Waiting for 0x318...")
    tk.Label(frame_time, textvariable=text_time, justify="left", anchor="w").pack(fill="x")

    frame_vin = tk.LabelFrame(col1, text="VIN (0x405)", padx=8, pady=8)
    frame_vin.pack(fill="x", padx=0, pady=(8,0))
    text_vin = tk.StringVar(value="Waiting for 0x405...")
    tk.Label(frame_vin, textvariable=text_vin, justify="left", anchor="w").pack(fill="x")

    # ---------- BOTTOM ROW: Unknown messages ----------
    bottom_row = tk.Frame(root)
    bottom_row.grid(row=1, column=0, columnspan=4, sticky="nsew", pady=(8,0))
    bottom_row.columnconfigure(0, weight=1)
    bottom_row.columnconfigure(1, weight=1)

    frame_unk_vehicle = tk.LabelFrame(bottom_row, text="Unknown (Vehicle Bus) — newest at right", padx=8, pady=8)
    frame_unk_vehicle.grid(row=0, column=0, sticky="nsew", padx=(0,8))
    frame_unk_chassis = tk.LabelFrame(bottom_row, text="Unknown (Chassis Bus) — newest at right", padx=8, pady=8)
    frame_unk_chassis.grid(row=0, column=1, sticky="nsew")

    unknown_vehicle_var = tk.StringVar(value="(none)")
    unknown_chassis_var = tk.StringVar(value="(none)")
    mono = ("Courier New", 9)
    tk.Label(frame_unk_vehicle, textvariable=unknown_vehicle_var, justify="left", anchor="nw", font=mono).pack(fill="both", expand=True)
    tk.Label(frame_unk_chassis, textvariable=unknown_chassis_var, justify="left", anchor="nw", font=mono).pack(fill="both", expand=True)

    # ===== Helpers =====
    def read_bits_le(payload: bytes, start: int, length: int) -> int:
        packed = int.from_bytes(payload[:8], byteorder='little', signed=False)
        return (packed >> start) & ((1 << length) - 1)

    def read_bits_be_stream(payload: bytes, start: int, length: int) -> int:
        packed = int.from_bytes(payload[:8], byteorder='big', signed=False)
        shift = 64 - (start + length)
        return (packed >> shift) & ((1 << length) - 1)

    def to_i8(u8: int) -> int:
        return u8 - 256 if u8 >= 128 else u8

    def b2s(val) -> str:
        return "true" if bool(val) else "false"

    def sign_extend(value: int, bitlen: int) -> int:
        sign_bit = 1 << (bitlen - 1)
        return (value - (1 << bitlen)) if (value & sign_bit) else value

    def printable_ascii(bs: bytes | None) -> str:
        if not bs:
            return ""
        return "".join(chr(b) for b in bs if 32 <= b <= 126)

    # ===== Value maps for Car Config (same as before, omitted here for brevity in comments) =====
    VAL = {
        "GTW_activeHighBeam": {1:"ACTIVE", 0:"NOT_ACTIVE"},
        "GTW_airSuspension": {0:"NONE", 2:"TESLA_ADAPTIVE", 1:"TESLA_STANDARD"},
        "GTW_airbagCutoffSwitch": {0:"CUTOFF_SWITCH_DISABLED", 1:"CUTOFF_SWITCH_ENABLED"},
        "GTW_audioType": {0:"BASE", 2:"BASE_WITH_PREMIUM200", 1:"PREMIUM"},
        "GTW_autopilot": {4:"BASIC", 2:"ENHANCED", 1:"HIGHWAY", 0:"NONE", 3:"SELF_DRIVING"},
        "GTW_autopilotCameraType": {0:"RCCB_CAMERAS"},
        "GTW_auxParkLamps": {2:"EU", 0:"NA_BASE", 1:"NA_PREMIUM", 3:"NONE"},
        "GTW_bPillarNFCParam": {0:"MODEL_3", 1:"MODEL_Y"},
        "GTW_brakeHWType": {1:"BREMBO_LARGE_P42_BREMBO_44MOC", 3:"BREMBO_LARGE_P42_BREMBO_LARGE_44MOC", 2:"BREMBO_LARGE_P42_MANDO_43MOC", 0:"BREMBO_P42_MANDO_43MOC"},
        "GTW_brakeLineSwitchType": {0:"DI_VC_SHARED", 1:"VC_ONLY"},
        "GTW_cabinPTCHeaterType": {0:"BORGWARNER", 1:"NONE"},
        "GTW_chassisType": {2:"MODEL_3_CHASSIS", 0:"MODEL_S_CHASSIS", 1:"MODEL_X_CHASSIS", 3:"MODEL_Y_CHASSIS"},
        "GTW_compressorType": {2:"DENSO_41CC_11K", 1:"DENSO_41CC_8K", 0:"HANON_33CC"},
        "GTW_connectivityPackage": {0:"BASE", 1:"PREMIUM"},
        "GTW_coolantPumpType": {0:"DUAL", 1:"SINGLE_PUMP_BATT"},
        "GTW_dasHw": {3:"PARKER_PASCAL_2_5", 4:"TESLA_AP3"},
        "GTW_deliveryStatus": {1:"DELIVERED", 0:"NOT_DELIVERED"},
        "GTW_drivetrainType": {1:"AWD", 0:"RWD"},
        "GTW_eBuckConfig": {1:"DEV_BUCK", 0:"NONE"},
        "GTW_eCallEnabled": {0:"DISABLED", 1:"ENABLED_OHC_SOS", 2:"ENABLED_UI_SOS"},
        "GTW_efficiencyPackage": {0:"DEFAULT", 2:"M3_LR_2020", 3:"M3_LR_PERFORMANCE_2020", 1:"M3_SR_PLUS_2020"},
        "GTW_epasType": {0:"MANDO_VGR69_GEN3"},
        "GTW_espValveType": {0:"UNKNOWN", 1:"VALVE_TYPE_1", 2:"VALVE_TYPE_2"},
        "GTW_exteriorColor": {5:"DEEP_BLUE", 3:"MIDNIGHT_SILVER", 6:"PEARL_WHITE", 0:"RED_MULTICOAT", 2:"SILVER_METALLIC", 1:"SOLID_BLACK"},
        "GTW_frontFogLamps": {1:"INSTALLED", 0:"NOT_INSTALLED"},
        "GTW_frontSeatHeaters": {1:"KONGSBERG_LOW_POWER", 0:"NONE"},
        "GTW_frontSeatReclinerHardware": {3:"LEFT_RIGHT_SEAT_REDUCED_RANGE", 2:"LEFT_SEAT_REDUCED_RANGE", 1:"RIGHT_SEAT_REDUCED_RANGE", 0:"STANDARD_RANGE"},
        "GTW_frontSeatType": {0:"BASE_TESLA", 3:"PREMIUM_L_TESLA_R_YANFENG", 2:"PREMIUM_L_YANFENG_R_TESLA", 1:"PREMIUM_TESLA", 4:"PREMIUM_YANFENG"},
        "GTW_headlamps": {0:"BASE", 1:"PREMIUM"},
        "GTW_headlightLevelerType": {1:"GEN1", 0:"NONE"},
        "GTW_homelinkType": {1:"HOMELINK_V_OPT_2", 0:"NONE"},
        "GTW_hvacPanelVaneType": {1:"CONVERGENT_V1", 0:"PARALLEL_V1"},
        "GTW_immersiveAudio": {1:"BASE", 0:"DISABLED", 2:"PREMIUM"},
        "GTW_interiorLighting": {0:"BASE", 1:"PREMIUM", 2:"PREMIUM_NO_POCKET_LIGHT"},
        "GTW_intrusionSensorType": {0:"NOT_INSTALLED", 1:"VODAFONE"},
        "GTW_lumbarECUType": {1:"ALFMEIER", 0:"NONE"},
        "GTW_mapRegion": {4:"AU", 3:"CN", 1:"EU", 9:"HK", 5:"JP", 7:"KR", 8:"ME", 10:"MO", 2:"NONE", 6:"TW", 0:"US"},
        "GTW_memoryMirrors": {0:"NOT_INSTALLED", 1:"SMR"},
        "GTW_numberHVILNodes": {0:"HVIL_NODES_0", 1:"HVIL_NODES_1", 2:"HVIL_NODES_2", 3:"HVIL_NODES_3", 4:"HVIL_NODES_4", 5:"HVIL_NODES_5"},
        "GTW_packEnergy": {3:"PACK_100_KWH", 0:"PACK_50_KWH", 2:"PACK_62_KWH", 1:"PACK_74_KWH", 4:"PACK_75_KWH"},
        "GTW_passengerAirbagType": {2:"EUROW", 0:"FULL_SUPPRESSION", 1:"SAFETY_VENT"},
        "GTW_passengerOccupancySensorType": {0:"OCS", 1:"RESISTIVE_PAD"},
        "GTW_pedestrianWarningSound": {0:"NONE", 1:"SPEAKER"},
        "GTW_performancePackage": {0:"BASE", 3:"BASE_PLUS", 4:"BASE_PLUS_AWD", 2:"LUDICROUS", 1:"PERFORMANCE"},
        "GTW_plcSupportType": {2:"NATIVE_CHARGE_PORT", 0:"NONE", 1:"ONBOARD_ADAPTER"},
        "GTW_powerSteeringColumn": {0:"NOT_INSTALLED", 1:"TK"},
        "GTW_radarHeaterType": {1:"BECKER_THIN_3M", 0:"NONE"},
        "GTW_rearFogLamps": {1:"INSTALLED", 0:"NOT_INSTALLED"},
        "GTW_rearGlassType": {0:"NX", 1:"TSA5_NOPET"},
        "GTW_rearLightType": {1:"EU_CN", 2:"GLOBAL", 0:"NA"},
        "GTW_rearSeatHeaters": {1:"KONGSBERG_LOW_POWER", 0:"NONE"},
        "GTW_refrigerantType": {0:"Default", 2:"R1234YF", 1:"R134A"},
        "GTW_restraintsHardwareType": {22:"EUROW_ECALL_M3", 33:"EUROW_ECALL_MY", 23:"EUROW_NO_ECALL_M3", 34:"EUROW_NO_ECALL_MY", 21:"NA_M3", 32:"NA_MY", 31:"NA_MY_OLD"},
        "GTW_rightHandDrive": {0:"LEFT", 1:"RIGHT"},
        "GTW_roofGlassType": {0:"TSA3_PET", 1:"TSA5_NOPET"},
        "GTW_roofType": {1:"FIXED_GLASS", 0:"METAL", 2:"PANORAMIC"},
        "GTW_softRange": {1:"RANGE_220_MILES", 2:"RANGE_93_MILES", 0:"STANDARD"},
        "GTW_spoilerType": {0:"NOT_INSTALLED", 1:"PASSIVE"},
        "GTW_steeringColumnMotorType": {0:"BOSCH", 1:"JE"},
        "GTW_steeringColumnUJointType": {0:"B_SAMPLE_PHASING", 1:"C_SAMPLE_PHASING"},
        "GTW_superchargingAccess": {1:"ALLOWED", 0:"NOT_ALLOWED", 2:"PAY_AS_YOU_GO"},
        "GTW_tireType": {4:"CONTI_ALL_SEASON_19", 17:"GOODYEAR_ALL_SEASON_20", 3:"HANKOOK_SUMMER_19", 1:"MICHELIN_ALL_SEASON_18",
                         19:"MICHELIN_ALL_SEASON_21", 2:"MICHELIN_SUMMER_18", 5:"MICHELIN_SUMMER_20", 18:"PIRELLI_SUMMER_21", 0:"UNKNOWN"},
        "GTW_towPackage": {0:"NONE", 1:"TESLA_REV1"},
        "GTW_tpmsType": {0:"CONTI_2", 1:"TESLA_BLE"},
        "GTW_twelveVBatteryType": {0:"ATLASBX_B24_FLOODED", 1:"CLARIOS_B24_FLOODED"},
        "GTW_vdcType": {0:"BOSCH_VDC", 1:"TESLA_VDC"},
        "GTW_wheelType": {4:"GEMINI_19_SQUARE", 5:"GEMINI_19_STAGGERED", 0:"PINWHEEL_18", 18:"PINWHEEL_18_CAP_KIT", 1:"STILETTO_19", 2:"STILETTO_20",
                          14:"STILETTO_20_DARK_SQUARE", 3:"STILETTO_20_DARK_STAGGERED", 19:"ZEROG_20_GUNPOWDER", 17:"APOLLO_19_SILVER",
                          20:"APOLLO_19_SILVER_CAP_KIT", 15:"INDUCTION_20_BLACK", 16:"UBERTURBINE_21_BLACK"},
        "GTW_windshieldType": {1:"EASTMAN_ACOUSTIC", 0:"SEKISUI_ACOUSTIC"},
        "GTW_xcpESP": {0:"FALSE", 1:"TRUE"},
        "GTW_xcpIbst": {0:"FALSE", 1:"TRUE"},
    }
    def map_val(name, raw):
        m = VAL.get(name)
        return m.get(raw, raw) if m else raw

    # ===== Decoders =====
    def decode_ui_status_0x353(payload: bytes) -> dict:
        b = lambda s,l: read_bits_le(payload, s, l)
        pcb_raw = b(48,8); cpu_raw = b(56,8)
        return {
            "wifi_active": b2s(b(6,1)), "wifi_connected": b2s(b(7,1)),
            "bluetooth_active": b2s(b(2,1)), "audio_active": b2s(b(1,1)),
            "system_active": b2s(b(8,1)), "ready_for_drive": b2s(b(9,1)),
            "cell_active": b2s(b(3,1)), "cell_connected": b2s(b(10,1)),
            "cell_network_tech": b(19,4), "cell_signal_bars": b(42,3),
            "cell_receiver_db": b(24,8) - 128, "display_ready": b2s(b(4,1)),
            "display_on": b2s(b(5,1)), "gps_active": b2s(b(16,1)),
            "radio_active": b2s(b(18,1)), "vpn_active": b2s(b(11,1)),
            "cpu_temp_c": to_i8(cpu_raw) + 40, "pcb_temp_c": to_i8(pcb_raw) + 40,
            "touch_active": b2s(b(0,1)), "screenshot_active": b2s(b(17,1)),
            "autopilot_trial": b(12,2), "factory_reset_state": b(14,2),
            "false_touch_count": b(32,8), "development_car": b2s(b(40,1)),
            "camera_active": b2s(b(41,1)),
        }

    def decode_range_soc_0x33a(payload: bytes) -> dict:
        b = lambda s,l: read_bits_le(payload, s, l)
        return {
            "range_miles": b(0,10), "ideal_range_miles": b(16,10),
            "rated_wh_per_mile": b(32,10), "soc_percent": b(48,7),
            "u_soe_percent": b(56,7),
        }

    def decode_charge_request_0x333(payload: bytes) -> dict:
        b = lambda s,l: read_bits_le(payload, s, l)
        return {
            "open_charge_port_request": b2s(b(0,1)),
            "close_charge_port_request": b2s(b(1,1)),
            "charge_enable_request": b2s(b(2,1)),
            "brick_v_logging_request": b2s(b(3,1)),
            "brick_balancing_disabled": b2s(b(4,1)),
            "ac_current_limit_a": b(8,7),
            "termination_percent": b(16,10) * 0.1,
            "smart_ac_enabled": b2s(b(26,1)),
            "scheduled_departure_enabled": b2s(b(27,1)),
            "soc_snapshot_exp_weeks": b(28,4) + 2,
            "cp_inlet_heater_request": b2s(b(32,1)),
        }

    def decode_odometer_0x3f3(payload: bytes) -> dict:
        raw24 = int.from_bytes(payload[:3], byteorder='little', signed=False)
        return {"odometer_km": raw24 * 0.1}

    def decode_gps_latlong_0x04f(payload: bytes) -> dict:
        raw_lat = read_bits_be_stream(payload, 0, 28)
        lat_signed = sign_extend(raw_lat, 28)
        latitude_deg = lat_signed * 1e-6
        raw_lon = read_bits_be_stream(payload, 28, 28)
        lon_signed = sign_extend(raw_lon, 28)
        longitude_deg = lon_signed * 1e-6
        accuracy_raw = read_bits_be_stream(payload, 57, 7)
        accuracy_m = accuracy_raw * 0.2
        return {"latitude_deg": latitude_deg, "longitude_deg": longitude_deg, "accuracy_m": accuracy_m}

    def decode_utc_time_0x318(payload: bytes) -> dict:
        year   = payload[0]; month  = payload[1]; second = payload[2]
        hour   = payload[3]; day    = payload[4]; minute = payload[5]
        return {"hour":hour, "minute":minute, "second":second, "month":month, "day":day, "year":year}

    # ---- Car Config (0x7FF), multiplexed — decode one frame ----
    def decode_car_config_0x7ff(payload: bytes) -> dict:
        b = lambda s,l: read_bits_le(payload, s, l)
        mux = payload[0]
        out = {"mux": mux}

        if mux == 1:  # m1
            out.update({
                "Country": b(16,16),
                "DAS HW": map_val("GTW_dasHw", b(40,3)),
                "Delivery Status": map_val("GTW_deliveryStatus", b(8,1)),
                "Drivetrain": map_val("GTW_drivetrainType", b(10,1)),
                "EPAS Type": map_val("GTW_epasType", b(9,1)),
                "Headlamps": map_val("GTW_headlamps", b(14,2)),
                "Rear Light Type": map_val("GTW_rearLightType", b(12,2)),
                "Right-Hand Drive": map_val("GTW_rightHandDrive", b(11,1)),
                "Tire Type": map_val("GTW_tireType", b(32,5)),
                "Restraints HW": map_val("GTW_restraintsHardwareType", b(48,8)),
            })
        elif mux == 2:  # m2
            out.update({
                "Active High Beam": map_val("GTW_activeHighBeam", b(34,1)),
                "Airbag Cutoff Switch": map_val("GTW_airbagCutoffSwitch", b(35,1)),
                "Aux Park Lamps": map_val("GTW_auxParkLamps", b(26,2)),
                "B-Pillar NFC": map_val("GTW_bPillarNFCParam", b(56,1)),
                "Brake HW Type": map_val("GTW_brakeHWType", b(59,2)),
                "PTC Heater Type": map_val("GTW_cabinPTCHeaterType", b(31,1)),
                "Exterior Color": map_val("GTW_exteriorColor", b(48,3)),
                "Front Fog Lamps": map_val("GTW_frontFogLamps", b(20,1)),
                "Front Seat Heaters": map_val("GTW_frontSeatHeaters", b(9,1)),
                "Homelink": map_val("GTW_homelinkType", b(13,1)),
                "HVAC Panel Vane": map_val("GTW_hvacPanelVaneType", b(29,1)),
                "Interior Lighting": map_val("GTW_interiorLighting", b(57,2)),
                "Intrusion Sensor": map_val("GTW_intrusionSensorType", b(36,1)),
                "Lumbar ECU": map_val("GTW_lumbarECUType", b(23,1)),
                "Memory Mirrors": map_val("GTW_memoryMirrors", b(17,1)),
                "HVIL Nodes": map_val("GTW_numberHVILNodes", b(51,2)),
                "Pedestrian Warning Sound": map_val("GTW_pedestrianWarningSound", b(54,1)),
                "Power Steering Column": map_val("GTW_powerSteeringColumn", b(18,1)),
                "Rear Fog Lamps": map_val("GTW_rearFogLamps", b(39,1)),
                "Rear Glass Type": map_val("GTW_rearGlassType", b(38,1)),
                "Rear Seat Heaters": map_val("GTW_rearSeatHeaters", b(10,1)),
                "Roof Glass Type": map_val("GTW_roofGlassType", b(61,1)),
                "Roof Type": map_val("GTW_roofType", b(40,1)),
                "Spoiler": map_val("GTW_spoilerType", b(37,1)),
                "Steering Column U-Joint": map_val("GTW_steeringColumnUJointType", b(55,1)),
                "Supercharging Access": map_val("GTW_superchargingAccess", b(45,2)),
                "TPMS Type": map_val("GTW_tpmsType", b(11,1)),
                "VDC Type": map_val("GTW_vdcType", b(14,1)),
                "Windshield Type": map_val("GTW_windshieldType", b(33,1)),
                "XCP ESP": map_val("GTW_xcpESP", b(16,1)),
                "XCP IBST": map_val("GTW_xcpIbst", b(15,1)),
                "eBuck Config": map_val("GTW_eBuckConfig", b(32,1)),
                "Autopilot Package": map_val("GTW_autopilot", b(42,3)),
            })
        elif mux == 3:  # m3
            out.update({
                "Air Suspension": map_val("GTW_airSuspension", b(22,2)),
                "Audio Type": map_val("GTW_audioType", b(30,2)),
                "Autopilot Camera Type": map_val("GTW_autopilotCameraType", b(26,1)),
                "Connectivity Package": map_val("GTW_connectivityPackage", b(27,1)),
                "Coolant Pump Type": map_val("GTW_coolantPumpType", b(17,1)),
                "Chassis Type": map_val("GTW_chassisType", b(18,3)),
                "Front Seat Recliner HW": map_val("GTW_frontSeatReclinerHardware", b(37,2)),
                "Front Seat Type": map_val("GTW_frontSeatType", b(60,3)),
                "Headlight Leveler": map_val("GTW_headlightLevelerType", b(47,1)),
                "Immersive Audio": map_val("GTW_immersiveAudio", b(56,2)),
                "Map Region": map_val("GTW_mapRegion", b(8,4)),
                "Pack Energy": map_val("GTW_packEnergy", b(32,5)),
                "Passenger Occ Sensor": map_val("GTW_passengerOccupancySensorType", b(24,1)),
                "Performance Package": map_val("GTW_performancePackage", b(12,3)),
                "PLC Support Type": map_val("GTW_plcSupportType", b(28,2)),
                "Radar Heater Type": map_val("GTW_radarHeaterType", b(55,1)),
                "Refrigerant Type": map_val("GTW_refrigerantType", b(45,2)),
                "Soft Range": map_val("GTW_softRange", b(42,3)),
                "Tow Package": map_val("GTW_towPackage", b(15,1)),
                "12V Battery Type": map_val("GTW_twelveVBatteryType", b(63,1)),
                "Wheel Type": map_val("GTW_wheelType", b(48,7)),
            })
        elif mux >= 4:  # m4 or beyond
            out.update({
                "Birthday": b(8,32),
                "Compressor Type": map_val("GTW_compressorType", b(46,2)),
                "eCall Enabled": map_val("GTW_eCallEnabled", b(40,2)),
                "Efficiency Package": map_val("GTW_efficiencyPackage", b(48,3)),
                "Passenger Airbag Type": map_val("GTW_passengerAirbagType", b(43,2)),
                "Steering Column Motor Type": map_val("GTW_steeringColumnMotorType", b(52,1)),
            })
        return out

    # Cache last-seen fields per mux
    cfg_cache = {1:{}, 2:{}, 3:{}, 4:{}}

    # ===== SWC state and wiring =====
    swc = {"left_pressed":0,"right_pressed":0,"left_tilt":0,"right_tilt":0,"left_scroll":0,"right_scroll":0}

    def swc_scroll(side, delta):
        key = f"{side}_scroll"
        swc[key] = max(-32, min(31, swc[key] + delta))
    def swc_tilt(side, direction): swc[f"{side}_tilt"] = direction
    def on_click_press(side):      swc[f"{side}_pressed"] = 1
    def on_click_release(side):    swc[f"{side}_pressed"] = 0

    btns[("left","up")]["command"]    = lambda: swc_scroll("left", +1)
    btns[("left","down")]["command"]  = lambda: swc_scroll("left", -1)
    btns[("left","left")]["command"]  = lambda: swc_tilt("left", -1)
    btns[("left","right")]["command"] = lambda: swc_tilt("left", +1)
    btns[("left","click")].bind("<ButtonPress-1>",  lambda e: on_click_press("left"))
    btns[("left","click")].bind("<ButtonRelease-1>",lambda e: on_click_release("left"))

    btns[("right","up")]["command"]    = lambda: swc_scroll("right", +1)
    btns[("right","down")]["command"]  = lambda: swc_scroll("right", -1)
    btns[("right","left")]["command"]  = lambda: swc_tilt("right", -1)
    btns[("right","right")]["command"] = lambda: swc_tilt("right", +1)
    btns[("right","click")].bind("<ButtonPress-1>",  lambda e: on_click_press("right"))
    btns[("right","click")].bind("<ButtonRelease-1>",lambda e: on_click_release("right"))

    def set_bits_le(val, start, length, field):
        mask = ((1 << length) - 1) << start
        val &= ~mask
        val |= (field & ((1 << length) - 1)) << start
        return val

    def encode_signed_field(field_val, bitlen):
        if field_val < 0:
            field_val = (1 << bitlen) + field_val
        return field_val & ((1 << bitlen) - 1)

    def build_swc_frame_bytes():
        v = 0
        v = set_bits_le(v, 0, 2, 1)  # m1
        v = set_bits_le(v, 5, 2, 1 if swc["left_pressed"] else 0)
        if swc["left_tilt"] > 0:      v = set_bits_le(v, 3, 2, 1)
        elif swc["left_tilt"] < 0:    v = set_bits_le(v, 14, 2, 1)
        v = set_bits_le(v, 16, 6, encode_signed_field(swc["left_scroll"], 6))
        v = set_bits_le(v, 12, 2, 1 if swc["right_pressed"] else 0)
        if swc["right_tilt"] > 0:     v = set_bits_le(v, 10, 2, 1)
        elif swc["right_tilt"] < 0:   v = set_bits_le(v, 8, 2, 1)
        v = set_bits_le(v, 24, 6, encode_signed_field(swc["right_scroll"], 6))
        return v.to_bytes(8, byteorder='little', signed=False)

    def swc_build_and_store():
        global swc_frame_bytes
        try:
            swc_frame_bytes = build_swc_frame_bytes()
        except Exception:
            pass
        swc["left_tilt"] = 0; swc["right_tilt"] = 0
        swc["left_scroll"] = 0; swc["right_scroll"] = 0
        window.after(50, swc_build_and_store)  # ~20 Hz
    swc_build_and_store()

    # ---- Formatting helpers ----
    def fmt_can_line(msg):
        try:
            did = f"0x{msg.arbitration_id:03X}"
            data_hex = " ".join(f"{b:02X}" for b in msg.data)
            return f"{did}: {data_hex}"
        except Exception:
            return "(malformed message)"

    def colize(lines, cols=3, col_width=30):
        if not lines: return "(none)"
        rows = []
        for i in range(0, len(lines), cols):
            chunk = lines[i:i+cols]
            while len(chunk) < cols: chunk.append("")
            row = "".join(f"{s:<{col_width}}" for s in chunk)
            rows.append(row.rstrip())
        return "\n".join(rows)

    # ---- Refresh loop ----
    def refresh_all():
        # 0x353
        try:
            p353 = bytes(ui_status_message.data)
            if len(p353) >= 8:
                d = decode_ui_status_0x353(p353)
                text_ui.set("\n".join([
                    f"Wi-Fi Active: {d['wifi_active']}",
                    f"Wi-Fi Connected: {d['wifi_connected']}",
                    f"BT: {d['bluetooth_active']}",
                    f"Audio: {d['audio_active']}",
                    f"System Active: {d['system_active']}",
                    f"Ready For Drive: {d['ready_for_drive']}",
                    f"Cell Active: {d['cell_active']}",
                    f"Cell Connected: {d['cell_connected']}",
                    f"Cell Network Tech: {d['cell_network_tech']}",
                    f"Cell Signal Bars: {d['cell_signal_bars']}",
                    f"Cell Receiver (dB): {d['cell_receiver_db']}",
                    f"Display Ready: {d['display_ready']}",
                    f"Display On: {d['display_on']}",
                    f"GPS Active: {d['gps_active']}",
                    f"Radio Active: {d['radio_active']}",
                    f"VPN Active: {d['vpn_active']}",
                    f"CPU Temp: {d['cpu_temp_c']:.0f}°C",
                    f"PCB Temp: {d['pcb_temp_c']:.0f}°C",
                    f"Touch Active: {d['touch_active']}",
                    f"Screenshot Active: {d['screenshot_active']}",
                    f"Autopilot Trial: {d['autopilot_trial']}",
                    f"Factory Reset State: {d['factory_reset_state']}",
                    f"False Touch Count: {d['false_touch_count']}",
                    f"Development Car: {d['development_car']}",
                    f"Camera Active: {d['camera_active']}",
                ]))
        except Exception as e:
            text_ui.set(f"Error decoding 0x353: {e}")

        # 0x33A
        try:
            p33a = bytes(ui_range_message.data)
            if len(p33a) >= 8:
                r = decode_range_soc_0x33a(p33a)
                text_range.set("\n".join([
                    f"Range (mi): {r['range_miles']}",
                    f"Ideal Range (mi): {r['ideal_range_miles']}",
                    f"SOC (%): {r['soc_percent']}",
                    f"uSOE (%): {r['u_soe_percent']}",
                    f"Rated (Wh/mi): {r['rated_wh_per_mile']}",
                ]))
        except Exception as e:
            text_range.set(f"Error decoding 0x33A: {e}")

        # 0x333
        try:
            p333 = bytes(ui_charge_message.data)
            if len(p333) >= 5:
                c = decode_charge_request_0x333(p333)
                text_charge.set("\n".join([
                    f"Open Charge Port Req: {c['open_charge_port_request']}",
                    f"Close Charge Port Req: {c['close_charge_port_request']}",
                    f"Charge Enable Req: {c['charge_enable_request']}",
                    f"CP Inlet Heater Req: {c['cp_inlet_heater_request']}",
                    f"AC Current Limit (A): {c['ac_current_limit_a']}",
                    f"Termination at (%): {c['termination_percent']:.1f}",
                    f"Smart AC Enabled: {c['smart_ac_enabled']}",
                    f"Scheduled Departure Enabled: {c['scheduled_departure_enabled']}",
                    f"SoC Snapshot Exp (weeks): {c['soc_snapshot_exp_weeks']}",
                    f"Brick V Logging Req: {c['brick_v_logging_request']}",
                    f"Brick Balancing Disabled: {c['brick_balancing_disabled']}",
                ]))
        except Exception as e:
            text_charge.set(f"Error decoding 0x333: {e}")

        # 0x3F3
        try:
            p3f3 = bytes(ui_odo_message.data)
            if len(p3f3) >= 3:
                o = decode_odometer_0x3f3(p3f3)
                text_odo.set(f"Odometer (km): {o['odometer_km']:.1f}")
        except Exception as e:
            text_odo.set(f"Error decoding 0x3F3: {e}")

        # 0x04F GPS
        try:
            p04f = bytes(gps_latlong_message.data)
            if len(p04f) >= 8:
                g = decode_gps_latlong_0x04f(p04f)
                text_gps.set("\n".join([
                    f"Latitude (deg): {g['latitude_deg']:.6f}",
                    f"Longitude (deg): {g['longitude_deg']:.6f}",
                    f"Accuracy (m): {g['accuracy_m']:.1f}",
                ]))
        except Exception as e:
            text_gps.set(f"Error decoding 0x04F: {e}")

        # 0x318 UTC time
        try:
            p318 = bytes(utc_time_message.data)
            if len(p318) >= 6:
                t = decode_utc_time_0x318(p318)
                text_time.set(f"{t['hour']:02d}:{t['minute']:02d}:{t['second']:02d} {t['month']:02d}/{t['day']:02d}/{t['year'] % 100:02d}")
        except Exception as e:
            text_time.set(f"Error decoding 0x318: {e}")

        # 0x405 VIN
        try:
            vina = vin_segments.get(16); vinc = vin_segments.get(18); vinb = vin_segments.get(17)
            a = printable_ascii(vina); c = printable_ascii(vinc); b = printable_ascii(vinb)
            first3 = a[-3:] if len(a) >= 3 else a
            mid7   = c[:7]  if len(c) >= 7 else c
            last7  = b[:7]  if len(b) >= 7 else b
            full_vin = (first3 + mid7 + last7)[:17]
            text_vin.set(f"VIN: {full_vin}" if len(full_vin) == 17 else "VIN: (waiting for all parts)")
        except Exception as e:
            text_vin.set(f"Error decoding 0x405: {e}")

        # 0x7FF Car Config — update cache, then render split boxes
        try:
            cfg_msg = globals().get("ui_car_config_message", None)
            if cfg_msg:
                p7ff = bytes(cfg_msg.data)
                if len(p7ff) >= 8:
                    d = decode_car_config_0x7ff(p7ff)
                    mux = d.pop("mux", None)
                    if mux in cfg_cache:
                        cfg_cache[mux].update(d)
                    elif mux and mux >= 4:
                        cfg_cache.setdefault(4, {}).update(d)

            # Box A = m1 + m2
            lines_a = []
            for mux in (1,2):
                sect = cfg_cache.get(mux, {})
                if sect:
                    lines_a.append(f"— m{mux} —")
                    for k in sorted(sect.keys()):
                        lines_a.append(f"{k}: {sect[k]}")
            text_cfg_a.set("\n".join(lines_a) if lines_a else "Waiting for 0x7FF (m1–m2)...")

            # Box B = m3 + rest (m4+)
            lines_b = []
            for mux in [3] + [k for k in sorted(cfg_cache.keys()) if k not in (1,2,3)]:
                sect = cfg_cache.get(mux, {})
                if sect:
                    lines_b.append(f"— m{mux} —")
                    for k in sorted(sect.keys()):
                        lines_b.append(f"{k}: {sect[k]}")
            text_cfg_b.set("\n".join(lines_b) if lines_b else "Waiting for 0x7FF (m3+)...")
        except Exception as e:
            text_cfg_a.set(f"Error decoding 0x7FF: {e}")
            text_cfg_b.set(f"Error decoding 0x7FF: {e}")

        # Unknown Vehicle — newest at right in 3 columns
        try:
            if unknown_vehicle_msgs:
                chronological = [fmt_can_line(m) for m in list(unknown_vehicle_msgs)]  # oldest->newest
                chronological = chronological[-36:]
                unknown_vehicle_var.set(colize(chronological, cols=3, col_width=30))
            else:
                unknown_vehicle_var.set("(none)")
        except Exception as e:
            unknown_vehicle_var.set(f"Error: {e}")

        # Unknown Chassis — newest at right in 3 columns
        try:
            if unknown_chassis_msgs:
                chronological = [fmt_can_line(m) for m in list(unknown_chassis_msgs)]
                chronological = chronological[-36:]
                unknown_chassis_var.set(colize(chronological, cols=3, col_width=30))
            else:
                unknown_chassis_var.set("(none)")
        except Exception as e:
            unknown_chassis_var.set(f"Error: {e}")

        window.after(100, refresh_all)

    # Kick off the first refresh tick
    window.after(0, refresh_all)
    window.mainloop()

# Start the GUI thread
gui_thread = threading.Thread(target=gui_thread)
gui_thread.start()

#Recieve from all the buses
def cha_receive():
    while True:
        chassis_message = chassis_bus.recv()
        if not chassis_message:
            continue

        if chassis_message.arbitration_id == 0x3F3:
            global ui_odo_message
            ui_odo_message = chassis_message
        elif chassis_message.arbitration_id == 0x04F:
            global gps_latlong_message
            gps_latlong_message = chassis_message
        else:
            # stash unknown chassis messages
            unknown_chassis_msgs.append(chassis_message)

cha_receive = threading.Thread(target=cha_receive)
cha_receive.start()


def veh_receive():
    while True:
        vehicle_message = vehicle_bus.recv()
        if not vehicle_message:
            continue

        if vehicle_message.arbitration_id == 0x353:
            global ui_status_message
            ui_status_message = vehicle_message
        elif vehicle_message.arbitration_id == 0x33A:
            global ui_range_message
            ui_range_message = vehicle_message
        elif vehicle_message.arbitration_id == 0x333:
            global ui_charge_message
            ui_charge_message = vehicle_message
        elif vehicle_message.arbitration_id == 0x318:
            global utc_time_message
            utc_time_message = vehicle_message
        elif vehicle_message.arbitration_id == 0x405:
            # Multiplexed VIN message
            global vin_mux_last, vin_segments, vin_string
            data = vehicle_message.data
            if not data or len(data) < 2:
                continue
            mux = data[0]
            vin_mux_last = mux
            seg = bytes(data[1:8])  # 56-bit ASCII lives here

            # Map mux -> segment
            # m16 = VINA405 ("0000" then first 3 VIN chars)
            # m17 = VINB405 (last 7 of VIN)
            # m18 = VINC405 (middle 7 of VIN)
            if mux in (16, 17, 18):
                vin_segments[mux] = seg

                # Try to reconstruct full VIN when all three parts are present
                a = vin_segments.get(16)
                b = vin_segments.get(17)
                c = vin_segments.get(18)

                def safe_ascii(bs):
                    try:
                        return bs.decode('ascii', errors='ignore')
                    except Exception:
                        return ""

                if a and b and c:
                    # VINA: "0000" + first 3 chars -> take the LAST 3 bytes as first 3 VIN chars
                    first3 = safe_ascii(a[-3:])
                    mid7   = safe_ascii(c)       # VINC
                    last7  = safe_ascii(b)       # VINB
                    candidate = (first3 + mid7 + last7)
                    # Keep only first 17 characters just in case anything extra sneaks in
                    vin_string = candidate[:17]
        elif vehicle_message.arbitration_id == 0x7FF:
            global ui_car_config_message
            ui_car_config_message = vehicle_message

        else:
            # stash unknown vehicle messages
            unknown_vehicle_msgs.append(vehicle_message)
        # print("VEH:", vehicle_message)

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

            can.Message(arbitration_id=0x221, data=[ # Vehicle Controller Front LV Power State 64 turns on parkling brake light
                ignition*96,random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255)], is_extended_id=False), 

            can.Message(arbitration_id=0x228, data=[ # EPB Right Status
                0x03,0x16,0x00,0x0C,0x51,0x80,((counter_4bit_100ms & 0xF) << 4) | 0x1,(((counter_4bit_100ms + 4) & 0xF) << 4)], is_extended_id=False), 

            can.Message(arbitration_id=0x288, data=[ # EPB Left Status
                0x03,0x16,0x00,0xCC,0x50,0x80,((counter_4bit_100ms & 0xF) << 4) | 0x1,(((counter_4bit_100ms + 4) & 0xF) << 4)], is_extended_id=False), 

            can.Message(arbitration_id=0x212, data=[ # BMS Status - Makes MCU show charging - 2nd byte: 8 - Ready to charge, 16 - Starting to charge, 24 - chargine complete, 32 - green charging bar, 40 - charging stopped, 48 - no message
                0xb9,4,0x93,0x0c,0x01,0xff,0x3f,0x01], is_extended_id=False), 

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
                arbitration_id=0x145,data=[0,0,0,0,0,0,0,0],is_extended_id=False # ESP Status (INCOMPLETE)
            ),
            #can.Message(
            #    arbitration_id=0x370,data=[0,0,0,0,0,0,0,0],is_extended_id=False 
            #),
            
            #can.Message(arbitration_id=id_counter, data=[
            #    random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255),random.randint(0,255)], is_extended_id=False),
        ]
        
        if swc_frame_bytes:
            messages_100ms_vehicle.append(
                can.Message(arbitration_id=0x3C2, data=list(swc_frame_bytes), is_extended_id=False)
            )
        
        #Update checksums and counters here
        counter_8bit_100ms = (counter_8bit_100ms + 1) % 256
        counter_4bit_100ms = (counter_4bit_100ms + 1) % 15

        # Send Messages
        for message in messages_100ms_vehicle:
            try:
                vehicle_bus.send(message)
            except can.CanError as e:
                print("VEH send error:", e)
            wpt.sleep(0.002)  # a hair more breathing room for SLCAN

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

