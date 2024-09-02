
import math
from unavlib.control import UAVControl
from unavlib.control import geospatial
from pymavlink import mavutil
import time
import inspect 
import traceback
import asyncio

# https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py

def dict_index(d, target_value):
    return [key for key, value in d.items() if value == target_value]

def dict_reverse(d):
    return {v: i for i,v in d.items()}


class Telemetry:
    def __init__(self, mavctl, master, start_time):
        self.mavconn = master
        self.inavctl = None
        self.mavctl = mavctl
        self.start_time = start_time

        # Data stream rates (in Hz)
        self.stream_rates = {
            'HEARTBEAT': 2,
            'ATTITUDE': 10,
            'MISSION_CURRENT': 1,
            'GPS_RAW_INT': 5,
            'GLOBAL_POSITION_INT': 5,
            'GLOBAL_POSITION_ORIGIN': 2,
            'RC_CHANNELS': 10,
            'BATTERY_STATUS': 2,
            'SCALED_PRESSURE': 0.2,
            'VFR_HUD': 3,
            'SYS_STATUS': 1,
            #'EXTRA1': 3,
            #'EXTRA2': 2,
            #'EXTRA3': 1,
        }

        # Track last sent times for different messages
        self.last_gcs_heartbeat = 0
        self.last_msg_time = {key: 0 for key in self.stream_rates}

        # Define the functions associated with each telemetry stream
        self.stream_functions = {
            'HEARTBEAT': self.send_heartbeat,
            'ATTITUDE': self.send_attitude,
            'MISSION_CURRENT': self.send_mission_current,
            'GPS_RAW_INT': self.send_gps_raw_int,
            'GLOBAL_POSITION_INT': self.send_global_position_int,
            'GLOBAL_POSITION_ORIGIN': self.send_global_position_origin,
            'RC_CHANNELS': self.send_rc_channels,
            'BATTERY_STATUS': self.send_battery_status,
            'SCALED_PRESSURE': self.send_scaled_pressure,
            'VFR_HUD': self.send_vfr_hud,
            'SYS_STATUS': self.send_sys_status,
        }

    def send(self, current_time):
        """Iterate over the telemetry streams and send messages if the interval has passed."""
        for stream_name, rate in self.stream_rates.items():
            if current_time - self.last_msg_time[stream_name] >= (1.0 / rate):
                self.stream_functions[stream_name]()
                self.last_msg_time[stream_name] = current_time

    def send_heartbeat(self):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.mavctl.armed:
            base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        self.mavconn.mav.heartbeat_send(
            type=self.mavctl.type,
            autopilot=self.mavctl.autopilot,
            base_mode=base_mode,
            custom_mode=self.mavctl.mode_id,
            system_status=self.mavctl.system_status
        )

    def send_attitude(self):
        self.mavconn.mav.attitude_send(
            time_boot_ms=int((time.time() - self.start_time) * 1000),
            roll=self.mavctl.attitude['roll'],
            pitch=self.mavctl.attitude['pitch'],
            yaw=self.mavctl.attitude['yaw'],
            rollspeed=self.mavctl.attitude['rollspeed'],
            pitchspeed=self.mavctl.attitude['pitchspeed'],
            yawspeed=self.mavctl.attitude['yawspeed']
        )

    def send_gps_raw_int(self):
        self.mavconn.mav.gps_raw_int_send(
            time_usec=int(time.time() * 1000000),
            fix_type=3,
            lat=int(self.mavctl.position.lat * 1e7),
            lon=int(self.mavctl.position.lon * 1e7),
            alt=int(self.mavctl.position.alt * 1e3),
            eph=100,
            epv=100,
            vel=0,
            cog=0,
            satellites_visible=10
        )

    def send_mission_current(self):
        self.mavconn.mav.mission_current_send(
            seq=self.mavctl.current_mission_seq,
        )

    def send_battery_status(self):
        self.mavconn.mav.battery_status_send(
            id=0,
            battery_function=mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
            type=mavutil.mavlink.MAV_BATTERY_TYPE_LIPO, # can change
            temperature=int(self.mavctl.battery.get('temperature', 25.0) * 100),
            voltages=[int(self.mavctl.battery.get('voltage', 11.1) * 1000)] * 10,
            current_battery=int(self.mavctl.battery.get('current', 1.0) * 100),
            current_consumed=self.mavctl.battery.get('mahdrawn', 100),
            energy_consumed=self.mavctl.battery.get('mwhdrawn', 100),
            battery_remaining=self.mavctl.battery.get('remaining', 100)
        )

    def send_global_position_int(self):
        self.mavconn.mav.global_position_int_send(
            time_boot_ms=int((time.time() - self.start_time) * 1000),
            lat=int(self.mavctl.position.lat * 1e7),
            lon=int(self.mavctl.position.lon * 1e7),
            alt=int(self.mavctl.position.alt * 1e3),
            relative_alt=int(self.mavctl.position.alt * 1e3),
            vx=0, vy=0, vz=0,
            hdg=0
        )

    def send_global_position_origin(self):
        self.mavconn.mav.set_gps_global_origin_send(
            target_system=self.mavconn.target_system,
            latitude=int(self.mavctl.global_position_origin.lat * 1e7),
            longitude=int(self.mavctl.global_position_origin.lon * 1e7),
            altitude=int(self.mavctl.global_position_origin.alt * 1e3)
        )

    def send_rc_channels(self):
        self.mavconn.mav.rc_channels_send(
            time_boot_ms=int((time.time() - self.start_time) * 1000),
            chan1_raw=self.mavctl.rc_channels[0],
            chan2_raw=self.mavctl.rc_channels[1],
            chan3_raw=self.mavctl.rc_channels[2],
            chan4_raw=self.mavctl.rc_channels[3],
            chan5_raw=self.mavctl.rc_channels[4],
            chan6_raw=self.mavctl.rc_channels[5],
            chan7_raw=self.mavctl.rc_channels[6],
            chan8_raw=self.mavctl.rc_channels[7],
            chan9_raw=self.mavctl.rc_channels[8],
            chan10_raw=self.mavctl.rc_channels[9],
            chan11_raw=self.mavctl.rc_channels[10],
            chan12_raw=self.mavctl.rc_channels[11],
            chan13_raw=self.mavctl.rc_channels[12],
            chan14_raw=self.mavctl.rc_channels[13],
            chan15_raw=self.mavctl.rc_channels[14],
            chan16_raw=self.mavctl.rc_channels[15],
            chan17_raw=0, chan18_raw=0,
            chancount=16,
            rssi=self.mavctl.rssi
        )

    def send_scaled_pressure(self):
        self.mavconn.mav.scaled_pressure_send(
            time_boot_ms=int((time.time() - self.start_time) * 1000),
            press_abs=self.mavctl.scaled_pressure.get('press_abs', 1013.25),
            press_diff=self.mavctl.scaled_pressure.get('press_diff', 0.0),
            temperature=int(self.mavctl.scaled_pressure.get('temperature', 20.0) * 100)
        )

    def send_sys_status(self):
        self.mavconn.mav.sys_status_send(
            onboard_control_sensors_present=0,
            onboard_control_sensors_enabled=0,
            onboard_control_sensors_health=0,
            load=500,
            voltage_battery=int(self.mavctl.sys_status.get('voltage_battery', 11.1) * 1000),
            current_battery=int(self.mavctl.sys_status.get('current_battery', 1.0) * 100),
            battery_remaining=self.mavctl.sys_status.get('battery_remaining', 100),
            drop_rate_comm=0,
            errors_comm=0,
            errors_count1=self.mavctl.sys_status.get('errors', 0),
            errors_count2=0,
            errors_count3=0,
            errors_count4=0
        )

    def send_vfr_hud(self):
        self.mavconn.mav.vfr_hud_send(
            airspeed=0,
            groundspeed=0,
            heading=0,
            throttle=0,
            alt=self.mavctl.position.alt,
            climb=0
        )

class MavlinkControl:
    def __init__(self, inav_conn, platform_type=mavutil.mavlink.MAV_TYPE_QUADROTOR, connection_string='udpout:localhost:14550', use_mavlink2=True):
        self.mavconn = mavutil.mavlink_connection(
            connection_string,
            source_system=1,
            source_component=1,
            dialect="ardupilotmega",
            force_mavlink2=use_mavlink2
        )
        self.running = True
        self.start_time = time.time()
        self.autopilot = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
        self.type = platform_type
        self.inavctl = inav_conn

        self.start_pos = geospatial.GPSposition(45.0, -73.0, 0)
        self.attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'rollspeed': 0.0, 'pitchspeed': 0.0, 'yawspeed': 0.0}
        self.position = self.start_pos
        self.gps_speed = [0.0, 0.0, 0.0]
        self.battery = {'voltage': 11.1, 'current': 1.0, 'remaining': 100}
        self.armed = False
        self.mode = 'STABILIZE'
        self.modes = mavutil.AP_MAV_TYPE_MODE_MAP[self.type]
        self.modes_index = dict_reverse(self.modes)
        self.mode_id = self.modes_index[self.mode]

        self.inav_last_modes_cmd = []

        self.system_status = mavutil.mavlink.MAV_STATE_STANDBY
        self.flying = False
        self.battery = {'voltage': 11.1, 'current': 1.0, 'remaining': 100, 'temperature': 0.0, 'mahdrawn': 0}
        self.sys_status = {'voltage_battery': 11.1, 'current_battery': 1.0, 'battery_remaining': 100, 'errors': 0}
        self.global_position_origin = self.start_pos
        self.scaled_pressure = {'press_abs': 1013.25, 'press_diff': 0.0, 'temperature': 20.0}
        self.rc_channels = [0] * 16
        self.rssi = 255

        # Mission storage and state tracking
        self.destination = geospatial.GPSposition(0.0, 0.0, 0)
        self.mission_download = False
        self.mission_items = []
        self.mission_id = 0
        self.current_mission_seq = 0
        self.mission_state = mavutil.mavlink.MISSION_STATE_NO_MISSION

        # Initialize telemetry
        self.telemetry = Telemetry(self, self.mavconn, self.start_time)

    def update(self):
        if len(self.mission_items) == 0:
            self.mission_state = mavutil.mavlink.MISSION_STATE_NO_MISSION

        elif self.mode_id == 3 and self.mission_state in (2,4):
            self.mission_state = mavutil.mavlink.MISSION_STATE_ACTIVE
            self.current_mission_seq = 0

        elif self.mode_id != 3 and self.mission_state == 3:
            self.mission_state = mavutil.mavlink.MISSION_STATE_PAUSED

    def translate_command(self, command_id):
        if command_id in mavutil.mavlink.enums['MAV_CMD']:
            return mavutil.mavlink.enums['MAV_CMD'][command_id].name
        return f"UNKNOWN_COMMAND_{command_id}"


    def handle_param_request_list(self, msg):
        """Handle PARAM_REQUEST_LIST by responding with the minimal set of parameters."""
        # Respond with each parameter
        for i, param in enumerate(self.parameters):
            self.mavconn.mav.param_value_send(
                param_id=param['param_id'],
                param_value=param['param_value'],
                param_type=param['param_type'],
                param_count=self.param_count,
                param_index=i
            )
        print("Handled PARAM_REQUEST_LIST: Sent minimal set of parameters.")

    def handle_param_set(self, msg):
        """Handle PARAM_SET by updating the parameter value."""
        param_id = msg.param_id.rstrip(b'\x00').decode('utf-8')
        for param in self.parameters:
            if param['param_id'].decode('utf-8') == param_id:
                param['param_value'] = msg.param_value
                print(f"Updated parameter {param_id} to {msg.param_value}")
                self.mavconn.mav.param_value_send(
                    param_id=param['param_id'],
                    param_value=param['param_value'],
                    param_type=param['param_type'],
                    param_count=self.param_count,
                    param_index=self.parameters.index(param)
                )
                break

    def handle_mission_request_list(self, msg):
        """Handle MISSION_REQUEST_LIST by sending stored mission items."""
        for seq in range(len(self.mission_items)):
            self.mavconn.mav.mission_request_int_send(
                target_system=self.mavconn.target_system,
                target_component=self.mavconn.target_component,
                seq=seq
            )
            print(f"Requested to send mission item {seq} to GCS.")
            time.sleep(0.1)  # Small delay to prevent message overload

    def handle_mission_request_int(self, msg):
        """Handle MISSION_REQUEST_INT by sending the specific mission item requested by the GCS."""
        seq = msg.seq
        if seq < len(self.mission_items):
            mission_item = self.mission_items[seq]
            self.mavconn.mav.mission_item_int_send(
                target_system=self.mavconn.target_system,
                target_component=self.mavconn.target_component,
                seq=mission_item.seq,
                frame=mission_item.frame,
                command=mission_item.command,
                current=mission_item.current,
                autocontinue=mission_item.autocontinue,
                param1=mission_item.param1,
                param2=mission_item.param2,
                param3=mission_item.param3,
                param4=mission_item.param4,
                x=mission_item.x,
                y=mission_item.y,
                z=mission_item.z,
                mission_type=mission_item.mission_type
            )
            print(f"Sent mission item {seq} to GCS.")

            if seq == len(self.mission_items) - 1:
                # All mission items have been sent, send MISSION_ACK
                self.mavconn.mav.mission_ack_send(
                    target_system=self.mavconn.target_system,
                    target_component=self.mavconn.target_component,
                    type=mavutil.mavlink.MAV_MISSION_ACCEPTED
                )
                print("All mission items sent. Sent MISSION_ACK to GCS.")

        else:
            print(f"Invalid mission item sequence: {seq}")



    def arm(self, arm_disarm):
        # arm checks here
        self.inavctl.set_mode(self.inavctl.inav.modesID.ARM, on=arm_disarm)
        for i in range(3):
            self.armed = self.inavctl.armed
            if self.armed:
                break
            time.sleep(1)

    def set_mode(self, mode_id):
        if mode_id == self.modes_index["AUTO"] and len(self.mission_items)==0:
            return False

        self.mode_id = mode_id
        self.mode = self.modes.get(mode_id, "UNKNOWN")
        self.inav_mode = self.ap_to_inav_modes(mode_id)
        
        for inav_mode_id in self.inav_last_modes_cmd:
            self.inavctl.set_mode(inav_mode_id, on=False)

        for inav_mode_id in self.inav_mode:
            self.inavctl.set_mode(inav_mode_id, on=True)

        self.inav_last_modes_cmd = self.inav_mode
        #print('Active modes:')
        #for i in self.inavctl.get_active_modes():
        #    print('\t',i, self.inavctl.inav.modesID.get(i))
        print(f"Flight mode changed to: {self.mode} ({[self.inavctl.inav.modesID.get(i) for i in self.inav_mode]})")
        #print('inavctl rc:',self.inavctl.channels)
        #print('real rc:',self.inavctl.board.RC['channels'])
        return True


    def ap_to_inav_modes(self, apmode):
        match apmode:
            case 0: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # STABILIZE
                    return [ self.inavctl.inav.modesID.ANGLE ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # MANUAL
                    return [ self.inavctl.inav.modesID.MANUAL ]

            case 1: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # ACRO
                    return [  ]  # INAV HAS NO MODE ID DEFINED FOR ACRO AND I'M PISSED
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # CIRCLE
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]

            case 2: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # ALT_HOLD
                    return [ self.inavctl.inav.modesID.NAV_ALTHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # STABILIZE
                    return [ self.inavctl.inav.modesID.ANGLE ]

            case 3: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # AUTO
                    return [ self.inavctl.inav.modesID.NAV_WP ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # TRAINING
                    return [ self.inavctl.inav.modesID.MANUAL ]

            case 4:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # GUIDED
                    return [ self.inavctl.inav.modesID.GCS_NAV, self.inavctl.inav.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # ACRO
                    return [  ]

            case 5: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # LOITER
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # FBWA
                    return [ self.inavctl.inav.modesID.NAV_COURSE_HOLD ]

            case 6:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # RTL
                    return [ self.inavctl.inav.modesID.RTH ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # FBWB
                    return [ self.inavctl.inav.modesID.NAV_COURSE_HOLD , self.inavctl.inav.modesID.ANG_HOLD]

            case 7: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # CIRCLE
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # CRUISE
                    return [ self.inavctl.inav.modesID.NAV_CRUISE ]

            case 8: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # POSITION
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # AUTOTUNE
                    return [ self.inavctl.inav.modesID.AUTO_TUNE ]

            case 9:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # LAND
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]  

            case 10:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # OF_LOITER
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # AUTO
                    return [ self.inavctl.inav.modesID.NAV_WP ]

            case 11: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # DRIFT
                    return [ self.inavctl.inav.modesID.ANGLE ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # RTL
                    return [ self.inavctl.inav.modesID.RTH ]

            case 12: 
                if self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # LOITER
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]

            case 13: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # SPORT
                    return [  ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # TAKEOFF
                    return [ self.inavctl.inav.modesID.NAV_LAUNCH ]

            case 14: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # FLIP
                    return None
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # AVOID_ADSB
                    return None

            case 15: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # AUTOTUNE
                    return [ self.inavctl.inav.modesID.AUTO_TUNE ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # GUIDED
                    return [ self.inavctl.inav.modesID.GCS_NAV, self.inavctl.inav.modesID.NAV_POSHOLD ] 

            case 16: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # POSHOLD
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # INITIALISING
                    return [ self.inavctl.inav.modesID.NAV_MANUAL ]

            case 17:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # BRAKE
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QSTABILIZE
                    return [ self.inavctl.inav.modesID.NAV_ANGLE ]

            case 18:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # THROW
                    return None
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QHOVER
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]

            case 19: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # AVOID ADSB
                    return None
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QLOITER
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]

            case 20: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # GUIDED_NOGPS
                    return [ self.inavctl.inav.modesID.GCS_NAV, self.inavctl.inav.modesID.NAV_POSHOLD ] 
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QLAND
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]

            case 21: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # SMART_RTL
                    return [ self.inavctl.inav.modesID.RTH ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QRTL
                    return [ self.inavctl.inav.modesID.RTH ]

            case 22:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # FLOWHOLD
                    return [ self.inavctl.inav.modesID.NAV_POSHOLD ]  
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QAUTOTUNE
                    return [ self.inavctl.inav.modesID.AUTOTUNE ]

            case 23: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # FOLLOW
                    return [ self.inavctl.inav.modesID.GCS_NAV, self.inavctl.inav.modesID.NAV_POSHOLD ] 
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # QACRO
                    return [  ]

            case 24: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # ZIGZAG
                    return None
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # THERMAL
                    return None

            case 25: 
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # SYSTEMID
                    return [ self.inavctl.inav.modesID.MANUAL ] 
                elif self.type == mavutil.mavlink.MAV_TYPE_FIXED_WING: # LOITERALTQLAND
                    return [ self.inavctl.inav.modesID.POSHOLD ]

            case 26:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # AUTOROTATE
                    return None

            case 27:
                if self.type == mavutil.mavlink.MAV_TYPE_QUADROTOR: # AUTO_RTL
                    return [ self.inavctl.inav.modesID.RTH ]

            case _:
                return None

    def receive_messages(self):
        msg = self.mavconn.recv_match(blocking=False)
        if msg is not None:
            if msg.get_type() == 'COMMAND_LONG':
                command_name = self.translate_command(msg.command)
                reply = None
                
                # https://mavlink.io/en/messages/common.html#mav_commands


                if msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    #print(f"SET_MSG_INTERVAL: {self.translate_command(msg.param1)} {msg.param2}")
                    reply = True # handle later

                # ARM/DISARM
                elif msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    arm_disarm = msg.param1 == 1.0
                    self.arm(arm_disarm)
                    reply = self.armed == arm_disarm
                    print(f"Vehicle {'armed' if self.armed else 'disarmed'}")

                    self.system_status = mavutil.mavlink.MAV_STATE_ACTIVE #fake it for now
                
                # mode change
                elif msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    mode_id = int(msg.param2)
                    reply = self.set_mode(mode_id)
                    #print(f"Mode changed: Sending ACK for mode change to {self.mode}")
                    self.inavctl

                # mission start
                elif msg.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
                    print(f"Message requested: {self.translate_command(msg.param1)}")
                    reply = False

                
                elif msg.command == mavutil.mavlink.MAV_CMD_MISSION_START:
                    print("Command: Mission start")
                    reply = self.set_mode(self.modes_index["AUTO"])

                else:
                    print(f"Received message: COMMAND_LONG -> {msg.to_dict()}, Command: {command_name} ({msg.command})")

                if reply is not None:
                    cmd_res = mavutil.mavlink.MAV_RESULT_ACCEPTED if reply == True else mavutil.mavlink.MAV_RESULT_FAILED
                    self.mavconn.mav.command_ack_send(msg.command, cmd_res)

            elif msg.get_type() == 'REQUEST_DATA_STREAM': # deprecated!
                stream_name = mavutil.mavlink.enums['MAV_DATA_STREAM'].get(msg.req_stream_id).name
                #self.telemetry.stream_rates[stream_name] = msg.req_message_rate
                self.mavconn.mav.data_stream_send(
                    stream_id=msg.req_stream_id,
                    message_rate=msg.req_message_rate,
                    on_off=msg.start_stop
                )
                print(f"Received message: REQUEST_DATA_STREAM -> Stream: {stream_name}, Rate: {msg.req_message_rate}, Start/Stop: {msg.start_stop}")


            #elif msg.get_type() == 'PARAM_REQUEST_LIST': # this makes things worse
            #    print(f"Received PARAM_REQUEST_LIST -> {msg.to_dict()}")
            #    self.mavconn.mav.param_value_send(
            #        param_id=b'\x00' * 16,  # 16-byte empty parameter ID
            #        param_value=0, 
            #        param_type=0,
            #        param_count=0,  
            #        param_index=0 
            #    )

            # handle this LATER 
            #elif msg.get_type() == 'PARAM_REQUEST_LIST':
            #    print(f"Received PARAM_REQUEST_LIST -> {msg.to_dict()}")
            #    self.handle_param_request_list(msg)
                
            #elif msg.get_type() == 'PARAM_SET':
            #    print(f"Received PARAM_SET -> {msg.to_dict()}")
            #    self.handle_param_set(msg)

            elif msg.get_type() == 'MISSION_ITEM': 
                print(f"Received {msg.get_type()}: Sequence {msg.seq}, Command {msg.command}, " 
                    f"Coordinates (Lat: {msg.x}, Lon: {msg.y}, Alt: {msg.z})")

                self.mission_items.append(msg)  # Store the mission item
                self.mavconn.mav.mission_ack_send(
                    target_system=self.mavconn.target_system,
                    target_component=self.mavconn.target_component,
                    type=mavutil.mavlink.MAV_MISSION_ACCEPTED
                )
            
            elif msg.get_type() == 'MISSION_ITEM_INT':
                print(f"Received {msg.get_type()}: Sequence {msg.seq}, Command {msg.command}, " 
                    f"Coordinates (Lat: {msg.x}, Lon: {msg.y}, Alt: {msg.z})")

                self.mission_items.append(msg)  # Store the mission item

                if len(self.mission_items) < self.expected_mission_count:
                    # Request the next mission item
                    self.mavconn.mav.mission_request_int_send(
                        target_system=self.mavconn.target_system,
                        target_component=self.mavconn.target_component,
                        seq=len(self.mission_items)
                    )
                    print(f"Requested mission item {len(self.mission_items)}")

                else:
                    # All mission items received, send MISSION_ACK
                    result = mavutil.mavlink.MAV_MISSION_ACCEPTED
                    self.mission_download = False
                    self.mavconn.mav.mission_ack_send(
                        target_system=self.mavconn.target_system,
                        target_component=self.mavconn.target_component,
                        type=result
                    )
                    print(f"Sent MISSION_ACK: result={result}")
                    self.current_mission_seq = 0
                    self.mission_state = mavutil.mavlink.MISSION_STATE_NOT_STARTED

            elif msg.get_type() == 'MISSION_COUNT':
                self.expected_mission_count = msg.count
                self.mission_download = True
                print(f"Received MISSION_COUNT: Count {msg.count}")
                # Request the first mission item
                self.mavconn.mav.mission_request_int_send(
                    target_system=self.mavconn.target_system,
                    target_component=self.mavconn.target_component,
                    seq=0
                )
                print("Requested mission item 0")

            elif msg.get_type() == 'MISSION_CLEAR_ALL':
                result = mavutil.mavlink.MAV_MISSION_ACCEPTED
                self.mavconn.mav.mission_ack_send(
                    target_system=self.mavconn.target_system,
                    target_component=self.mavconn.target_component,
                    type=result
                )
                print(f"Sent MISSION_ACK: result={result}")
                self.current_mission_seq = 0
                self.mission_state = mavutil.mavlink.MISSION_STATE_NOT_STARTED
                self.mission_items = []

            elif msg.get_type() == 'MISSION_REQUEST_LIST':
                print(f"Received MISSION_REQUEST_LIST: Sending {len(self.mission_items)} mission items.")
                self.handle_mission_request_list(msg)

            elif msg.get_type() == 'MISSION_REQUEST_INT':
                self.handle_mission_request_int(msg)


            elif msg.get_type() == 'COMMAND_INT':

                if msg.command == mavutil.mavlink.MAV_CMD_DO_REPOSITION:
                    print(f"Received MAV_CMD_DO_REPOSITION: Speed={msg.param1} m/s, Bitmask={msg.param2}, "
                        f"Radius={msg.param3} m, Yaw={msg.param4} deg, Latitude={msg.x}, Longitude={msg.y}, Altitude={msg.z}")
                else:
                    print(f"Received COMMAND_INT: Command {msg.command}, Coordinates (Lat: {msg.x}, Lon: {msg.y}, Alt: {msg.z})")

                result = mavutil.mavlink.MAV_MISSION_ACCEPTED
                self.mavconn.mav.mission_ack_send(
                    target_system=self.mavconn.target_system,
                    target_component=self.mavconn.target_component,
                    type=result
                )
                print(f"Sent MISSION_ACK: result={result}")

            elif msg.get_type() == 'HEARTBEAT':
                self.last_gcs_heartbeat = time.time()

            else: 
                print(f"Received -> {msg.to_dict()}")

    
    async def mav_ctl(self):
        print("Starting MAVLINK interface...")
        try:
            set_alt = 50
            last_msp = time.time()
            self.inavctl.debugprint = False
            self.inavctl.modes.keys()
            self.inavctl.new_supermode('GOTO', [self.inavctl.inav.modesID.GCS_NAV, self.inavctl.inav.modesID.NAV_POSHOLD])
            #self.inavctl.set_mode(self.inavctl.inav.modesID.MSP_RC_OVERRIDE, on=True)
            while self.running:
                if not self.inavctl.run: 
                    self.running = False
                    break
                current_time = time.time()
                self.update()
                self.telemetry.send(current_time)
                self.receive_messages()

                if time.time()-last_msp >= 0.1:
                    gpsd = self.inavctl.get_gps_data()
                    alt = self.inavctl.get_altitude()
                    gyro = self.inavctl.get_attitude()
                    self.position = geospatial.GPSposition(gpsd['lat'], gpsd['lon'], alt)
                    self.attitude =  {
                        'roll': math.radians(gyro['roll']), 
                        'pitch': math.radians(gyro['pitch']), 
                        'yaw': math.radians(gyro['yaw']), 
                        'rollspeed': 0.0, 
                        'pitchspeed': 0.0, 
                        'yawspeed': 0.0
                    }
                    self.battery =  {
                        'voltage': self.inavctl.board.ANALOG['voltage'] / 10.0, 
                        'current': self.inavctl.board.ANALOG['amperage'], 
                        'remaining': self.inavctl.board.ANALOG['battery_percentage'], 
                        'mwhdrawn': self.inavctl.board.ANALOG['mWhdrawn'],
                        'temperature': 0, 
                        'mahdrawn': self.inavctl.board.ANALOG['mAhdrawn']
                        }
                    self.sys_status = {
                        'voltage_battery': self.inavctl.board.ANALOG['voltage'], 
                        'current_battery': self.inavctl.board.ANALOG['amperage'], 
                        'battery_remaining': self.inavctl.board.ANALOG['battery_percentage'], 
                        'errors': 0
                        }
                    nav_status = self.inavctl.get_nav_status()
                    #print(nav_status)

                #vector = geospatial.gps_to_vector(pos, wp)
                #print('\n')
                #print('Channels:', inavctl.board.RC['channels'])
                #inavctl.set_mode("MSP RC OVERRIDE", on=True)
                #print('Active modes:', self.inavctl.get_active_modes())
                #print('Position:', self.position)
                #print('Attitude:', gyro)
                #print('Altitude:', alt)
                #print('Vector to waypoint:', vector)
                #print('Bearing:',vector.az - gyro['yaw'])
                #if inavctl.msp_override_active:
                #    print('OVERRIDE ACTIVE')
                await asyncio.sleep(0.01)

            self.running = False
            inavctl.stop()
            print('Stopped')

                

        except Exception:
            print('!!! Error in Flight Control loop !!!')
            print(traceback.format_exc())
            self.running = False
            self.system_status = mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION
            for i in range(3):
                self.telemetry.send_heartbeat()
                await asyncio.sleep(1)
            return 1



async def main():
    uavctl = UAVControl(device='/dev/ttyUSB0', baudrate=115200, platform="AIRPLANE")
    uavctl.msp_override_channels=[1, 2, 3, 4, 5, 6, 14]
    uavctl.msp_receiver = True

    try:
        await uavctl.connect()
        mav = MavlinkControl(inav_conn=uavctl, platform_type=mavutil.mavlink.MAV_TYPE_FIXED_WING, use_mavlink2=True)
        print("Connected to the flight controller")
        flight_control_task = asyncio.create_task(uavctl.flight_loop())
        mavloop = asyncio.create_task(mav.mav_ctl())
        await asyncio.gather(flight_control_task, mavloop)
    finally:
        print('\nConnection closed')

if __name__ == '__main__':
    asyncio.run(main())
