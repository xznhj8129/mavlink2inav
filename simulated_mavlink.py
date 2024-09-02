from pymavlink import mavutil
import time
import inspect 
import traceback
from mavint import *


if __name__ == "__main__":
    uav = MavlinkControl(platform_type=mavutil.mavlink.MAV_TYPE_FIXED_WING, use_mavlink2=True)
    uav.run()
