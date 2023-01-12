#!/usr/bin/env python3
from autoPluto import autoPluto


drone1 = autoPluto()
# print("arming")
drone1.comms.arm()
# print("calling run")
drone1.run()