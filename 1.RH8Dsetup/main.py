# -*- coding: utf-8 -*-
from pypot.dynamixel import autodetect_robot

my_robot = autodetect_robot()

for m in my_robot.motors:
    m.goal_position = 0.0

import json

config = my_robot.to_config()

with open('my_robot.json', 'w') as f:
    json.dump(config, f)

from pypot.robot import from_json

my_robot = from_json('my_robot.json')
print(my_robot.motors)

