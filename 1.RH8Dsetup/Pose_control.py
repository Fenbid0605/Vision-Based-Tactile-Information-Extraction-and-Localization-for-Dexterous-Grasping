import time
import motionhub    #自己编写的子程序
import pypot
import pypot.dynamixel
from pypot.robot import from_json
from pypot.dynamixel import DxlIO
import itertools
my_robot = from_json('my_robot.json')
# print(my_robot)
# print(pypot.dynamixel.get_available_ports(only_free=False))
# print(pypot.dynamixel.find_port(ids=(30,31,32,33,34,35,36,37,38),strict=True))
# print('hello')
if __name__ == '__main__':
#     ports = pypot.dynamixel.get_available_ports()
#     print('available ports:', ports)
#
#     if not ports:
#         raise IOError('No port available.')
#
#     port = ports[0]
#     print('Using the first on the list', port)
#
#     dxl_io = pypot.dynamixel.DxlIO(port)
#     print('Connected!')

    # found_ids = dxl_io.scan()
    # print('Found ids:', found_ids)
    #
    # if len(found_ids) < 2:
    #     raise IOError('You should connect at least two motors on the bus for this test.')
    #
    # ids = found_ids[:9],先屏蔽，不用每次打开都搜索一下
    # ids = [30, 31, 32, 33, 34, 35, 36, 37, 38]
    # dxl_io.enable_torque(ids)
    # pos = dict(zip(ids, itertools.repeat(0)))
    # print(pos)
    # dxl_io.set_goal_position(pos)
    motionhub.victory()
    time.sleep(3)
    i=5
    while i>0:
        i=i-1
        motionhub.position_detect()
        motionhub.palm_open()






