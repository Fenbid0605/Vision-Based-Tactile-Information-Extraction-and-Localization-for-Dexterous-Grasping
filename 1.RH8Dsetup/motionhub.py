import pypot.dynamixel
from pypot.dynamixel import DxlIO

ports = pypot.dynamixel.get_available_ports()
print('available ports:', ports)

if not ports:
    raise IOError('No port available.')

port = ports[0]
print('Using the first on the list', port)

dxl_io = pypot.dynamixel.DxlIO(port)
print('Connected!')
ids = [30, 31, 32, 33, 34, 35, 36, 37]
# 30 主板
# 31 腕部旋转
# 32 腕部内收
# 33 腕部屈曲
# 34 拇指内收
# 35 拇指屈曲
# 36 食指屈曲
# 37 中指屈曲
# 38 第 4 和第 5 指屈曲
dxl_io.enable_torque(ids)
def position_detect():
    print(dxl_io.get_present_position(ids))
def speed_detect():
    v_speed = dxl_io.get_present_speed(ids)
    print(v_speed)


#负数手掌往外扩，正数往内收，零值约为半握拳
def victory():
    # speed=180
    # dxl_io.set_moving_speed(ids,speed) #设置速度
    dxl_io.set_moving_speed({37: 0.01})
    dxl_io.set_goal_position({38: 105})
    dxl_io.set_goal_position({37: -155})
    dxl_io.set_goal_position({36: -170})
    dxl_io.set_goal_position({35: 65})


def button_press():
    dxl_io.set_moving_speed({ids: 0.01})
    dxl_io.set_goal_position({38: 105})
    dxl_io.set_goal_position({37: 65})
    dxl_io.set_goal_position({36: -170})
    dxl_io.set_goal_position({35: 65})

def palm_open():
    dxl_io.set_moving_speed({35: 0.00001})
    dxl_io.set_goal_position({38: -165})
    dxl_io.set_goal_position({37: -165})
    dxl_io.set_goal_position({36: -170})
    dxl_io.set_goal_position({35: -115})
    dxl_io.set_goal_position({34: -105})
