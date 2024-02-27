import pypot.dynamixel
from pypot.dynamixel import DxlIO
# 创建 DxlIO 对象
dxl_io = pypot.dynamixel.io.DxlIO(port='COM8', baudrate=1000000, timeout=0.1)
# with DxlIO('COM8') as dxl_io:
ids = [30, 31, 32, 33, 34, 35, 36, 37, 38]
# 获取当前位置信息
present_positions = dxl_io.get_present_position(ids)
print(present_positions)