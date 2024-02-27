import pypot.dynamixel

# 创建 DxlIO 对象
dxl_io = pypot.dynamixel.io.DxlIO(port='COM8', baudrate=1000000, timeout=0.1)

# 下电
dxl_io.disable_torque([30, 31, 32, 33, 34, 35, 36, 37, 38])  # 传入要下电的电机 ID 列表

# 关闭通信
dxl_io.close()