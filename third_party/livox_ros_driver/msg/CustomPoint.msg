# Livox 点格式。字段、类型、顺序必须与官方 livox_ros_driver 一致（MD5 校验）
uint32 offset_time      # 相对 timebase 的偏移，纳秒
float32 x               # m
float32 y               # m
float32 z               # m
uint8 reflectivity      # 0~255
uint8 tag               # livox tag（噪点标记）
uint8 line              # 激光线号
