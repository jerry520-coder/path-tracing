import math


# IMU传感器数据
acceleration = [0.67516, -7.91523, 5.88016]  # 加速度
# gyroscope = [-0.00107, 0.00533, 0.01278]  # 陀螺仪

# 加速度计算roll和pitch
roll = math.atan2(acceleration[1], acceleration[2])
pitch = math.atan2(
    -acceleration[0], math.sqrt(acceleration[1] ** 2 + acceleration[2] ** 2)
)

# 转换为角度（可选）
# roll = math.degrees(roll)
# pitch = math.degrees(pitch)


print("Roll:", roll)
print("Pitch:", pitch)
