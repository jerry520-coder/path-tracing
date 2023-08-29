import numpy as np
import matplotlib.pyplot as plt
import math
import argparse

# 解析命令行参数
parser = argparse.ArgumentParser()
parser.add_argument("--no-display", action="store_true")
args = parser.parse_args()


TIMESTAMP_ALIGNMENT = 0  # alignment


def interpolate_and_difference(data1, data2):
    # 将数据转换为numpy数组
    data1 = np.array(data1)
    data2 = np.array(data2)

    # 分别获取时间戳和值
    time1, values1 = data1[:, 0], data1[:, 1]
    time2, values2 = data2[:, 0], data2[:, 1]

    # 获取所有时间戳的并集，然后进行排序
    all_times = np.union1d(time1, time2)
    all_times.sort()

    # 使用线性插值获取在所有时间戳处的值
    interpolated_values1 = np.interp(all_times, time1, values1)
    interpolated_values2 = np.interp(all_times, time2, values2)

    # 计算两组数据的差值
    difference = interpolated_values1 - interpolated_values2

    # 返回新的数据，包含时间戳和差值
    return np.column_stack((all_times, difference))


def normalize_angle(angle):
    """
    Normalize an angle to the range of (-pi, pi]
    """
    normalized_angle = angle % (2 * math.pi)
    normalized_angle[normalized_angle > math.pi] -= 2 * math.pi
    return normalized_angle


def convert_angle(angle):
    """
    角度在 [0, 2π) 范围内循环是周期性的。你可以使用取余运算来实现循环转换，即将 x 取余 2π,得到的结果就是 [0, 2π) 范围内的角度。
    """
    new_angle = angle + math.pi

    # 使用模运算将角度映射到[0, 2π)区间
    new_angle = new_angle % (2 * math.pi)

    # 将角度映射到0到4π区间
    final_angle = 2 * new_angle

    return final_angle


################################################ 1. tracker ###############################################

# 读取第二个文件 期望x/y/yaw
# file2 = "/home/binghe/control data/0814/2/tracker.txt"  # 开始路径规划的部分
# file2 = "/home/binghe/control data/202308141136/tracker.txt"  # 开始路径规划的部分
file2 = "./tracker.txt"
data2 = np.loadtxt(file2, delimiter=",")
timestamps_tracker = data2[:, 0] / 1000  # 单位s
timestamps_tracker_align = data2[:, 0] / 1000  # 单位s
if TIMESTAMP_ALIGNMENT:
    timestamps_tracker_align = timestamps_tracker - timestamps_tracker[0]
desired_x = data2[:, 1]
desired_y = data2[:, 2]
desired_yaw = data2[:, 3]


################################################ 2. odom ###############################################
# 读取第一个文件 实际x/y/yaw  Vx/Vy/yaw_rate
# file1 = "/home/binghe/control data/0814/2/odom.txt"
# file1 = "/home/binghe/control data/202308141136/odom.txt"
file1 = "./odom.txt"
data1 = np.loadtxt(file1, delimiter=",")
timestamps_odom = data1[:, 0] / 1000  # 单位s
# 时间戳对齐
index = 0
if TIMESTAMP_ALIGNMENT:
    for i in range(len(timestamps_odom)):  # 实际的
        if timestamps_odom[i] >= timestamps_tracker[0]:
            index = i
            break

timestamps_odom_align = data1[index:, 0] / 1000
timestamps_odom_align_temp = np.copy(timestamps_odom_align)
if TIMESTAMP_ALIGNMENT:
    timestamps_odom_align -= timestamps_odom_align[0]
actual_x = data1[index:, 1]
actual_y = data1[index:, 2]
actual_yaw = data1[index:, 3]
# if TIMESTAMP_ALIGNMENT:
#     timestamps_odom -= timestamps_odom_align[0]
actual_Vx = data1[index:, 4]
actual_Vy = data1[index:, 5]
actual_yaw_rate = data1[index:, 6]
actual_velocity = np.sqrt(actual_Vx**2 + actual_Vy**2)

############################################# 3. actuator ##################################################
# 读取第三个文件 期望V/yaw_rate
# file3 = "/home/binghe/control data/0814/2/actuator.txt"
# file3 = "/home/binghe/control data/202308141136/actuator.txt"
file3 = "./actuator.txt"
data3 = np.loadtxt(file3, delimiter=",")
timestamps_actuator = data3[:, 0] / 1000  # 单位s
# 时间戳对齐
start = 0
if TIMESTAMP_ALIGNMENT:
    for i in range(len(timestamps_actuator)):  # 期望的
        if timestamps_actuator[i] >= timestamps_odom[0]:
            start = i
            break
# end = -1
# if TIMESTAMP_ALIGNMENT:
#     for i in range(len(timestamps_actuator) - 1, -1, -1):  # 期望的
#         if timestamps_actuator[i] <= timestamps_odom[-1]:
#             end = i
#             break
timestamps_actuator_align = data3[start:, 0] / 1000
# if TIMESTAMP_ALIGNMENT:
#     timestamps_actuator_align -= timestamps_odom_align[0]
desired_velocity = data3[start:, 1]
desired_yaw_rate = data3[start:, 2]


############################################# 4. PID 文件 ##################################################
# PID的P/I/D
file4 = "./PID.txt"
data4 = np.loadtxt(file4, delimiter=",")
timestamps_PID = data4[:, 0] / 1000  # 单位s
P = data4[:, 1]
I = data4[:, 2]
D = data4[:, 3]

############################################# 5. LFController ##################################################
# 角度误差、小车到目标点距离、线速度减速值
file5 = "./LFController.txt"
data5 = np.loadtxt(file5, delimiter=",")
timestamps_LFController = data5[:, 0] / 1000  # 单位s
angular_error = data5[:, 1]
aimpoint_body_norm = data5[:, 2]
deceleration = data5[:, 3]

############################################# 6. virtual_point ##################################################
# 小车跟踪的虚拟点
file6 = "./virtual_point.txt"
data6 = np.loadtxt(file6, delimiter=",")
timestamps_virtual_point = data6[:, 0] / 1000  # 单位s
virtual_point_x = data6[:, 1]
virtual_point_y = data6[:, 2]
target_x = data6[:, 3]
target_y = data6[:, 4]
target_w = data6[:, 5]


#
#
#
#
#
#
#
############################################# 1. 绘制位姿对比图 ##################################################
# 绘制位姿对比图
fig = plt.figure(figsize=(15, 10))
fig.suptitle("odom")
subsum = 4


# x
plt.subplot(subsum, 1, 1)
plt.plot(timestamps_tracker_align, desired_x, label="Desired x", linewidth=1, alpha=1)
plt.plot(timestamps_odom_align, actual_x, label="Actual x", linewidth=1, alpha=0.5)
plt.xlabel("Timestamp(s)")
plt.ylabel("X Position(m)")
plt.legend()

# y
plt.subplot(subsum, 1, 2)
plt.plot(timestamps_tracker_align, desired_y, label="Desired y", linewidth=1, alpha=1)
plt.plot(timestamps_odom_align, actual_y, label="Actual y", linewidth=1, alpha=0.5)
plt.xlabel("Timestamp(s)")
plt.ylabel("Y Position(m)")
plt.legend()

# yaw
plt.subplot(subsum, 1, 3)
yaw_diff = interpolate_and_difference(
    data2[:, [0, 3]], data1[:, [0, 3]]
)  # desired_yaw 、actual_yaw


plt.plot(
    timestamps_tracker_align,
    desired_yaw,
    label="Desired yaw",
    linewidth=1,
    alpha=1,
)
plt.plot(
    timestamps_odom_align,
    actual_yaw,
    label="Actual yaw",
    linewidth=1,
    alpha=0.5,
)
# plt.plot(
#     timestamps_LFController, angular_error, label="angular_error", linewidth=1, alpha=1
# )
# plt.plot(timestamps_virtual_point, target_w, label="target_w", linewidth=1, alpha=1)
# plt.plot(
#     yaw_diff[:, 0] / 1000,
#     normalize_angle(yaw_diff[:, 1]),
#     label="yaw_diff",
#     linewidth=1,
#     alpha=0.3,
# )
plt.xlabel("Timestamp(s)")
plt.ylabel("Yaw(rad)")
plt.legend()


# angular_error
plt.subplot(subsum, 1, 4)
plt.plot(
    timestamps_LFController, angular_error, label="angular_error", linewidth=1, alpha=1
)
plt.plot(
    yaw_diff[:, 0] / 1000,
    normalize_angle(yaw_diff[:, 1]),
    label="yaw_diff",
    linewidth=1,
    alpha=1,
)
# plt.plot(timestamps_odom_align, actual_yaw, label="Actual yaw", linewidth=1, alpha=0.5)
plt.xlabel("Timestamp(s)")
plt.ylabel("angular_error(rad)")
plt.legend()

# deceleration
# plt.subplot(subsum, 1, 5)
# plt.plot(
#     timestamps_LFController, deceleration, label="deceleration", linewidth=1, alpha=1
# )
# plt.xlabel("Timestamp(s)")
# plt.ylabel("deceleration(m/s)")
# plt.legend()

# D
# plt.subplot(subsum, 1, 6)
# plt.plot(timestamps_PID, D, label="D", linewidth=1, alpha=1)
# plt.xlabel("Timestamp(s)")
# plt.ylabel("D")
# plt.legend()


############################################# 2. 绘制速度与角速度对比图 ##################################################
# 绘制速度与角速度对比图
fig = plt.figure(figsize=(15, 10))
fig.suptitle("Velocity and Yaw Rate")
subsum = 5

# 线速度
plt.subplot(subsum, 1, 1)
plt.plot(
    timestamps_actuator_align,
    desired_velocity,
    label="Desired Velocity",
    linewidth=1.5,
    alpha=1,
)
plt.plot(
    timestamps_odom_align_temp,
    actual_velocity,
    label="Actual Velocity",
    linewidth=0.5,
    alpha=0.7,
)
plt.xlabel("Timestamp(s)")
plt.ylabel("Velocity(m/s)")
plt.title("Actual vs Desired Velocities")
plt.ylim(-0.1, 0.5)
plt.legend()

# 角速度
plt.subplot(subsum, 1, 2)
plt.plot(
    timestamps_actuator_align,
    desired_yaw_rate,
    label="Desired Yaw Rate",
    linewidth=1.5,
    alpha=1,
)
plt.plot(
    timestamps_odom_align_temp,
    actual_yaw_rate,
    label="Actual Yaw Rate",
    linewidth=0.5,
    alpha=0.7,
)
plt.xlabel("Timestamp(s)")
plt.ylabel("Yaw Rate(rad/s)")
plt.title("Actual vs Desired Yaw Rates")
plt.ylim(-0.8, 0.8)
plt.legend()


# yaw
plt.subplot(subsum, 1, 3)
plt.plot(
    timestamps_tracker_align, desired_yaw, label="Desired yaw", linewidth=1, alpha=1
)
plt.plot(timestamps_odom_align, actual_yaw, label="Actual yaw", linewidth=1, alpha=0.5)
plt.xlabel("Timestamp(s)")
plt.ylabel("Yaw(rad)")
plt.legend()

# angular_error
plt.subplot(subsum, 1, 4)
plt.plot(
    timestamps_LFController, angular_error, label="angular_error", linewidth=1, alpha=1
)
plt.plot(
    yaw_diff[:, 0] / 1000,
    normalize_angle(yaw_diff[:, 1]),
    label="yaw_diff",
    linewidth=1,
    alpha=1,
)
# plt.plot(timestamps_odom_align, actual_yaw, label="Actual yaw", linewidth=1, alpha=0.5)
plt.xlabel("Timestamp(s)")
plt.ylabel("angular_error(rad)")
plt.legend()

# angular_error
plt.subplot(subsum, 1, 5)
plt.plot(
    timestamps_LFController, angular_error, label="angular_error", linewidth=1, alpha=1
)
# plt.plot(
#     yaw_diff[:, 0] / 1000,
#     normalize_angle(yaw_diff[:, 1]),
#     label="yaw_diff",
#     linewidth=1,
#     alpha=1,
# )
# plt.plot(timestamps_odom_align, actual_yaw, label="Actual yaw", linewidth=1, alpha=0.5)
plt.xlabel("Timestamp(s)")
plt.ylabel("angular_error(rad)")
plt.ylim(-0.2, 0.2)
plt.legend()


############################################# 3. PID参数 ##################################################
fig = plt.figure(figsize=(15, 10))
fig.suptitle("PID")
subsum = 4

# P
plt.subplot(subsum, 1, 1)
plt.plot(timestamps_PID, P, label="P", linewidth=1, alpha=1)
plt.xlabel("Timestamp(s)")
plt.ylabel("P")
plt.legend()

# I
plt.subplot(subsum, 1, 2)
plt.plot(timestamps_PID, I, label="I", linewidth=1, alpha=1)
plt.xlabel("Timestamp(s)")
plt.ylabel("I")
plt.legend()

# D
plt.subplot(subsum, 1, 3)
plt.plot(timestamps_PID, D, label="D", linewidth=1, alpha=1)
plt.xlabel("Timestamp(s)")
plt.ylabel("D")
plt.ylim(-2, 2)
plt.legend()

# 角速度
plt.subplot(subsum, 1, 4)
plt.plot(
    timestamps_actuator_align,
    desired_yaw_rate,
    label="Desired Yaw Rate",
    linewidth=1.5,
    alpha=1,
)
plt.plot(
    timestamps_odom_align_temp,
    actual_yaw_rate,
    label="Actual Yaw Rate",
    linewidth=0.5,
    alpha=0.5,
)
plt.xlabel("Timestamp(s)")
plt.ylabel("Yaw Rate(rad/s)")
plt.title("Actual vs Desired Yaw Rates")
plt.ylim(-0.8, 0.8)
plt.legend()


############################################# 4.yaw角误差 ##################################################
fig = plt.figure(figsize=(15, 10))
fig.suptitle("yaw erro And virtual_point")
subsum = 4

# angular_error
plt.subplot(subsum, 1, 1)
plt.plot(
    timestamps_LFController, angular_error, label="angular_error", linewidth=1, alpha=1
)
plt.plot(
    yaw_diff[:, 0] / 1000,
    normalize_angle(yaw_diff[:, 1]),
    label="yaw_diff",
    linewidth=1,
    alpha=1,
)
plt.xlabel("Timestamp(s)")
plt.ylabel("angular_error(rad)")
plt.legend()


# aimpoint_body_norm
plt.subplot(subsum, 1, 2)
plt.plot(
    timestamps_LFController,
    aimpoint_body_norm,
    label="aimpoint_body_norm",
    linewidth=1,
    alpha=1,
)
plt.xlabel("Timestamp(s)")
plt.ylabel("aimpoint_body_norm(m)")
plt.legend()

# virtual_point_x
plt.subplot(subsum, 1, 3)
plt.plot(
    timestamps_virtual_point,
    virtual_point_x,
    label="virtual_point_x",
    linewidth=1,
    alpha=1,
)
plt.plot(timestamps_virtual_point, target_x, label="target_x", linewidth=1, alpha=1)
plt.xlabel("Timestamp(s)")
plt.ylabel("virtual_point_x(m)")
plt.legend()

# virtual_point_y
plt.subplot(subsum, 1, 4)
plt.plot(
    timestamps_virtual_point,
    virtual_point_y,
    label="virtual_point_y",
    linewidth=1,
    alpha=1,
)
plt.plot(timestamps_virtual_point, target_y, label="target_y", linewidth=1, alpha=1)
plt.xlabel("Timestamp(s)")
plt.ylabel("virtual_point_y(m)")
plt.legend()


############################################# 6. 绘制位姿误差对比图 ##################################################
# 绘制位姿对比图
fig = plt.figure(figsize=(15, 10))
fig.suptitle("odom_error")
subsum = 4


# 时间戳对齐
index = 0
for i in range(len(timestamps_tracker_align)):  # 实际的
    if timestamps_odom_align[i] >= timestamps_tracker_align[0]:
        index = i
        break

# timestamps_odom_align1 = timestamps_odom_align[index:-1]

# x
plt.subplot(subsum, 1, 1)
plt.plot(timestamps_tracker_align, desired_x, label="Desired x", linewidth=1, alpha=1)
plt.plot(
    timestamps_odom_align[index:-1],
    actual_x[index:-1],
    label="Actual x",
    linewidth=1,
    alpha=0.5,
)
plt.xlabel("Timestamp(s)")
plt.ylabel("X Position(m)")
plt.legend()

# y
plt.subplot(subsum, 1, 2)
plt.plot(
    timestamps_tracker_align,
    desired_y,
    label="Desired y",
    linewidth=1,
    alpha=1,
)
plt.plot(
    timestamps_odom_align[index:-1],
    actual_y[index:-1],
    label="Actual y",
    linewidth=1,
    alpha=0.5,
)
plt.xlabel("Timestamp(s)")
plt.ylabel("Y Position(m)")
plt.legend()

# x_error
plt.subplot(subsum, 1, 3)
x_error = interpolate_and_difference(data2[:, [0, 1]], data1[index:-1, [0, 1]])
plt.plot(x_error[:, 0] / 1000, x_error[:, 1], label="Desired y", linewidth=1, alpha=1)
plt.xlabel("Timestamp(s)")
plt.ylabel("X error(m)")
plt.legend()

# 计算平均值
x_error_mean = np.mean(x_error[:, 1])
print("x_error_mean:", x_error_mean)

# y_error
plt.subplot(subsum, 1, 4)
y_error = interpolate_and_difference(data2[:, [0, 2]], data1[index:-1, [0, 2]])
plt.plot(y_error[:, 0] / 1000, y_error[:, 1], label="Desired y", linewidth=1, alpha=1)
plt.xlabel("Timestamp(s)")
plt.ylabel("Y error(m)")
plt.legend()

# 计算平均值
y_error_mean = np.mean(abs(y_error[:, 1]))
print("y_error_mean:", y_error_mean)


############################################# 7. 绘制X/Y轨迹对比图 ##################################################
# 绘制位姿对比图
fig = plt.figure(figsize=(15, 10))
fig.suptitle("X/Y track")
subsum = 1

# 时间戳对齐
# index = 0
# for i in range(len(timestamps_tracker_align)):  # 实际的
#     if timestamps_odom_align[i] >= timestamps_tracker_align[0]:
#         index = i
#         break

# x/y track
plt.subplot(subsum, 1, 1)
plt.plot(
    desired_x, desired_y, label="Desired x/y", linewidth=1, alpha=0.7, linestyle="--"
)
plt.plot(actual_x, actual_y, label="Actual x/y", linewidth=1, alpha=1)
plt.xlabel("X(m)")
plt.ylabel("Y(m)")
plt.grid(True)
plt.axis("equal")
plt.legend()


############################################# 结束绘图 ##################################################
plt.tight_layout()
if not args.no_display:
    plt.show()
