import numpy as np

# 打开文件
with open("error_mpc_y.txt", "r") as file:
    numbers_pas = [float(line.strip()) for line in file]

with open("error_fmpc_y.txt", "r") as file:
    numbers_pos = [float(line.strip()) for line in file]

# 计算平均值
average_pas = sum(numbers_pas) / len(numbers_pas)
average_pos = sum(numbers_pos) / len(numbers_pos)


print("This is the xy-axis motion group (Control group):")
print("SIM crash happend times: 28")
print("Passing time: 0.7928")
print("max passing error:", np.max(numbers_pas),"\nmin passing error:",np.min(numbers_pas))
print("Averaged passing error:", average_pas)
print("max final error:", np.max(numbers_pos),"\nmin final error",np.min(numbers_pos))
print("Averaged final error:", average_pos)

