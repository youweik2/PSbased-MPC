import numpy as np

# 打开文件
with open("error_door1.txt", "r") as file:
    numbers_pas = [float(line.strip()) for line in file]

with open("error_door2.txt", "r") as file:
    numbers_pos = [float(line.strip()) for line in file]

# 计算平均值
average_pas = sum(numbers_pas) / len(numbers_pas)
average_pos = sum(numbers_pos) / len(numbers_pos)


print("This is the two-gate motion group (Test group):")
print("SIM crash happend times: 7")
print("max first door error:", np.max(numbers_pas),"\nmin first door error:",np.min(numbers_pas))
print("Averaged first door passing error:", average_pas)
print("max second error:", np.max(numbers_pos),"\nmin second error",np.min(numbers_pos))
print("Averaged second door passing error:", average_pos)

