import numpy as np
import matplotlib.pyplot as plt

# 从文本文件中读取一维数组
data1 = np.loadtxt('beta=1.0.txt')
data2 = np.loadtxt('beta=3.0.txt')
data3 = np.loadtxt('beta=5.0.txt')
data4 = np.loadtxt('beta=7.0.txt')
data5 = np.loadtxt('beta=10.0.txt')

# 获取 x 轴的值，范围从第 5 个到第 40 个数据点
x_values = np.arange(5, 30)

# 绘制图表
plt.plot(x_values, data1[1,4:29], color='blue', label='beta=1.0')
plt.fill_between(x_values, data1[0,4:29], data1[2,4:29], color='blue', alpha=0.1)

plt.plot(x_values, data2[1,4:29], color='red', label='beta=3.0')
plt.fill_between(x_values, data2[0,4:29], data2[2,4:29], color='red', alpha=0.1)

plt.plot(x_values, data3[1,4:29], color='green', label='beta=5.0')
plt.fill_between(x_values, data3[0,4:29], data3[2,4:29], color='green', alpha=0.1)

plt.plot(x_values, data4[1,4:29], color='yellow', label='beta=7.0')
plt.fill_between(x_values, data4[0,4:29], data4[2,4:29], color='yellow', alpha=0.1)

plt.plot(x_values, data5[1,4:29], color='purple', label='beta=10.0')
plt.fill_between(x_values, data5[0,4:29], data5[2,4:29], color='purple', alpha=0.1)

plt.xlabel('Epochs')
plt.ylabel('Loss')
plt.title('Loss vs. Epochs of different beta: Start')
plt.grid(True)
plt.legend()  # 添加图例
plt.show()

