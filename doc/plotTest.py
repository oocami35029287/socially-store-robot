import matplotlib.pyplot as plt
from matplotlib.path import Path
plt.use('TkAgg')
from matplotlib.patches import PathPatch

# 创建一个简单的三角形路径
triangle_path = Path([(0, 0), (1, 0), (0.5, 1), (0, 0)])

# 创建一个路径补丁，将三角形添加到图形中，并设置填充颜色
triangle_patch = PathPatch(triangle_path, facecolor='blue', edgecolor='black')

# 创建一个图形
fig, ax = plt.subplots()

# 添加路径补丁到图形中
ax.add_patch(triangle_patch)

# 设置坐标轴范围
ax.set_xlim(-1, 2)
ax.set_ylim(-1, 2)

# 显示图形
plt.show()
