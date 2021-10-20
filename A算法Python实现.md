# A*算法Python实现

```python
import numpy as np
import sys
from matplotlib.patches import Rectangle
from matplotlib.pyplot import MultipleLocator, close
import matplotlib.pyplot as plt


class Point:
    '''
    路径点的定义
    '''
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.G_cost = sys.maxsize
        self.H_cost = sys.maxsize
        self.parent = parent

    def __repr__(self) -> str:
        return f"Point({self.x}, {self.y})"


    def __eq__(self, o: object) -> bool:
        if self.x == o.x and self.y == o.y:
            return True
        return False


def selectPointInOpenList(open_list:list):
    """
    在待前行点列表中选出损失最小的点
    """
    mincost_index = None
    mincost = sys.maxsize
    # 先从新加入的节点搜索，这样可以获得跟好的结果
    for index, point in enumerate(reversed(open_list)):
        if (point.G_cost + point.H_cost) < mincost:
            mincost = point.G_cost + point.H_cost
            mincost_index = len(open_list) - 1 - index
    return mincost_index


def isWalkable(pointNeighbourhood:Point, point:Point, mapsize:tuple, open_list:list, close_list:list, obstracle_list:list):
    """
    判断该点是否可行
    """
    # 是否超出地图区域
    if pointNeighbourhood.x >= mapsize[0] or pointNeighbourhood.x < 0:
        return False
    if pointNeighbourhood.y >= mapsize[1] or pointNeighbourhood.y < 0:
        return False
    # 是否是障碍物
    if pointNeighbourhood in obstacle_list:
        return False
    # 是否是已经走过的点
    if pointNeighbourhood in close_list:
        return False
    # 是否已经加入待前行点列表中
    if pointNeighbourhood in open_list:
        isNeedBetterParent(pointNeighbourhood, point)
    return True


def isNeedBetterParent(pointNeighbourhood:Point, point:Point):
    """
    当邻域的点已经在待前行列表中时，检查邻域点经过当前点的路径是否比领域点之前的路径G_cost更小
    如果更小的话，更新领域点的父节点为当前点，更新新路径的G_cost
    """
    new_G_cost = diagonal_cost(pointNeighbourhood, point) + point.G_cost
    if new_G_cost < pointNeighbourhood.G_cost:
        pointNeighbourhood.G_cost = new_G_cost
        pointNeighbourhood.parent = point


def searchForNeighbourhood(point:Point, mapsize:tuple, open_list:list, close_list:list, obstracle_list:list):
    """
    1 探索该点相邻的八个点
    2 找出可行点加入待前行点列表中
    """
    bias = (-1, 0, 1) # 左下、正左、左上、正下、正上、右下、正右、右上

    for i in bias:
        for j in bias:
            if i == 0 and j == 0:
                continue
            pointNeighbourhood = Point(point.x + i, point.y + j, point)
            if isWalkable(pointNeighbourhood, point, mapsize, open_list, close_list, obstracle_list) is True:
                # 计算损失
                # G_cost 是当前节点到父节点之前的损失 + 父节点的G_cost
                # H_cost 是估计当前节点到终点的损失
                pointNeighbourhood.G_cost = diagonal_cost(pointNeighbourhood, point) + point.G_cost
                pointNeighbourhood.H_cost = diagonal_cost(pointNeighbourhood, end_point)
                open_list.append(pointNeighbourhood)


def diagonal_cost(current_point:Point, target_point:Point):
    """
    对角损失计算函数
    """
    xd = abs(current_point.x - target_point.x)
    yd = abs(current_point.y - target_point.y)
    return xd + yd + (np.sqrt(2) - 2) * min(xd, yd)
    

if __name__ == '__main__':
    # 地图大小设定
    mapsize = (11, 12)
    # 设定地图的起点和终点
    start_point = Point(1, 2)
    end_point = Point(8, 2)
    # 设定障碍物
    obstacle_list = [Point(3, 0), Point(3, 1), Point(3, 2), Point(3, 3), Point(3, 4), Point(3, 5), Point(4, 5)]

    # 初始化一张画板
    plt.figure(figsize=(5, 5))

    # 获取该画板的轴域（具体可编辑的画）
    ax = plt.gca()
    ax.set_xlim([0, mapsize[0]])
    ax.set_ylim([0, mapsize[1]])
    ax.set_aspect('equal', adjustable='box') # 设定x、y轴单位长度相等

    # 设定轴域的坐标轴刻度，并基于刻度画出方格背景
    major_locator=MultipleLocator(1)
    ax.xaxis.set_major_locator(major_locator)
    ax.yaxis.set_major_locator(major_locator)
    ax.grid()

    # 绘制地图的起点和终点
    ax.add_patch(Rectangle((start_point.x, start_point.y), width=1, height=1, color='green')) # 起点
    ax.add_patch(Rectangle((end_point.x, end_point.y), width=1, height=1, color='red'))       # 终点
    plt.pause(0.01)

    # 绘制障碍物
    for obstacle in obstacle_list:
        rec = Rectangle((obstacle.x, obstacle.y), width=1, height=1, color='gray')
        ax.add_patch(rec)
    plt.pause(0.01)

    # 开闭集合设定
    open_list = []
    close_list = []

    # 计算启点的损失
    start_point.G_cost = diagonal_cost(start_point, start_point)
    start_point.H_cost = diagonal_cost(start_point, end_point)
    # 将起始点加入待前行列表中
    open_list.append(start_point)

    # 设定当前点作为任务执行点
    current_point = start_point
    while True:
        # 1 从待前行列表中选出损失最小的点，作为下一步前行的点
        mincost_index = selectPointInOpenList(open_list)
        current_point = open_list[mincost_index]

        # 2 将该点从待前行点列表中移除，并加入到已前行列表中
        close_list.append(open_list.pop(mincost_index))

        # 3 查看起始点相邻的点，选择其中可到达的放个加入待前行列表中
        searchForNeighbourhood(current_point, mapsize, open_list, close_list, obstacle_list)

        # 可视化试探结果
        for openPoint in open_list:
            rec = Rectangle((openPoint.x, openPoint.y), width=1, height=1, color='#66ff99')
            ax.add_patch(rec)

        for closePoint in close_list:
            rec = Rectangle((closePoint.x, closePoint.y), width=1, height=1, color='#C9C9C9')
            ax.add_patch(rec)
        plt.pause(1)

        # 当到达终点退出路径试探
        if current_point == end_point:
            break
    
    # 绘制最终路径
    while True:
        rec = Rectangle((current_point.x, current_point.y), width=1, height=1, color='blue')
        ax.add_patch(rec)
        plt.pause(0.01)

        current_point = current_point.parent
        if current_point is None:
            break
    
    plt.show()
```

