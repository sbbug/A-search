from __future__ import print_function
import matplotlib.pyplot as plt


class AStarGraph(object):
    # 定义含有屏障的类板
    def __init__(self):
        self.barriers = []
        self.barriers.append(
            [(2, 4), (2, 5), (2, 6), (3, 6), (4, 6), (5, 6), (5, 5), (5, 4), (5, 3), (5, 2), (4, 2), (3, 2)])

    def heuristic(self, start, goal):
        #使用切比雪夫距离启发式，如果我们可以移动一个正方形，无论是相邻的还是对角线
        D = 1
        D2 = 1
        dx = abs(start[0] - goal[0]) #横向距离
        dy = abs(start[1] - goal[1]) #纵向距离
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

    def get_vertex_neighbours(self, pos):
        n = []
        # 漫游八个方向
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
            x2 = pos[0] + dx
            y2 = pos[1] + dy
            if x2 < 0 or x2 > 7 or y2 < 0 or y2 > 7:
                continue
            n.append((x2, y2))
        return n

    #计算移动代价
    def move_cost(self, a, b):
        for barrier in self.barriers:
            if b in barrier:
                return 100  # 遇到墙时代价极高
        return 1  # 正常代价


def AStarSearch(start, end, graph):
    G = {}  # 从起始位置到每个位置的实际移动成本
    F = {}  #：估计从开始到结束通过这个位置的移动成本

    # 开始初始化值
    G[start] = 0
    F[start] = graph.heuristic(start, end)

    closedVertices = set()
    openVertices = set([start])
    cameFrom = {}

    while len(openVertices) > 0:
        # 得到开放列表中F值最低的顶点
        current = None
        currentFscore = None
        for pos in openVertices:
            if current is None or F[pos] < currentFscore:
                currentFscore = F[pos]
                current = pos

        # 检查我们是否达到了目标
        if current == end:
            # 回溯我们的路线
            path = [current]
            while current in cameFrom:
                current = cameFrom[current]
                path.append(current)
            path.reverse()
            return path, F[end]  # 完成

        # 将当前顶点标记为关闭
        openVertices.remove(current)
        closedVertices.add(current)

        # 更新当前位置附近顶点的得分
        for neighbour in graph.get_vertex_neighbours(current):
            if neighbour in closedVertices:
                continue  # 我们已经详尽地处理了这个节点
            candidateG = G[current] + graph.move_cost(current, neighbour)

            if neighbour not in openVertices:
                openVertices.add(neighbour)  # 发现一个新的顶点
            elif candidateG >= G[neighbour]:
                continue  # 这个G分数比之前发现的要差

            # 采用这个G分数
            cameFrom[neighbour] = current
            G[neighbour] = candidateG
            H = graph.heuristic(neighbour, end)
            F[neighbour] = G[neighbour] + H

    raise RuntimeError("A* failed to find a solution")


if __name__ == "__main__":
    graph = AStarGraph()
    result, cost = AStarSearch((0, 0), (5, 7), graph)
    print("route", result)
    print("cost", cost)
    plt.plot([v[0] for v in result], [v[1] for v in result])
    for barrier in graph.barriers:
        plt.plot([v[0] for v in barrier], [v[1] for v in barrier])
    plt.xlim(-1, 8)
    plt.ylim(-1, 8)
    plt.show()