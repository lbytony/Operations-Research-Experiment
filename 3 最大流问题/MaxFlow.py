# -*- coding: UTF-8 -*-
import copy
from collections import deque
import numpy as np

""" 
图论最大流问题的求解
:author: Li Boyan 
:date: 2020/05/24
"""


def inRange(target, a, b):
    if a <= target <= b:
        return True
    else:
        return False


class MaxFlow:

    def __init__(self):
        self.maxFlow = 0
        self.start = -1
        self.end = -1

    def inputMap(self):
        self.n = int(input("请输入图的顶点个数："))
        self.matrix = [[0 for i in range(self.n)] for i in range(self.n)]
        self.path = list(range(self.n))
        self.edge = int(input("请输入图的有向边个数："))
        i = 0
        while i < self.edge:
            a, b, c, f = map(int, input("请输入第{}个有向边的起点、终点、容量和流量：".format(i + 1)).split())
            if not (inRange(a, 0, self.n-1) and inRange(b, 0, self.n-1) and c >= f):
                print("数据输入有误，请查证后在输入！")
                continue
            self.matrix[a][b] = c - f
            self.matrix[b][a] = f
            i = i + 1
        self.start = int(input("请输入起始点的标号："))
        self.end = int(input("请输入结束点的标号："))

    def printMatrix(self):
        print("存储的残存网络的邻接矩阵为：")
        print(np.array(self.matrix))

    def inputMapManual(self):
        """
        通过修改程序，手动设置网络的邻接矩阵
        """
        self.n = 7
        self.matrix = [
            [0, 6, 0, 0, 0, 0, 0],
            [7, 0, 4, 0, 0, 0, 0],
            [9, 0, 0, 5, 0, 0, 0],
            [0, 6, 0, 0, 3, 4, 0],
            [0, 5, 0, 2, 0, 0, 2],
            [0, 0, 5, 0, 0, 0, 5],
            [0, 0, 0, 4, 7, 5, 0]
        ]
        self.path = list(range(self.n))
        self.start = 0
        self.end = 6

    def doMaxFlow(self):
        graph = copy.deepcopy(self.matrix)

        while self.BFS(graph):
            cfp = float('inf')

            # 寻找残余容量 c_f(p)
            i = self.end
            while i != self.start:
                u = self.path[i]
                cfp = min(cfp, graph[u][i])
                i = self.path[i]

            # 在每条边上添加流量
            i = self.end
            while i != self.start:
                u = self.path[i]
                graph[u][i] -= cfp
                graph[i][u] += cfp
                i = self.path[i]
            self.maxFlow += cfp
        return graph

    def BFS(self, graph):
        """
        BFS算法查找增广链
        """
        visited = [False for _ in range(self.n)]
        visited[self.start] = True
        queue = deque([self.start])
        while queue:
            temp = queue.popleft()
            if temp == self.end:    # 找到路径
                return True
            for i in range(self.n):
                if not visited[i] and (graph[temp][i] > 0):
                    queue.append(i)
                    visited[i] = True
                    self.path[i] = temp
        return visited[self.end]

    def printResult(self, graph):
        print("flow =", self.maxFlow)
        print("最大流图为：\n", np.array(graph))


def main():
    """
    图论最大流问题的求解
    """
    data = 0
    print("================= Begin ================")
    mf = MaxFlow()
    if data == 0:
        mf.inputMap()
    else:
        print("已通过程序内部数据导入网络构型")
        mf.inputMapManual()
    mf.printMatrix()
    graph = mf.doMaxFlow()
    mf.printResult(graph)
    print("================== End =================")


if __name__ == '__main__':
    main()
