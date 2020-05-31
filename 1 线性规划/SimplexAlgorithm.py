# -*- coding: UTF-8 -*-
"""
单纯形法实现线性规划
:author: Li Boyan
:date: 2020/03/23
"""
import time

import numpy as np

M = 9999999


def printSingleExpression(exp):
    """
    输出单个表达式
    """
    output = ''
    for i in range(len(exp)):
        if exp[i] == 1:
            if output == '':
                output += 'x' + str(i + 1)
            else:
                output += ' + x' + str(i + 1)
        elif exp[i] == -1:
            if output == '':
                output += '-x' + str(i + 1)
            else:
                output += ' - x' + str(i + 1)
        elif exp[i] == 0:
            output += ''
        elif exp[i] == -M:
            output += ' - Mx' + str(i + 1)
        elif exp[i] < 0:
            if output == '':
                output += '-' + str(int(abs(exp[i]))) + 'x' + str(i + 1)
            else:
                output += ' - ' + str(int(abs(exp[i]))) + 'x' + str(i + 1)
        else:
            if output == '':
                output += str(int(exp[i])) + 'x' + str(i + 1)
            else:
                output += ' + ' + str(int(exp[i])) + 'x' + str(i + 1)
    return output


def reverseSign(sign):
    """
    符号取反
    """
    if sign == '>=':
        sign = '<='
    elif sign == '>':
        sign = '<'
    elif sign == '<':
        sign = '>'
    elif sign == '<=':
        sign = '>='
    return sign


class LinearProgramming:
    """
    n: 约束个数
    xn: 决策变量个数
    A: 约束矩阵
    b: 资源限量
    c: 目标函数系数
    minmax: 目标函数最大/最小
    sign: 约束符号
    result: 决策结果
    xresult: 决策结果所对应的决策变量的取值
    """

    def __init__(self):
        self.A = None
        self.b = []
        self.c = []
        self.n = 0
        self.xn = 0
        self.minmax = 0
        self.sign = []
        self.result = 0
        self.value = 0
        self.xresult = None

    def inputConstrainit(self):
        """
        输入线性规划的约束条件
        """
        self.n = int(input("请输入约束个数："))
        self.xn = int(input("请输入决策变量个数："))
        print("约定：大于>，小于<，等于=，大于等于>=，小于等于<=，决策变量x1~x{}>=0".format(self.xn))
        self.A = np.zeros((self.n, self.xn))
        self.xresult = [0  for _ in range(self.xn)]
        for i in range(self.n):
            print("请输入{}个约束式参数：".format(i + 1))
            for j in range(self.xn):
                self.A[i][j] = int(input("约束{}, x{}的系数为：".format(i + 1, j + 1)))
            sign = input("约束{}的符号为：".format(i + 1)).replace(' ', '')
            self.sign.append(sign)
            self.b.append(int(input('约束{}的b为：'.format(i + 1))))

    def inputTarget(self):
        """
        输入目标函数
        """
        choice = input("请输入最大max/最小min：")
        while choice != 'max' and choice != 'min':
            choice = input("输入错误，请重新输入。\n请输入最大max/最小min：")
        if choice == 'max':
            self.minmax = 1
        elif choice == 'min':
            self.minmax = 0
        print("请输入目标函数：")
        for i in range(self.xn):
            self.c.append(int(input("目标函数, x{}的参数为：".format(i + 1))))

    def printExpression(self):
        """
        打印线性规划所有表达式
        """
        print('约束为：')
        for i in range(self.n):
            output = printSingleExpression(self.A[i])
            print(output, end='')
            print(' {} {}'.format(self.sign[i], str(self.b[i])))
        print('约束矩阵为：')
        print(self.A)
        print('目标方程为：')
        print('min z = ' if (self.minmax == 0) else 'max z = ', end='')
        output = printSingleExpression(self.c)
        print(output)

    def transToStd(self):
        """
        转换为标准型
        """
        global M
        # 目标函数化最大
        if self.minmax == 0:
            self.c *= -1
            self.minmax = 1
        # 资源限量非负
        for i in range(self.n):
            if self.b[i] < 0:
                self.A[i] *= -1
                self.b[i] *= -1
                self.sign[i] = reverseSign(self.sign[i])
        # 约束条件化为等式
        for i in range(self.n):
            if self.sign[i] == '>=' or self.sign[i] == '>':
                # 减去剩余变量
                self.xn += 1
                addCol = np.zeros(self.n)
                addCol[i] = -1
                self.A = np.c_[self.A, addCol]
                self.c.append(0)
            elif self.sign[i] == '<=' or self.sign[i] == '<':
                # 加上松弛变量
                self.xn += 1
                addCol = np.zeros(self.n)
                addCol[i] = 1
                self.A = np.c_[self.A, addCol]
                self.c.append(0)
        # 添加人工变量
        for i in range(self.n):
            if self.sign[i] == '>=' or self.sign[i] == '>':
                self.xn += 1
                addCol = np.zeros(self.n)
                addCol[i] = 1
                self.A = np.c_[self.A, addCol]
                self.c.append(-M)
            if self.sign[i] == '=':
                self.xn += 1
                addCol = np.zeros(self.n)
                addCol[i] = 1
                self.A = np.c_[self.A, addCol]
                self.c.append(-M)
            self.sign[i] = '='

    def getC_B(self, x_b):
        """
        基变量对应的价值系数
        """
        c_b = []
        for x in x_b:
            c_b.append(self.c[x])
        return c_b

    def getC_N(self, c_b):
        """
        计算检验数
        """
        c_n = []
        for i in range(self.xn):
            cj = self.c[i]
            for j in range(self.n):
                cj -= c_b[j] * self.A[j, i]
            c_n.append(cj)
        return c_n

    def calcTheta(self, c_n):
        """
        计算theta值
        """
        theta = []
        j = c_n.index(max(c_n))
        for i in range(self.n):
            if self.A[i, j] > 0:
                theta.append(self.b[i] / self.A[i, j])
            else:
                theta.append(float("inf"))
        return theta

    def transBase(self, c_n, theta):
        """
        进行基变换
        """
        j = c_n.index(max(c_n))
        i = theta.index(min(theta))
        main_elem = self.A[i, j]
        self.A[i] = self.A[i] / main_elem
        for num in range(self.n):
            if num != i:
                self.A[num] -= self.A[num][j] * self.A[i]

    @staticmethod
    def transX_B(c_n, theta, x_b):
        """
        进行基的换入与换出
        """
        i = theta.index(min(theta))
        j = c_n.index(max(c_n))
        x_b[i] = j
        return x_b

    def getBestValue(self, c_b):
        """
        计算最优值
        """
        value = 0
        for i in range(self.n):
            value += c_b[i] * self.b[i]
        return value

    def judge(self, c_n, c_b):
        """
        判断解的情况
        """
        count = c_n.count(0)
        if max(c_n) <= 0:
            for i in range(len(c_b)):
                if c_b[i] == -M:
                    if self.b[i] != 0:
                        return 4  # 无可行解
            if count == self.n + 1:
                return 1  # 有最优解
            else:
                return 2  # 有无穷多最优解
        else:
            for i in range(len(c_n)):
                if c_n[i] > 0:
                    lis = []
                    for j in range(self.n):
                        lis.append(self.A[j, i])
                    if max(lis) < 0:
                        for j in range(len(c_b)):
                            if c_b[j] == -M:
                                if self.b[j] != 0:
                                    return 4  # 无可行解
                        return 3  # 有无界解
        return 0  # 继续迭代

    def simplexAlgorithm(self):
        """
        单纯形法解决线性规划问题
        """
        # 确定初始可行基X的下标,x_b存储可行基X的下标
        x_b = []
        for i in range(self.n):
            x_b.append(self.xn - self.n + i)

        # X_B对应的价值系数c_b
        c_b = self.getC_B(x_b)

        # 1.计算检验数cn, theta
        # 2.判断解的情况 若未达到终止条件 找到主元素 改变可行基 进行单位变换  进行第一步
        # 3.若达到终止条件，输出结果
        while True:
            c_n = self.getC_N(c_b)
            theta = self.calcTheta(c_n)
            self.printExcel(c_b, x_b, theta, c_n)
            self.result = self.judge(c_n, c_b)
            if self.result != 0:
                break
            self.transBase(c_n, theta)
            x_b = self.transX_B(c_n, theta, x_b)
            c_b = self.getC_B(x_b)
        self.value = self.getBestValue(c_b)
        self.printResult(x_b)

    def printExcel(self, c_b, x_b, theta, c_n):
        print("%6s" % "", end="")
        print("%6s" % "Cj", end="")
        print("%6s" % "", end="")
        for i in self.c:
            if i != -M:
                print("%6.1f" % i, end="")
            else:
                print("%6s" % "-M", end="")
        print("%6s" % "theta")
        # 第二行
        print("%6s" % "c_b", end="")
        print("%6s" % "x_b", end="")
        print("%6s" % "b", end="")
        for i in range(1, self.xn+1):
            print("%6d" % i, end="")
        print("%6s" % "--")

        # 打印数字
        for i in range(self.n):
            if c_b[i] != -M:
                print("%6.2f" % c_b[i], end="")
            else:
                print("%6s" % "-M", end="")
            print("%6d" % (x_b[i] + 1), end="")
            print("%6s" % self.b[i], end="")
            for j in range(self.xn):
                print("%6.2f" % self.A[i, j], end="")
            print("%6.2f" % theta[i])
        # 打印检验数
        print("%6s" % "", end="")
        print("%6s" % "", end="")
        print("%6s" % "", end="")
        for i in range(self.xn):
            if c_n[i] > 10000 or c_n[i] < -10000:
                print("%6s" % "MM", end="")
            else:
                print("%6.f" % c_n[i], end="")
        print("%6s" % "--")
        print("")

    def printResult(self, x_b):
        """
        打印线性规划决策结果
        """
        if self.result == 1:
            num = 0
            for i in x_b:
                if i <= len(self.xresult):
                    if self.b[num] < 0:
                        print("该问题无可行解:")
                        return
                    self.xresult[i - 1] = self.b[num]
                num += 1
            print("该问题有最优解:")
            print("X=(", end="")
            for i in range(len(self.xresult)):
                if i == len(self.xresult) - 1:
                    print("%.2f" % self.xresult[i], end="")
                else:
                    print("%.2f," % self.xresult[i], end="")
            print(")")
            print("最优值:%.3f" % self.value)
        elif self.result == 2:
            print("该问题有无穷多解,其中一个最优解为:")
            num = 0
            for i in x_b:
                if i <= len(self.xresult):
                    if self.b[num] < 0:
                        return 4
                    self.xresult[i - 1] = self.b[num]
                num += 1
            print("X=(", end="")
            for i in range(len(self.xresult)):
                if i == len(self.xresult) - 1:
                    print("%.0f" % self.xresult[i], end="")
                else:
                    print("%.0f," % self.xresult[i], end="")
            print(")")
            print("最优值:%.3f" % self.value)

        elif self.result == 3:
            print("该问题有无界解")

        elif self.result == 4:
            print("该问题无可行解")


if __name__ == '__main__':
    lp = LinearProgramming()
    print('====================================')
    lp.inputConstrainit()
    lp.inputTarget()
    print('====================================')
    print('转化为标准型前：')
    lp.printExpression()
    print('==========TransForming...===========')
    time.sleep(1)
    lp.transToStd()
    print('转化为标准型并添加人工变量后：')
    lp.printExpression()
    print('==========Calculating...============')
    time.sleep(1)
    lp.simplexAlgorithm()
    print('============== End =================')