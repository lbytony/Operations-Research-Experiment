# -*- coding: UTF-8 -*-
import sympy as sym
import numpy as np

""" 
共轭梯度法求解非线性规划问题
:author: Li Boyan 
:date: 2020/04/29
"""


def inputFunction():
    """
    函数输入
    :returns: 函数表达式, 自变量
    """
    N = int(input("请输入函数的维数(整数)："))
    varlist = ''
    for i in range(N - 1):
        varlist += 'x{} '.format(i + 1)
    varlist += 'x{}'.format(N)
    vars = list(sym.symbols(varlist))
    poly = sym.sympify(input("请输入目标函数(自变量为x1-x{}, 乘方为**)：".format(N)))
    return poly, vars


def haise(poly, vars):
    """
    计算海塞矩阵
    :param poly: 函数表达式
    :param vars: 自变量
    :return:
    """
    H = sym.zeros(len(vars), len(vars))
    for j, r in enumerate(vars):
        for k, s in enumerate(vars):
            H[j, k] = sym.diff(sym.diff(poly, r), s)
    return H


def getGrad(poly, x, vars):
    """
    求在x点处函数的梯度
    :param poly: 函数表达式
    :param x: 自变量取值
    :param vars: 自变量
    :return: 梯度值
    """
    grad = np.zeros(len(x))
    sub = {}
    for i in range(len(vars)):
        sub[vars[i]] = x[i]
    for i in range(len(vars)):
        grad[i] = sym.diff(poly, vars[i]).subs(sub)
    return grad


def main():
    """
    使用共轭梯度法求解非线性规划问题主函数
    """
    print("================= Begin ================")
    poly, vars = inputFunction()
    H = haise(poly, vars)
    x = np.zeros(len(vars))
    grad = getGrad(poly, x, vars)
    p = -1 * grad  # 梯度方向
    i = 1  # 迭代次数
    x = x.T
    while np.linalg.norm(grad, ord=2) > 1e-10:
        lam = -np.dot(grad.T, p) / np.dot(np.dot(p.T, H), p)
        x = x + np.dot(lam, p)
        grad = getGrad(poly, x, vars)
        p = -grad + np.dot(np.dot(np.dot(grad.T, H), p) / np.dot(np.dot(p.T, H), p), p)
        print("========================================")
        print("迭代第{}次范数值为:".format(i), np.linalg.norm(grad, ord=2))
        print("迭代第{}次lambda的值为:\n".format(i), lam)
        print("迭代第{}次梯度值为:\n".format(i), grad)
        print("迭代第{}次P的值为:\n".format(i), p)
        print("迭代第{}次x的值为:\n".format(i), x)
        i = i + 1
    sub = {}
    for i in range(len(vars)):
        sub[vars[i]] = x[i]
    print("最终结果为：当自变量取值为" + str(x) + "时，函数值为" + str(poly.subs(sub)))
    print("================== End =================")


if __name__ == '__main__':
    main()