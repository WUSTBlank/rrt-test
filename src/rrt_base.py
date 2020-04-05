"""
# -*- coding: utf-8 -*-

@Software: PyCharm
@Site:
@File: rrt_base.py
@Author: HBlank
@E-mail: hehaowei@126.com
@Time: 4月 03, 2020
@Des：rrt算法的基本功能函数实现

"""
import numpy as np
import random

from src.tree import Tree
from src.geometry import steer


class RRTBase(object):
    def __init__(self, X, Q, x_init, x_goal, max_samples, r, prc):
        """
        :param X: 搜索空间
        :param Q: 边的长度列表
        :param x_init: 起点
        :param x_goal: 终点
        :param max_samples: 最大采样数
        :param r: 分辨率
        :param prc: 检测概率
        """
        self.X = X  # 搜索空间
        self.Q = Q  # 边的长度列表
        self.x_init = x_init  # 起点
        self.x_goal = x_goal  # 终点
        self.max_samples = max_samples  # 最大采样数
        self.r = r  # 分辨率
        self.prc = prc  # 检测概率
        self.samples_taken = 0
        self.trees = []  # 所有的树列表
        self.add_tree()  # 初始化一棵树

    def add_tree(self):
        """
        初始化一棵空树，并把树放到trees数组里
        """
        self.trees.append(Tree(self.X))

    def add_vertex(self, tree, v):
        """
        添加顶点
        :param tree: int，树的编号
        :param v:tuple,要添加的顶点
        :return:
        """
        self.trees[tree].V.insert(0, v + v, v)
        self.trees[tree].V_count += 1  # 树中顶点数量递增
        self.samples_taken += 1  # 采样数递增

    def add_edge(self, tree, child, parent):
        """
        添加边
        :param tree:  int，树的编号
        :param child: tuple，子节点
        :param parent: tuple,父节点
        :return:
        """
        self.trees[tree].E[child] = parent

    def nearby(self, tree, x, n):
        """
        :param tree: int，树的编号
        :param x:tuple,搜索x周围的顶点
        :param n:int，返回的最大顶点数
        :return:x附近的顶点列表
        """
        return self.trees[tree].V.nearest(x, num_results=n, objects="raw")

    def get_nearest(self, tree, x):
        """
        找到离x最近的顶点
        :param tree: int，树的编号
        :param x:tuple,搜索x周围的顶点
        :return:tuple,距离x最近的顶点
        """
        return next(self.nearby(tree, x, 1))

    def new_and_near(self, tree, q):
        """
        找到要添加的新节点和离他最近的顶点
        :param tree:int，树的编号
        :param q:转向时边的长度
        :return:最新的顶点和距离最近的顶点
        """
        x_rand = self.X.sample_free()
        x_nearest = self.get_nearest(tree, x_rand)
        x_new = self.bound_point(steer(x_nearest, x_rand, q[0]))
        # 检查得到的新节点是否在X_free中并且不在已知节点中
        if not self.trees[tree].V.count(x_new) == 0 or not self.X.obstacle_free(x_new):
            return None, None
        self.samples_taken += 1
        return x_new, x_nearest

    def connect_to_point(self, tree, x_a, x_b):
        """
        把节点添加到树中
        :param tree: int，树的编号
        :param x_a: 父节点
        :param x_b: 子节点，要链接的新节点
        :return: bool,成功为true，失败为false
        """
        if self.trees[tree].V.count(x_b) == 0 and self.X.collision_free(x_a, x_b, self.r):
            self.add_vertex(tree, x_b)
            self.add_edge(tree, x_b, x_a)
            return True
        return False

    def can_connect_to_goal(self, tree):
        """
        确认是否还需要把节点添加到树中
        :param tree: int，树的编号
        :return:bool,达到为true，失败为false
        """
        x_nearest = self.get_nearest(tree, self.x_goal)
        if self.x_goal in self.trees[tree].E and x_nearest in self.trees[tree].E[self.x_goal]:
            return True
        if self.X.collision_free(x_nearest, self.x_goal, self.r):
            return True
        return False

    def get_path(self):
        """
        返回从起点到终点的路径
        :return: 可能的路径，或者None
        """
        if self.can_connect_to_goal(0):
            print("Can connect to goal")
            self.connect_to_goal(0)
            path = self.reconstruct_path(0, self.x_init, self.x_goal)
            return path
        print("Can not connect to goal")

    def connect_to_goal(self, tree):
        """
        把goal点链接到图中
        :param tree:
        :return:
        """
        x_nearest = self.get_nearest(tree, self.x_goal)
        self.trees[tree].E[self.x_goal] = x_nearest

    def reconstruct_path(self, tree, x_init, x_goal):
        """
        从起点到终点的可行路径
        :param tree:树的编号
        :param x_init:起点
        :param x_goal:终点
        :return:可行路径
        """
        path = [x_goal]
        current = x_goal
        if x_init == x_goal:
            return path
        while not self.trees[tree].E[current] == x_init:
            path.append(self.trees[tree].E[current])
            current = self.trees[tree].E[current]
        path.append(x_init)
        path.reverse()
        return path

    def check_solution(self):
        """
        确定找到解决方案
        :return:找到放回true，路径，没找到返回false，None
        """
        if self.prc and random.random() < self.prc:
            print("Check if can connect to goal at", str(self.samples_taken), "samples")
            path = self.get_path()
            if path is not None:
                return True, path
        if self.samples_taken > self.max_samples:
            return True, self.get_path()
        return False, None

    def bound_point(self, point):
        """
        查看是否越界
        :param point:点
        :return:点
        """
        point = np.maximum(point, self.X.dimension_lengths[:, 0])
        point = np.minimum(point, self.X.dimension_lengths[:, 1])
        return tuple(point)
