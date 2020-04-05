"""
-*- coding: utf-8 -*-

@Software: PyCharm
@Site: 
@File: tree.py
@Author: HBlank
@E-mail: hehaowei@126.com
@Time: 4月 03, 2020
@Des：

"""
from rtree import index


class Tree(object):
    def __init__(self, X):
        """
        树的操作
        :param X: Search Space
        """
        p = index.Property()
        p.dimension = X.dimensions
        self.V = index.Index(interleaved=True, properties=p)  # vertices in an rtree
        self.V_count = 0
        self.E = {}  # edges in form E[child] = parent
