"""
-*- coding: utf-8 -*-

@Software: PyCharm
@Site: 
@File: rrt.py
@Author: HBlank
@E-mail: hehaowei@126.com
@Time: 4月 03, 2020
@Des：实现rrt

"""
from src.rrt_base import RRTBase


class RRT(RRTBase):
    def __init__(self, X, Q, x_init, x_goal, max_samples, r, prc):
        super().__init__(X, Q, x_init, x_goal, max_samples, r, prc)

    def rrt_search(self):
        self.add_vertex(0, self.x_init)
        self.add_edge(0, self.x_init, None)

        while True:
            for q in self.Q:
                for i in range(q[1]):

                    x_new, x_nearest = self.new_and_near(0, q)

                    if x_new == None:
                        continue
                    self.connect_to_point(0, x_nearest, x_new)

                    solution = self.check_solution()
                    if solution[0]:
                        return solution[1]
