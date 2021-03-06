import matplotlib.pyplot as plt
import random
import math
import copy
import operator
import matplotlib.animation as animation
import numpy as np


class Node(object):
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT_connect(object):
    """
    Class for RRT_connect Planning
    """

    def __init__(self, initial, goal, obstacle_list, rand_area):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:random sampling Area [min,max]

        """
        self.initial = Node(initial[0], initial[1])
        self.goal = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.step = 0.5  # velocity
        self.goalSampleRate = 0.5  # 选择终点的概率
        self.maxIter = 5000
        self.obstacleList = obstacle_list
        self.RRT_a = [self.initial]
        self.RRT_b = [self.goal]

    def random_samping(self):
        if random.random() > self.goalSampleRate:
            qrand = self.random_node()
        else:
            qrand = self.goal
        return qrand

    def connect(self, node_list, qrand):
        while True:
            qnew, S = self.extend(node_list, qrand)
            if S != "Advanced":
                break
        return S

    def extend(self, node_list, qrand):
        qnear = self.nearset_neighbor(node_list, qrand)
        qnew = self.new_config(node_list, qnear, qrand)
        if qnew is not None:
            if self.Euclidean_distance(qnew, qrand) < self.step:
                S = "Reached"
            else:
                S = "Advanced"
        else:
            S = "Trapped"
        self.draw_graph(qrand)
        return qnew, S

    def new_config(self, node_list, qnear, qrand):
        theta = math.atan2(qrand.y - qnear.y, qrand.x - qnear.x)
        qnew = copy.deepcopy(qnear)
        qnew.x += self.step * math.cos(theta)
        qnew.y += self.step * math.sin(theta)
        qnew.parent = qnear
        if self.collision_check(self.obstacleList, qnew):
            node_list.append(qnew)
        else:
            qnew = None
        return qnew

    def random_node(self):
        """
        产生随机节点
        :return:
       """
        node_x = random.uniform(self.min_rand, self.max_rand)
        node_y = random.uniform(self.min_rand, self.max_rand)
        node = Node(node_x, node_y)

        return node

    def nearset_neighbor(self, node_list, qrand):
        """
        :param node_list:
        :param qrand:
        :return:
        """
        d_list = [(node.x - qrand.x) ** 2 + (node.y - qrand.y) ** 2 for node in node_list]  # euclidean distance
        min_index = d_list.index(min(d_list))
        return node_list[min_index]

    def Euclidean_distance(self, node1, node2):
        dx = node1.x - node2.x
        dy = node1.y - node2.y
        d = math.sqrt(dx ** 2 + dy ** 2)
        return d

    def distance_line(self, node, initial, goal):
        d = 0
        if initial.x == goal.x:
            if (node.y - initial.y) * (node.y - goal.y) < 0:
                d = abs(node.x - initial.x)
        elif initial.y == goal.y:
            if (node.x - initial.x) * (node.x - goal.x) < 0:
                d = abs(node.y - initial.y)
        else:
            k = (initial.y - goal.y) / (initial.x - goal.x)
            b = initial.y - k * initial.x
            if (-1 / k * node.x - node.y + 1 / k * goal.x + goal.y) * (
                    -1 / k * node.x - node.y + 1 / k * initial.x + initial.y) < 0:
                d = abs((k * node.x - node.y + b) / math.sqrt(k ** 2 + 1))
            else:
                ds = self.Euclidean_distance(node, initial)
                de = self.Euclidean_distance(node, goal)
                d = min(ds, de)
        return d

    def collision_check(self, obstacle_list, initial, goal=None):
        a = 1
        # collision check for node
        for (ox, oy, size) in obstacle_list:
            dx = ox - initial.x
            dy = oy - initial.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                a = 0  # collision
            # collision check for line segment
            if goal is not None:
                d2 = self.distance_line(Node(ox, oy), initial, goal)
                if d2 <= size:
                    a = 0
        return a  # safe

    def planning(self):
        """
        Path planning

        animation: flag for animation on or off
        """

        while True:
            # Random Sampling
            qrand = self.random_samping()
            # extend tree
            qnew, S = self.extend(self.RRT_a, qrand)
            if S != "Trapped":
                if self.connect(self.RRT_b, qnew) == "Reached":
                    path_goal = []
                    node = self.RRT_b[-1]
                    while node.parent is not None:
                        path_goal.append([node.x, node.y])
                        node = node.parent
                    path_goal.append([self.goal.x, self.goal.y])
                    path_initial = []
                    node = self.RRT_a[-1]
                    while node.parent is not None:
                        path_initial.append([node.x, node.y])
                        node = node.parent
                    path_initial.append([self.initial.x, self.initial.y])
                    path_initial.reverse()
                    path = path_initial + path_goal
                    return path
            self.swap()

    def swap(self):
        a = self.RRT_a
        self.RRT_a = self.RRT_b
        self.RRT_b = a

    def short_cut(self, path):
        P = path[0]
        new_path = [P]
        i = 1
        while not operator.eq(path[i], [self.initial.x, self.initial.y]):
            # while [P,path[i+1]] collision free and i+1<len(path):
            while self.collision_check(self.obstacleList, Node(P[0], P[1]), Node(path[i + 1][0], path[i + 1][1])):
                i = i + 1
                if i + 1 > len(path) - 1:
                    break
            new_path.append(path[i])
            P = path[i]
            i = i + 1
            if i > len(path) - 1:
                break
        return new_path

    def draw_graph(self, qrand=None):
        """
        Draw Graph
        """
        plt.clf()  # 清除上次画的图
        if qrand is not None:
            plt.plot(qrand.x, qrand.y, "^g")
        for node in self.RRT_a:
            if node.parent is not None:
                plt.plot([node.x, node.parent.x], [
                    node.y, node.parent.y], "-g")
        for node in self.RRT_b:
            if node.parent is not None:
                plt.plot([node.x, node.parent.x], [
                    node.y, node.parent.y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, marker='o', color='black', ms=44 * size)

        plt.plot(self.initial.x, self.initial.y, "^r")
        plt.plot(self.goal.x, self.goal.y, "^b")
        plt.axis('equal')
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)

    def draw_static(self, path, new_path):
        """
        画出静态图像
        :return:
        """
        plt.clf()  # 清除上次画的图

        for node in self.RRT_a:
            if node.parent is not None:
                plt.plot([node.x, node.parent.x], [
                    node.y, node.parent.y], "-g")
        for node in self.RRT_b:
            if node.parent is not None:
                plt.plot([node.x, node.parent.x], [
                    node.y, node.parent.y], "-r")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, marker="o", color='black', ms=44 * size)

        plt.plot(self.initial.x, self.initial.y, "^r")
        plt.plot(self.goal.x, self.goal.y, "^b")
        plt.axis('equal')
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])

        plt.plot([data[0] for data in path], [data[1] for data in path], '-r', label='path')
        plt.plot([data[0] for data in new_path], [data[1] for data in new_path], '-b', label='short cut')
        plt.grid(True)
        plt.legend()
        plt.show()


def main():
    print("start RRT_connect path planning")
    initial = [0, 0]
    goal = [9, 9]
    n = 10  # number of obstacle
    obstacle_list = []
    for i in range(n):
        # random.seed(i)
        x = random.uniform(initial[0], goal[0])
        y = random.uniform(initial[1], goal[1])
        radius = random.uniform(0.5, 1)
        # x, y, radius = 4, 4, 2
        obstacle_list.append((x, y, radius))

    # Set Initial parameters
    rrt = RRT_connect(initial, goal, rand_area=[-2, 10], obstacle_list=obstacle_list)
    path = rrt.planning()
    # print(path)
    new_path = rrt.short_cut(path)
    # Draw final path
    plt.close()
    rrt.draw_static(path, new_path)

    # print(new_path)


if __name__ == '__main__':
    main()
