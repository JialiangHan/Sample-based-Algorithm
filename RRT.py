import matplotlib.pyplot as plt
import random
import math
import copy
import operator



class Node(object):
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT(object):
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacle_list, rand_area):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:random sampling Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expandDis = 0.5 # velocity
        self.goalSampleRate = 0.5 # 选择终点的概率
        self.maxIter = 5000
        self.obstacleList = obstacle_list
        self.nodeList = [self.start]

    def random_node(self):
        """
        产生随机节点
        :return:
       """
        node_x = random.uniform(self.min_rand, self.max_rand)
        node_y = random.uniform(self.min_rand, self.max_rand)
        node = [node_x, node_y]

        return node

    @staticmethod
    def get_nearest_list_index(node_list, rnd):
        """
        :param node_list:
        :param rnd:
        :return:
        """
        d_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list] # euclidean distance
        min_index = d_list.index(min(d_list))
        return min_index

    def Euclidean_distance(self, node1, node2):
        dx = node1[0] - node2[0]
        dy = node1[1] - node2[1]
        d = math.sqrt(dx ** 2 + dy ** 2)
        return d

    def distance_line(self, node, start, end):
        d = 0
        if start[0] == end[0]:
            if (node[1] - start[1]) * (node[1] - end[1]) < 0:
                d = abs(node[0] - start[0])
        elif start[1] == end[1]:
            if (node[0] - start[0]) * (node[0] - end[0]) < 0:
                d = abs(node[1] - start[1])
        else:
            k = (start[1] - end[1]) / (start[0] - end[0])
            b = start[1] - k * start[0]
            if (-1 / k * node[0] - node[1] + 1 / k * end[0] + end[1]) * (
                    -1 / k * node[0] - node[1] + 1 / k * start[0] + start[1]) < 0:
                d = abs((k * node[0] - node[1] + b) / math.sqrt(k ** 2 + 1))
            else:
                ds = self.Euclidean_distance(node, start)
                de = self.Euclidean_distance(node, end)
                d = min(ds, de)
        return d

    def collision_check(self, obstacle_list, start, end=None):
        a = 1
        # collision check for node
        for (ox, oy, size) in obstacle_list:
            dx = ox - start[0]
            dy = oy - start[1]
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                a = 0 # collision
            # collision check for line segment
            if end is not None:
                d2 = self.distance_line([ox, oy], start, end)
                if d2 <= size:
                    a = 0
        return a # safe

    def planning(self):
        """
        Path planning

        animation: flag for animation on or off
        """

        while True:
            # Random Sampling
            if random.random() > self.goalSampleRate:
                rnd = self.random_node()
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            min_index = self.get_nearest_list_index(self.nodeList, rnd)
            # print(min_index)

            # expand tree
            nearest_node = self.nodeList[min_index]

            # 返回弧度制
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)

            new_node = copy.deepcopy(nearest_node)
            new_node.x += self.expandDis * math.cos(theta)
            new_node.y += self.expandDis * math.sin(theta)
            new_node.parent = min_index

            if not self.collision_check(self.obstacleList, [new_node.x, new_node.y]):
                continue

            self.nodeList.append(new_node)

            # check goal
            dx = new_node.x - self.end.x
            dy = new_node.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if True:
                self.draw_graph(rnd)

        path = [[self.end.x, self.end.y]]
        last_index = len(self.nodeList) - 1
        while self.nodeList[last_index].parent is not None:
            node = self.nodeList[last_index]
            path.append([node.x, node.y])
            last_index = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def short_cut(self, path):
        P = path[0]
        new_path = [P]
        i = 1
        while not operator.eq(path[i], [self.start.x, self.start.y]):
            # while [P,path[i+1]] collision free and i+1<len(path):
            while self.collision_check(self.obstacleList, P, path[i + 1]):
                i = i + 1
                if i + 1 > len(path) - 1:
                    break
            new_path.append(path[i])
            P = path[i]
            i = i + 1
            if i > len(path)-1 :
                break
        return new_path

    def draw_graph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf() # 清除上次画的图
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^g")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, marker='o', color='black',ms=44 * size)

        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        plt.axis('equal')
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)

    def draw_static(self, path,new_path):
        """
        画出静态图像
        :return:
        """
        plt.clf() # 清除上次画的图

        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, marker="o", color='black',ms=44 * size)

        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        plt.axis('equal')
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])

        plt.plot([data[0] for data in path], [data[1] for data in path], '-r',label='path')
        plt.plot([data[0] for data in new_path], [data[1] for data in new_path], '-b',label='short cut')
        plt.grid(True)
        plt.legend()
        plt.show()


def main():
    print("start RRT path planning")
    start = [0, 0]
    goal = [9, 9]
    n = 10 # number of obstacle
    obstacle_list = []
    for i in range(n):
        # random.seed(i)
        x = random.uniform(start[0], goal[0])
        y = random.uniform(start[1], goal[1])
        radius = random.uniform(0.5, 1)
        # x, y, radius = 4, 4, 2
        obstacle_list.append((x, y, radius))

    # Set Initial parameters
    rrt = RRT(start, goal, rand_area=[-2, 10], obstacle_list=obstacle_list)
    path = rrt.planning()
    # print(path)
    new_path = rrt.short_cut(path)
    # Draw final path
    plt.close()
    rrt.draw_static(path,new_path)


    # print(new_path)



if __name__ == '__main__':
    main()
