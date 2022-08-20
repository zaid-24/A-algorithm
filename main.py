
import math
import matplotlib.pyplot as plt

class AStarPlanner:
    def __init__(self, obstacle_x, obstacle_y, size, radius):
        # obstacle_x:list of Obstacles in x direction
        # obstacle_y:list of Obstacles in y direction
        # radius: robot radius
        # size: grid size
        # initializing min and max x and y as 0
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.x_width, self.y_width = 0, 0
        self.size = size
        self.radius = radius
        self.obstacle_map = None
        self.motion = self.get_motion_model()
        self.map_calculation(obstacle_x, obstacle_y)
    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x 
            self.y = y
            self.cost = cost
            self.parent_index = parent_index
        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, start_x, start_y, goal_x, goal_y):
        start_node = self.Node(self.indexes(start_x, self.min_x),
                               self.indexes(start_y, self.min_y), 0.0, -1)
        goal_node = self.Node(self.indexes(goal_x, self.min_x),
                              self.indexes(goal_y, self.min_y), 0.0, -1)
        open_list, closed_list = dict(), dict()
        open_list[self.grid_index(start_node)] = start_node
        while 1:
            if len(open_list) == 0:
                print("The goal position do not lies within border")
                break
            c_id = min(
                open_list,
                key=lambda o: open_list[o].cost + self.calculate_heu(goal_node,
                                                                     open_list[
                                                                         o]))
            cur = open_list[c_id]
            #displaying graph
            plt.plot(self.position_in_grid(cur.x, self.min_x),self.position_in_grid(cur.y, self.min_y), ".g")
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
            if len(closed_list.keys()) % 10 == 0:
                plt.pause(0.001)
            if cur.x == goal_node.x and cur.y == goal_node.y:
                print("Algortihm completed")
                goal_node.parent_index = cur.parent_index
                goal_node.cost = cur.cost
                break
            # Remove the item from the open list and then adding it to closed list
            del open_list[c_id]
            closed_list[c_id] = cur
            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(cur.x + self.motion[i][0],
                                 cur.y + self.motion[i][1],
                                 cur.cost + self.motion[i][2], c_id)
                nodeInd = self.grid_index(node)
                if not self.checking_node(node):
                    continue
                # If the node is not right to go then don't move there
                if nodeInd in closed_list:
                    continue
                if nodeInd not in open_list:
                    open_list[nodeInd] = node  
                else:
                    if open_list[nodeInd].cost > node.cost:
                        open_list[nodeInd] = node
        x_path, y_path = self.calc_final_path(goal_node, closed_list)
        # x_path: x position list of the final path
        # y_path: y position list of the final path
        return x_path, y_path

    def calc_final_path(self, goal_node, closed_list):
        # generate final course
        x_path, y_path = [self.position_in_grid(goal_node.x, self.min_x)], [
            self.position_in_grid(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_list[parent_index]
            x_path.append(self.position_in_grid(n.x, self.min_x))
            y_path.append(self.position_in_grid(n.y, self.min_y))
            parent_index = n.parent_index
        return x_path, y_path

    def position_in_grid(self, index, min_position):
        pos = index * self.size + min_position
        return pos
    def grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)
    @staticmethod
    def calculate_heu(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d
    def indexes(self, position, min_pos):
        return round((position - min_pos) / self.size)

    def checking_node(self, node):
        px = self.position_in_grid(node.x, self.min_x)
        py = self.position_in_grid(node.y, self.min_y)
        if px < self.min_x:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False    
        elif py < self.min_y:
            return False
        # checking for collision
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def map_calculation(self, obstacle_x, obstacle_y):
        self.min_x = round(min(obstacle_x))
        self.min_y = round(min(obstacle_y))
        self.max_x = round(max(obstacle_x))
        self.max_y = round(max(obstacle_y))
        self.x_width = round((self.max_x - self.min_x) / self.size)
        self.y_width = round((self.max_y - self.min_y) / self.size)
        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.position_in_grid(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.position_in_grid(iy, self.min_y)
                for iobstacle_x, iobstacle_y in zip(obstacle_x, obstacle_y):
                    d = math.hypot(iobstacle_x - x, iobstacle_y - y)
                    if d <= self.radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion

def main():
    with open('queries.txt') as f:
        line = f.readlines()
    for queries in line:
        # print(queries[0],queries[2],queries[4])
        a=queries.split()
        start_x = int(a[0])
        start_y = int(a[1])
        goal_x = int(a[2])
        goal_y = int(a[3])
        grid_size = 1
        robot_radius = 1.0
        # set obstacle positions
        obstacle_x, obstacle_y = [], []
        for i in range(0, 60):
            obstacle_x.append(i)
            obstacle_y.append(0)
        for i in range(0, 40):
            obstacle_x.append(60.0)
            obstacle_y.append(i)
        for i in range(0, 61):
            obstacle_x.append(i)
            obstacle_y.append(40.0)
        for i in range(0, 41):
            obstacle_x.append(0)
            obstacle_y.append(i)
        # obstacles co-ordinates
        for i in range(20, 30):
            obstacle_x.append(20.0)
            obstacle_y.append(i)
        for i in range(20, 31):
            obstacle_x.append(30.0)
            obstacle_y.append(i)
        for i in range(20, 30):
            obstacle_x.append(i)
            obstacle_y.append(20)
        for i in range(20, 30):
            obstacle_x.append(i)
            obstacle_y.append(30)

        plt.plot(obstacle_x, obstacle_y, ".b")
        plt.plot(start_x, start_y, ".g")
        plt.plot(goal_x, goal_y, ".g")
        plt.grid(True)
        plt.axis("equal")

        a_star = AStarPlanner(obstacle_x, obstacle_y, grid_size, robot_radius)
        x_path, y_path = a_star.planning(start_x, start_y, goal_x, goal_y)
        plt.plot(x_path, y_path, "-r")
        plt.pause(0.001)
        plt.show()
if __name__ == '__main__':
    main()
