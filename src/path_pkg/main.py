# encoding=utf-8
import math
# init map size
# width = 100
# height = 100
# def block(mymap):
#     print("Please enter the coordinates of the obstacle, ending with -1")
#     while True:
#         x = int(input("Please enter the x:")) - 1
#         y = int(input("Please enter the y:")) - 1
#         if x >= 0 and x < width and y >= 0 and y < height:
#             mymap[x][y] = 1
#         else:
#             break

def getDistance(now_node, e_node):
    if e_node.x == now_node.x or e_node.y == now_node.y:
        return 1
    else:
        return 2
    # return math.sqrt(pow((e_node.x - now_node.x), 2) + pow((e_node.y - now_node.y), 2))

def ans_path(s_node, G):
    print("path", end=":")

    for item in G:
        if item.x == s_node.x and item.y == s_node.y:
            item.res_out()
    pass

def A_star_algorithm(mymap, s_node, e_node):
    path = []
    width = len(mymap[0])
    height = len(mymap)

    s_node.h = getDistance(s_node, e_node)
    s_node.get_f()  # init start node and get f
    open = [s_node]  # open table
    close = []  # close table

    class find_node:
        def __init__(self, in_x, in_y, in_pre):
            self.x = in_x
            self.y = in_y
            self.pre = in_pre

        def res_out(self):
            path.append(node((self.x + 1), (self.y + 1), None))
            if self.pre != None:
                self.pre.res_out()

    G = [find_node(s_node.x, s_node.y, None)]
    M = []
    while len(open) != 0:
        min_f = open[0].f
        min_node = open[0]


        for item in open:
            if item.f < min_f:
                min_f = item.f
                min_node = item

        open.remove(min_node)
        close.append(min_node)
        min_s_node = find_node(min_node.x, min_node.y, None)

        if min_node.x == e_node.x and min_node.y == e_node.y:
            ans_path(min_s_node, G)  # end
            return

        if min_node.x - 1 >= 0 and mymap[min_node.x - 1][min_node.y] == 0:  # up node
            new_node = node(min_node.x - 1, min_node.y, min_node)
            new_node.g = min_node.g + 1
            new_node.h = getDistance(new_node, e_node)
            new_node.get_f()
            if not (new_node.x == min_node.fa.x and new_node.y == min_node.fa.y):  # son and father diff
                M.append(new_node)
        if min_node.y - 1 >= 0 and mymap[min_node.x][min_node.y - 1] == 0:  # left node
            new_node = node(min_node.x, min_node.y - 1, min_node)
            new_node.g = min_node.g + 1
            new_node.h = getDistance(new_node, e_node)
            new_node.get_f()
            if not (new_node.x == min_node.fa.x and new_node.y == min_node.fa.y):
                M.append(new_node)
        if min_node.x + 1 <= width - 1 and mymap[min_node.x + 1][min_node.y] == 0:  # down node
            new_node = node(min_node.x + 1, min_node.y, min_node)
            new_node.g = min_node.g + 1
            new_node.h = getDistance(new_node, e_node)
            new_node.get_f()
            if not (new_node.x == min_node.fa.x and new_node.y == min_node.fa.y):
                M.append(new_node)
        if min_node.y + 1 <= height - 1 and mymap[min_node.x][min_node.y + 1] == 0:  # right node
            new_node = node(min_node.x, min_node.y + 1, min_node)
            new_node.g = min_node.g + 1
            new_node.h = getDistance(new_node, e_node)
            new_node.get_f()
            if not (new_node.x == min_node.fa.x and new_node.y == min_node.fa.y):
                M.append(new_node)
        if min_node.x - 1 >= 0 and min_node.y - 1 >= 0 and mymap[min_node.x - 1][min_node.y - 1] == 0:  # up left node
            new_node = node(min_node.x - 1, min_node.y - 1, min_node)
            new_node.g = min_node.g + 1
            new_node.h = getDistance(new_node, e_node)
            new_node.get_f()
            if not (new_node.x == min_node.fa.x and new_node.y == min_node.fa.y):  # son and father diff
                M.append(new_node)
        if min_node.x - 1 >= 0 and min_node.y + 1 <= height - 1 and mymap[min_node.x - 1][min_node.y + 1] == 0:  # up right node
            new_node = node(min_node.x - 1, min_node.y + 1, min_node)
            new_node.g = min_node.g + 1
            new_node.h = getDistance(new_node, e_node)
            new_node.get_f()
            if not (new_node.x == min_node.fa.x and new_node.y == min_node.fa.y):  # son and father diff
                M.append(new_node)
        if min_node.x + 1 <= width - 1 and min_node.y - 1 >= 0 and mymap[min_node.x + 1][min_node.y - 1] == 0:  # left down node
            new_node = node(min_node.x + 1, min_node.y - 1, min_node)
            new_node.g = min_node.g + 1
            new_node.h = getDistance(new_node, e_node)
            new_node.get_f()
            if not (new_node.x == min_node.fa.x and new_node.y == min_node.fa.y):  # son and father diff
                M.append(new_node)
        if min_node.x + 1 <= width - 1 and min_node.y + 1 <= height - 1 and mymap[min_node.x + 1][min_node.y + 1] == 0:  # right down node
            new_node = node(min_node.x + 1, min_node.y + 1, min_node)
            new_node.g = min_node.g + 1
            new_node.h = getDistance(new_node, e_node)
            new_node.get_f()
            if not (new_node.x == min_node.fa.x and new_node.y == min_node.fa.y):  # son and father diff
                M.append(new_node)



        for index, item in enumerate(M):
            find = False
            for g_index, g_item in enumerate(G):
                if g_item.x == item.x and g_item.y == item.y:
                    find = True
                    if min_node.g + 1 < item.g:
                        g_item.pre = min_s_node
                        item.g = min_node.g + 1
                        item.get_f()
            if not find:
                open.append(item)  # can't find ：enter open table
                for g_index, g_item in enumerate(G):
                    if g_item.x == min_s_node.x and g_item.y == min_s_node.y:
                        new_search_node = find_node(item.x, item.y, g_item)  # Generating waypoints
                        G.append(new_search_node)

        M.clear()
    print("can't find path")
    return

class node:
    def __init__(self, in_x, in_y, in_fa):
        self.x = in_x
        self.y = in_y
        self.g = 0
        self.h = 0.0
        self.f = self.g + self.h
        self.fa = in_fa

    def get_f(self):
        self.f = self.g + self.h  # compute node f value

    def node_out(self):
        print("(%d,%d),f=%f" % (self.x, self.y, self.f))  # print info
        return

# if __name__=="__main__":
#     # mymap = []
#     # # init mymap width*height and fill 0
#     # for i in range(0, width):
#     #     mymap.append(height * [0])
#     # block(mymap) # init block
#     # print("The map is shown below:")
#     # print(mymap)
#     # sx = int(input("Please enter the start x:")) - 1
#     # sy = int(input("Please enter the start y:")) - 1
#     # s_node = node(sx, sy, None)
#     # s_node.fa = s_node
#     # ex = int(input("Please enter the end x:")) - 1
#     # ey = int(input("Please enter the end y:")) - 1
#     # e_node = node(ex, ey, None)

#     A_star_algorithm(mymap, s_node, e_node)
#     # print ans
#     for i in range(1, len(path) + 1):
#         if i == len(path):
#             print("({0},{1})".format(path[-i].x, path[-i].y), end = "")
#         else:
#             print("({0},{1})->".format(path[-i].x, path[-i].y), end = "")
    
#     # k=input("\npress close to exit")
