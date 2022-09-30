import Map as mp
import numpy as np
from typing import Union
import collections
import heapq

class node:
    def __init__(self, prevnode=None, g=None, cord=[None, None], f=None):
        self.g = g
        self.prevnode = prevnode
        self.cord = cord
        self.f = f

    def __eq__(self, other):
        return self.cord[0] == other.cord[0] and self.cord[1] == other.cord[1]

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f

class SLinkedList:
    def __init__(self):
        self.tail = None

# Uses Manhattan distance
def getf(goal_position, curent_node: node):
    return curent_node.g + abs(goal_position[0] - curent_node.cord[0]) + abs(goal_position[1] - curent_node.cord[1])

def getCost(pos, int_map):
        return int_map[pos[0],pos[1]]


def astar_edgar_full(themap: mp.Map_Obj):
    openQueue = []
    closedQueue = []
    heapq.heapify(openQueue)
    int_map, str_map = themap.get_maps()

    # Create start and end nodes
    start_node = node(None, g=0, f=0, cord=themap.get_start_pos())
    start_node.f = getf(themap.get_goal_pos(), start_node)
    end_node = node(None, 0, cord=[themap.get_goal_pos()[0], themap.get_goal_pos()[1]])

    heapq.heappush(openQueue, start_node)
    # Loop with A* until the goal node is found
    while len(openQueue) > 0:

        # My original solution for openqueue management, before switching to heapq found on the internet
        '''
        current = openQueue[0]
        index_current = 0
        for index, newNode in enumerate(openQueue):
            if newNode.f < current.f:
                current = newNode
                index_current = index

        # Graduate current node to closedQueue
        openQueue.pop(index_current)
        closedQueue.append(current)
        '''

        # Saw this heapq thing on the internet, checking if works better
        current = heapq.heappop(openQueue)
        closedQueue.append(current)

        # If the current is the goal
        if current.cord[0] == end_node.cord[0] and current.cord[1] == end_node.cord[1]:
            themap.show_map(str_map)
            print("Cost of path-edgar_full: ", current.g)
            while current.g != 0:
                themap.paint_box_finished(current.cord, str_map)
                current = current.prevnode
            return themap, int_map, str_map

        # Get the children of current node
        for new_pos in [[0, 1], [1, 0], [1, 1], [0, -1], [-1, 0], [-1, -1], [1, -1], [-1, 1]]:
            pos = [(current.cord[0] + new_pos[0]), (current.cord[1] + new_pos[1])]
            # pos = np.add(current.cord, new_pos)

            # Make sure its walkable
            if int_map[pos[0]][pos[1]] == -1:
                continue

            if getCost(pos, int_map) == 4:
                continue

            child = node(prevnode=current, g=(current.g + getCost(pos=pos, int_map=int_map)), cord=pos)
            child.f = getf(end_node.cord, child)

            # Check if already looked at
            is_in_closed = False
            for existing_child in closedQueue:
                if collections.Counter(child.cord) == collections.Counter(existing_child.cord) and getf(
                        themap.get_goal_pos(), child) > getf(themap.get_goal_pos(), child_in_queue):
                    is_in_closed = True
            if is_in_closed: continue

            # Check if in queue already, and if the current path is shorter
            is_in_open = False
            for child_in_queue in openQueue:
                if collections.Counter(child.cord) == collections.Counter(child_in_queue.cord) and getf(
                        themap.get_goal_pos(), child) > getf(themap.get_goal_pos(), child_in_queue):
                    is_in_open = True
            if is_in_open:
                continue

            # Child is already in the open list
            if len([open_node for open_node in openQueue if (child.cord[0] == open_node.cord[0] and child.cord[1] ==
                                                             open_node.cord[1]) and child.g >= open_node.g]) > 0:
                continue

            # Child is already in the closed list
            if len([open_node for open_node in closedQueue if (child.cord[0] == open_node.cord[0] and child.cord[1] ==
                                                               open_node.cord[1]) and child.g > open_node.g]) > 0:
                continue

            # if count % 25 == 0: themap.show_map(str_map)
            themap.paint_box(pos, str_map)
            heapq.heappush(openQueue, child)

    return themap, int_map, str_map


def astar_custom(themap: mp.Map_Obj):
    openQueue = []
    closedQueue = []
    heapq.heapify(openQueue)
    int_map, str_map = themap.get_maps()

    # Create start and end nodes
    start_node = node(None, g=0, f=0, cord=themap.get_start_pos())
    start_node.f = getf(themap.get_goal_pos(), start_node)
    end_node = node(None, 0, cord=[themap.get_goal_pos()[0], themap.get_goal_pos()[1]])

    heapq.heappush(openQueue, start_node)
    count = 0
    # Loop with A* until the goal node is found
    while len(openQueue) > 0:

        # Saw this heapq thing on the internet, checking if works better
        current = heapq.heappop(openQueue)
        closedQueue.append(current)

        # If the current is the goal
        if current.cord[0] == end_node.cord[0] and current.cord[1] == end_node.cord[1]:
            themap.show_map(str_map)
            print("Cost of path-custom: ", current.g)
            while current.g != 0:
                themap.paint_box_finished(current.cord, str_map)
                current = current.prevnode
            return themap, int_map, str_map

        # Get the children of current node
        for new_pos in [[0, 1], [1, 0], [1, 1], [0, -1], [-1, 0], [-1, -1], [1, -1], [-1, 1]]:
            pos = [(current.cord[0] + new_pos[0]), (current.cord[1] + new_pos[1])]
            # pos = np.add(current.cord, new_pos)

            # Make sure its walkable
            if int_map[pos[0]][pos[1]] == -1:
                continue

            child = node(prevnode=current, g=(current.g + getCost(pos=pos, int_map=int_map)), cord=pos)
            child.f = getf(end_node.cord, child)

            # Check if already looked at
            is_in_closed = False
            for existing_child in closedQueue:
                if collections.Counter(child.cord) == collections.Counter(existing_child.cord) and getf(
                        themap.get_goal_pos(), child) > getf(themap.get_goal_pos(), child_in_queue):
                    is_in_closed = True
            if is_in_closed: continue

            # Check if in queue already, and if the current path is shorter
            is_in_open = False
            for child_in_queue in openQueue:
                if collections.Counter(child.cord) == collections.Counter(child_in_queue.cord) and getf(
                        themap.get_goal_pos(), child) > getf(themap.get_goal_pos(), child_in_queue):
                    is_in_open = True
            if is_in_open:
                continue

            # Child is already in the open list
            if len([open_node for open_node in openQueue if (child.cord[0] == open_node.cord[0] and child.cord[1] ==
                                                             open_node.cord[1]) and child.g >= open_node.g]) > 0:
                continue

            # Child is already in the closed list
            if len([open_node for open_node in closedQueue if (child.cord[0] == open_node.cord[0] and child.cord[1] ==
                                                               open_node.cord[1]) and child.g > open_node.g]) > 0:
                continue

            # Paint the searched node and add it to openQueue
            themap.paint_box(pos, str_map)
            heapq.heappush(openQueue, child)

    return themap, int_map, str_map

def astar(themap: mp.Map_Obj):
    openQueue = []
    closedQueue = []
    heapq.heapify(openQueue)
    int_map, str_map = themap.get_maps()

    # Create start and end nodes
    start_node = node(None, g=0, f=0, cord=themap.get_start_pos())
    start_node.f = getf(themap.get_goal_pos(), start_node)
    end_node = node(None, 0, cord=[themap.get_goal_pos()[0], themap.get_goal_pos()[1]])

    heapq.heappush(openQueue, start_node)
    count = 0
    # Loop with A* until the goal node is found
    while len(openQueue) > 0:
        # Saw this heapq thing on the internet, checking if works better
        current = heapq.heappop(openQueue)
        closedQueue.append(current)

        # If the current is the goal
        if current.cord[0] == end_node.cord[0] and current.cord[1] == end_node.cord[1]:
            print("Cost of path-normal: ", current.g)
            themap.show_map(str_map)
            while current.g != 0:
                themap.paint_box_finished(current.cord ,str_map)
                current = current.prevnode
            return themap, int_map, str_map

        # Get the children of current node
        for new_pos in [[0,1], [1,0], [1,1], [0,-1], [-1,0], [-1, -1], [1, -1], [-1, 1]]:
            pos = [(current.cord[0] + new_pos[0]), (current.cord[1] + new_pos[1])]
            #pos = np.add(current.cord, new_pos)

            # Make sure its walkable
            if int_map[pos[0]][pos[1]] == -1:
                continue

            child = node(prevnode=current, g=(current.g+1), cord=pos)
            child.f = getf(end_node.cord, child)

            # Check if already looked at
            is_in_closed = False
            for existing_child in closedQueue:
                if collections.Counter( child.cord) == collections.Counter( existing_child.cord) and getf(themap.get_goal_pos(), child) > getf(themap.get_goal_pos(), child_in_queue):
                    is_in_closed = True
            if is_in_closed: continue

            # Check if in queue already, and if the current path is shorter
            is_in_open = False
            for child_in_queue in openQueue:
                if collections.Counter(child.cord) == collections.Counter( child_in_queue.cord) and getf(themap.get_goal_pos(), child) > getf(themap.get_goal_pos(), child_in_queue):
                    is_in_open = True
            if is_in_open:
                continue

            # Child is already in the open list
            if len([open_node for open_node in openQueue if (child.cord[0] == open_node.cord[0] and child.cord[1] == open_node.cord[1]) and child.g >= open_node.g]) > 0:
                continue

            # Child is already in the closed list
            if len([open_node for open_node in closedQueue if (child.cord[0] == open_node.cord[0] and child.cord[1] == open_node.cord[1]) and child.g > open_node.g]) > 0:
                continue

            # Paint the searched node and add it to openQueue
            themap.paint_box(pos, str_map)
            heapq.heappush(openQueue ,child)

    return themap, int_map, str_map

def task31():
    map1 = mp.Map_Obj()
    data1, data2 = map1.read_map(path="Samfundet_map_1.csv")

    mapObj, newIntMap, themap = astar(map1)

    mapObj.show_map(themap)

def task32():
    map2 = mp.Map_Obj(task=2)

    data1, data2 = map2.read_map(path="Samfundet_map_1.csv")

    mapObj, newIntMap, finishedMap = astar(map2)

    mapObj.show_map(finishedMap)

def task41():
    map3 = mp.Map_Obj(task=3)
    data1, data2 = map3.read_map(path="Samfundet_map_2.csv")

    mapObj, newIntMap, finishedMap = astar_custom(map3)

    mapObj.show_map(finishedMap)

def task42():
    map4 = mp.Map_Obj(task=4)
    intMap, strMap = map4.read_map(path="Samfundet_map_Edgar_full.csv")
    mapObj, newIntMap, finishedMap = astar_edgar_full(map4)
    map4.show_map(finishedMap)

def main():
    task31()
    task32()
    task41()
    task42()

if __name__ == "__main__":
    main()