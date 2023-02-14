import cv2
import random
import numpy as np
from collections import deque
from queue import Queue
from helper_functions import get_adj_list, get_unvisited_child_nodes, get_weighted_adj_list, color_dict


class Planner():

    def __init__(self, map, start, end, max_iterations=10000):
        """
        Performs specified path planning algorithm on a grid map to find path between a starting and an ending point.

        Parameters
        ----------
        map : array_like
           Map of the environment.
        start : tuple of ints
           Starting point in the map.
        end : tuple of ints
           Ending point in the map.
        max_iterations : int
           Maximum number of iterations for the random planner

        """

        self.map = map
        self.start = start
        self.end = end
        self.max_iterations = max_iterations
    

    def dfs(self, showImage = False):
        """
        Performs Depth-First Search algorithm on a grid map to find path between a starting and an ending point.

        Returns
        -------
        list
            The path found by using the algorithm.
        """
        # tracking of nodes in depth first search is done using a stack data structure
        # using deque class from collections library to implement a stack
        dfs_stack = deque()
        visited = {}
        parent_point = {}

        # get the information regarding the neighbours of all the points in the map
        adj_list = get_adj_list(self.map)

        for point in adj_list.keys():
            # print(point, ': ', adj_list[point])
            visited[point] = False
            parent_point[point] = None

        current_point = self.start
        visited[current_point] = True
        dfs_stack.append(current_point)

        # declaring the child variable to avoid early reference error
        child = None
        count = 0 # for visualization
        while dfs_stack:
            child_list = get_unvisited_child_nodes(adj_list, current_point, visited)

            # set the last element in stack as current point if no unvisited children
            if len(child_list) == 0:
                current_point = dfs_stack.pop()
            
            for child in child_list:
                
                # if child already visited, move on to next child node
                if visited[child]:
                    continue
                
                # check if the child node is the end goal
                if child == self.end:
                    visited[child] = True
                    parent_point[child] = current_point
                    dfs_stack.clear()
                    break
                
                visited[child] = True
                parent_point[child] = current_point
                dfs_stack.append(child)
                current_point = child   

                self.map[child] = color_dict['traversal']
                if showImage:
                    if count % 15 == 0:
                        map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                        cv2.imshow('result', map_resized) 
                        cv2.waitKey(1)
        
                break

            count += 1

        # check if a path was found betweeen start and end positions
        if child != self.end:
            print("\nNo path found between start and end positions!\n")
            if showImage:
                map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized)         
                cv2.waitKey()
            return 0, 0
        else:
            print('Path found!\n')
            if showImage:
                map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized)         
                cv2.waitKey()
                cv2.destroyAllWindows()

        # now the path can be obtained by starting from end goal and tracing back to start through the parent_point nodes
        dfs_path = []
        curr_point = self.end
        while curr_point != self.start:
            parent_pt = parent_point[curr_point]
            dfs_path.append(parent_pt)
            curr_point = parent_pt
        
        return dfs_path[:-1][::-1], count


    def bfs(self, showImage = False):
        """
        Performs Breadth-First Search algorithm on a grid map to find path between a starting and an ending point.

        Returns
        -------
        list
            The path found by using the algorithm.
        """
        # tracking of nodes in breadth first search is done using a queue data structure
        bfs_queue = Queue()
        visited = {}
        parent_point = {}
        level = {}
        # get the information regarding the neighbours of all the points in the map
        adj_list = get_adj_list(self.map)


        for point in adj_list.keys():
            # print(point, ': ', adj_list[point])
            visited[point] = False
            parent_point[point] = None
            level[point] = -1
            
        visited[self.start] = True
        level[self.start] = 0
        bfs_queue.put(self.start)

        # declaring the child variable to avoid early reference error
        child = None
        count = 0 # for visualization
        while not bfs_queue.empty():
            
            current_point = bfs_queue.get()
            for child in adj_list[current_point]:
                # check if the child node is already visited (skip the child node if already visited)
                if visited[child]:
                    continue

                # if the child node is not already visited
                else:
                    # check if the the child node is the end goal
                    if child == self.end:
                        visited[child] = True
                        parent_point[child] = current_point
                        level[child] = level[current_point] + 1
                        # clear all values from the queue to stop the loop
                        with bfs_queue.mutex:
                            bfs_queue.queue.clear()
                        break

                    visited[child] = True
                    parent_point[child] = current_point
                    level[child] = level[current_point] + 1
                    # add child to the queue
                    bfs_queue.put(child)

                    self.map[child] = color_dict['traversal']

                    if showImage:
                        if count % 10 == 0:
                            map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                            cv2.imshow('result', map_resized) 
                            cv2.waitKey(1)
            count += 1

        # check if a path was found betweeen start and end positions
        if child != self.end:
            print("\nNo path found between start and end positions!\n")
            if showImage:
                map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized)         
                cv2.waitKey()
            return 0, 0
        else:
            print('Path found!\n')
            if showImage:
                map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized)         
                cv2.waitKey()
                cv2.destroyAllWindows()

        # now the shortest path can be obtained by starting from end goal and tracing back to start through the parent_point nodes
        bfs_path = []
        curr_point = self.end
        while curr_point != self.start:
            parent_pt = parent_point[curr_point]
            bfs_path.append(parent_pt)
            curr_point = parent_pt
        
        return bfs_path[:-1][::-1], count



    def dijkstra(self, showImage = False):
        """
        Performs Dijkstra's algorithm on a grid map to find path between a starting and an ending point. Shortest path is guarenteed.

        Returns
        -------
        list
            The path found by using the algorithm.
        """

        # tracking of nodes in Dijkstra's algorithm is done using a queue data structure
        dijkstra_queue = Queue()
        visited = {}
        parent_point = {}
        level = {}
        # get the information regarding the neighbours of all the points in the map
        weighted_adj_list = get_weighted_adj_list(self.map)

        for point in weighted_adj_list.keys():
            # print(point, ': ', weighted_adj_list[point])
            visited[point] = False
            parent_point[point] = None
            level[point] = -1
            
        visited[self.start] = True
        level[self.start] = 0
        current_point = self.start
        dijkstra_queue.put(self.start)
        
        # declaring the child variable to avoid early reference error
        child = None
        count = 0 # for visualization
        while not dijkstra_queue.empty():
            
            current_point = dijkstra_queue.get()
            for child, cost in weighted_adj_list[current_point]:
                # check if the child node is already visited (skip the child node if already visited)
                if visited[child]:
                    continue

                # if the child node is not already visited
                else:
                    # check if the the child node is the end goal
                    if child == self.end:
                        visited[child] = True
                        parent_point[child] = current_point
                        level[child] = level[current_point] + cost
                        # clear all values from the queue to stop the loop
                        with dijkstra_queue.mutex:
                            dijkstra_queue.queue.clear()
                        break

                    visited[child] = True
                    parent_point[child] = current_point
                    level[child] = level[current_point] + cost
                    # add child to the queue
                    dijkstra_queue.put(child)

                    self.map[child] = color_dict['traversal']
                    if showImage:
                        if count % 20 == 0:
                            map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                            cv2.imshow('result', map_resized) 
                            cv2.waitKey(1)
            count += 1

        # check if a path was found betweeen start and end positions
        if child != self.end:
            print("\nNo path found between start and end positions!\n")
            if showImage:
                map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized)         
                cv2.waitKey()
            return 0, 0
        else:
            print('Path found!\n')
            if showImage:
                map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized)         
                cv2.waitKey()
                cv2.destroyAllWindows()

        # now the shortest path can be obtained by starting from end goal and tracing back to start through the parent_point nodes
        dijkstra_path = []
        curr_point = self.end
        while curr_point != self.start:
            parent_pt = parent_point[curr_point]
            dijkstra_path.append(parent_pt)
            curr_point = parent_pt
        
        return dijkstra_path[:-1][::-1], count


    def random_planner(self, showImage = False):
        """
        Performs a random planner based path planning on a grid map to find path between a starting and an ending point.

        Returns
        -------
        list
            The path found by using the algorithm.
        """

        parent_point = {}
        # get the information regarding the neighbours of all the points in the map
        adj_list = get_adj_list(self.map)

        for point in adj_list.keys():
            # print(point, ': ', adj_list[point])
            parent_point[point] = None
            
        current_point = self.start
        random_path = [current_point]

        # declaring the child variable to avoid early reference error
        child = None
        count = 0 # for visualization and checking number of iterations
        while current_point != self.end and count < self.max_iterations and len(adj_list[current_point]) > 0:

            if current_point == None:
                break
            # choose a child randomly
            child = random.choice(adj_list[current_point])
                
            
            # check if the child node is the end goal
            if child == self.end:
                parent_point[child] = current_point
                break
            
            parent_point[child] = current_point
            current_point = child   
            random_path.append(current_point)

            self.map[child] = color_dict['traversal']
            if showImage:
                if count % 1000 == 0:
                    map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                    cv2.imshow('result', map_resized) 
                    cv2.waitKey(1)
            
            count += 1 

        # check if a path was found betweeen start and end positions
        if count >= self.max_iterations and child != self.end:
            print('\nMaximum iterations exceeded! No path found!\n')
            if showImage:
                map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized)         
                cv2.waitKey()
            return 0, self.max_iterations

        elif child != self.end:
            print("\nNo path found between start and end positions!\n")
            if showImage:
                map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized)         
                cv2.waitKey()
            return 0, 0

        else:
            print('Path found!\n')
            if showImage:
                map_resized = cv2.resize(self.map, (6*self.map.shape[0], 6*self.map.shape[1]), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized)         
                cv2.waitKey()
                cv2.destroyAllWindows()
        
        return random_path, count