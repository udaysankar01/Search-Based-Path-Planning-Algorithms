import cv2
import numpy as np
import argparse, textwrap
from planner import Planner
from environment.obsctacle_field import create_grid_with_obstacles
from helper_functions import color_dict


def main():

    parser = argparse.ArgumentParser(description='Implementation of Path Planning Algorithms',
        usage='use "python %(prog)s --help" for more information',
        formatter_class=argparse.RawTextHelpFormatter)
    
    parser.add_argument('algorithm', default=1, type=int,
        help= textwrap.dedent('''\
        Depth-First Search: 1
        Breadth-First Search: 2
        Dijkstra's Algorithm: 3
        Random Planner: 4

        default: 1
        '''))
    parser.add_argument('--gridsize', default=128, type=int,
        help= textwrap.dedent('''\
        Size of the grid environment. default: 128
        '''))
    parser.add_argument("--coverage", type=int, default=30,
        help= textwrap.dedent('''\
        Desired coverage in percentage of obstacles in the environment. default: 30
        '''))
    parser.add_argument("--maxiter", type=int, default=10000,
        help= textwrap.dedent('''\
        Number of maximum iterations to be performed for Random planner based path planning. default: 10000
        '''))
    

    args = parser.parse_args()
    algorithm_type = args.algorithm
    grid_size = args.gridsize
    desired_coverage = args.coverage
    max_iterations = args.maxiter

    if algorithm_type not in [1, 2, 3, 4]:
        print("\nAlgorithm number chosen is wrong!\n")
        quit()

    """
    Generate a grid environment with desired obstacle coverage and size, in which a point robot will be 
    placed in the top-left corner, and the destination will be set as the bottom-right corner.
    """
    map = np.array(create_grid_with_obstacles(grid_size, desired_coverage))

    start = (0, 0)
    map[start] = color_dict['start']    # set the start cell as red color
    end = (grid_size - 1, grid_size - 1)
    map[end] = color_dict['goal']   # set the end goal cell as green color

    # testing for possible errors
    # map[1, 0] = color_dict['black']
    # map[0, 1] = color_dict['black']
    # map[1, 1] = color_dict['black']

    # visualizing the grid map
    map_resized = cv2.resize(map, (6*grid_size, 6*grid_size), interpolation=cv2.INTER_NEAREST)
    cv2.imshow('result', map_resized)         
    cv2.waitKey()

    
    if algorithm_type == 1:
        """
        Apply Depth-First search algorithm to find the path from start location and end location
        in the 2D grid map obtained above.
        """
        print("\nInitiating Depth-First Search Algorithm.\n")
        count = 0
        planner = Planner(map, start, end)
        dfs_path, iterations = planner.dfs(showImage=True)
        
        if iterations == 0:
            quit()

        for point in dfs_path:
            map[point] = color_dict['path']

            if count % 20 == 0:
                map_resized = cv2.resize(map, (6*grid_size, 6*grid_size), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized) 
                cv2.waitKey(1)
            count += 1
        map_resized = cv2.resize(map, (6*grid_size, 6*grid_size), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('result', map_resized)         
        cv2.waitKey()
        cv2.destroyAllWindows()

    
    elif algorithm_type == 2:
        """
        Apply Breadth-First search algorithm to find the path from start location and end location
        in the 2D grid map obtained above.
        """
        print("\nInitiating Breadth-First Search Algorithm.\n")
        planner = Planner(map, start, end)
        bfs_path, iterations = planner.bfs(showImage=True)

        if iterations == 0:
            quit()
        
        for point in bfs_path:
            map[point] = color_dict['path']

            map_resized = cv2.resize(map, (6*grid_size, 6*grid_size), interpolation=cv2.INTER_NEAREST)
            cv2.imshow('result', map_resized) 
            cv2.waitKey(1)

        map_resized = cv2.resize(map, (6*grid_size, 6*grid_size), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('result', map_resized)         
        cv2.waitKey()
        cv2.destroyAllWindows()

    elif algorithm_type == 3:
        """
        Apply Dijkstra algorithm to find the path from start location and end location
        in the 2D grid map obtained above.
        """
        print("\nInitiating Dijkstra's Algorithm.\n")
        planner = Planner(map, start, end)
        dijkstra_path, iterations = planner.dijkstra(showImage=True)

        if iterations == 0:
            quit()

        for point in dijkstra_path:
            map[point] = color_dict['path']

            map_resized = cv2.resize(map, (6*grid_size, 6*grid_size), interpolation=cv2.INTER_NEAREST)
            cv2.imshow('result', map_resized) 
            cv2.waitKey(1)

        map_resized = cv2.resize(map, (6*grid_size, 6*grid_size), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('result', map_resized)         
        cv2.waitKey()
        cv2.destroyAllWindows()

    else:
        """
        Apply a Random planner algorithm to find the path from start location and end location
        in the 2D grid map obtained above.
        """
        print("\nInitiating Random Planner algorithm.\n")
        count = 0
        planner = Planner(map, start, end, max_iterations)
        random_path, iterations = planner.random_planner(showImage=True)

        if iterations == 0:
            quit()

        for point in random_path:
            map[point] = color_dict['path']

            if count % 1000 == 0:
                map_resized = cv2.resize(map, (6*grid_size, 6*grid_size), interpolation=cv2.INTER_NEAREST)
                cv2.imshow('result', map_resized) 
                cv2.waitKey(1)
            count += 1

        map_resized = cv2.resize(map, (6*grid_size, 6*grid_size), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('result', map_resized)         
        cv2.waitKey()
        cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main()