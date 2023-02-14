import numpy as np
import matplotlib.pyplot as plt
from environment.obsctacle_field import create_grid_with_obstacles
from helper_functions import color_dict
from planner import Planner


def one_cycle(desired_coverage, grid_size, start, end, maxiter = 3):

    dfs_N_iter, bfs_N_iter, dijkstra_N_iter, random_N_iter = -1, -1, -1, -1
    flag = True
    counter = 0
    while counter < maxiter and (dfs_N_iter <= 0 or bfs_N_iter <= 0 or dijkstra_N_iter <= 0 or random_N_iter <= 0):

        map = np.array(create_grid_with_obstacles(grid_size, desired_coverage))

        start = (0, 0)
        map[start] = color_dict['start']    # set the start cell as green color
        end = (grid_size - 1, grid_size - 1)
        map[end] = color_dict['goal']   # set the end goal cell as red color

        planner = Planner(map, start, end, max_iterations=40000)

        if dfs_N_iter <= 0 or bfs_N_iter <= 0 or dijkstra_N_iter <= 0 or random_N_iter <= 0:
            # check if a path exist
            _, bfs_N_iter = planner.bfs()

            _, dfs_N_iter = planner.dfs()

            _, dijkstra_N_iter = planner.dijkstra()
            
            _, random_N_iter = planner.random_planner()

        counter += 1
    
    if counter >= maxiter:
        print('\nMax iterations per obstacle coverage exceeded. Terminating path planning!')
        flag = False
    if counter < maxiter:
        # print(dfs_N_iter, bfs_N_iter, dijkstra_N_iter, random_N_iter)
        return [dfs_N_iter, bfs_N_iter, dijkstra_N_iter, random_N_iter], flag


def main():
    
    print('\nPlotting the performance of the path planning algorithms under study!')
    
    flag = True
    grid_size = 128
    maxDensity = 75
    steps = 25
    obstacleDensities = np.linspace(0, 40, steps)

    dfs_iterations = []
    bfs_iterations = []
    dijkstra_iterations = []
    random_iterations = []

    for desired_coverage in obstacleDensities:
        if flag == True:
            start = (0, 0)
            end = (grid_size - 1, grid_size - 1)

            print(f"\n\nStarting path planning cycle for {desired_coverage} percent coverage!\n")
            iter_values, flag = one_cycle(desired_coverage, grid_size, start, end, maxiter=100)

            if flag == True: # for end case
                dfs_iterations.append(iter_values[0])
                bfs_iterations.append(iter_values[1])
                dijkstra_iterations.append(iter_values[2])
                random_iterations.append(iter_values[3])

    print('\n\nPlotting the iteration values!')
    plt.plot(obstacleDensities, dfs_iterations, label='DFS')
    plt.plot(obstacleDensities, bfs_iterations, label='BFS')
    plt.plot(obstacleDensities, dijkstra_iterations, label='Dijkstra')
    plt.plot(obstacleDensities, random_iterations, label='Random Planner')
    plt.xlabel('Obstacle Density')
    plt.ylabel('Iterations')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
