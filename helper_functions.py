import numpy as np

color_dict = {'start': (0, 255, 170), 
                'white': (255, 255, 255), 
                'black': (0, 0, 0), 
                'goal': (0, 0, 255),
                'traversal': (255, 255, 0),
                'path': (255, 0, 0)}

# (245, 233, 202)
def get_adj_list(image):
    """
    Returns the adjacency list of all traversable points in the grid environment.

    Parameters
    ----------
    image : array_like
        Map of the environment.
    
    Returns
    -------
    dict
        Dictionary containing information regarding the adjacency of each traversable point.
    """
    adj_list = {}
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            adj_list[(i, j)] = get_traversable_neighbours(image, (i, j))
    
    return adj_list


def get_weighted_adj_list(image):
    """
    Returns the adjacency list of all traversable points in the grid environment. A cost of 1 is assigned to 
    horizontal and vertical steps, while a cost of sqrt(2) is assigned to diagonal steps.

    Parameters
    ----------
    image : array_like
        Map of the environment.
    
    Returns
    -------
    dict
        Dictionary containing information regarding the adjacency and cost of movement of each traversable point.
    """
    weighted_adj_list = {}
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            weighted_adj_list[(i, j)] = get_traversable_weighted_neighbours(image, (i, j))
    
    return weighted_adj_list


def get_traversable_neighbours(image, loc):
    """
    Finds all the traversable neighbours of a point on an environment grid.

    Parameters
    ----------
    image : array_like
        Map of the environment.
    loc : tuple of ints
        The location of point.
    
    Returns
    -------
    list of tuples
        List containting all traversable nerighbours.
    """

    neighbour_list = []
    # check if the input point is an obstacle
    if list(image[loc]) ==  list(color_dict['black']):
        return neighbour_list
    
    x, y = loc
    for i, j in [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]:
        if i >= 0 and i <= (image.shape[0] - 1) and j >=0 and j <= (image.shape[1] - 1):
            if list(image[i, j]) != list(color_dict['black']):
                neighbour_list.append((i, j))
    return neighbour_list
    

def get_unvisited_child_nodes(adj_list, loc, visited):
    """
    Finds all the traversable neighbours of a point on an environment grid.

    Parameters
    ----------
    image : array_like
        Map of the environment.
    loc : tuple of ints
        The location of point.
    visited : dict
        Contains info regarding whether a point is already visited.


    Returns
    -------
    list
        List of unvisited child nodes of the input point.
    """
    unvisited_child_nodes = []
    
    for child in adj_list[loc]:

        if not visited[child]:
            unvisited_child_nodes.append(child)
    
    return unvisited_child_nodes


def get_traversable_weighted_neighbours(image, loc):
    """
    Finds all the traversable neighbours of a point on an environment grid with a cost of 1
    for horizontal and vertical steps and a cost of sqrt(2) for diagonal steps.

    Parameters
    ----------
    image : array_like
        Map of the environment.
    loc : tuple of ints
        The location of point.

    Returns
    -------
    list
        List containing neighbouring traversable point and respective cost of movement.
    """

    neighbour_weighted_list = []
    # check if the input point is an obstacle
    if list(image[loc]) ==  list(color_dict['black']):
        return neighbour_weighted_list

    x, y = loc
    check_list_1 = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]     # cost = 1
    check_list_2 = [(x - 1, y - 1), (x - 1, y + 1), (x + 1, y - 1), (x + 1, y + 1)]     # cost = sqrt(2)

    # neighbours with cost 1
    for i, j in check_list_1:
        if i >= 0 and i <= (image.shape[0] - 1) and j >=0 and j <= (image.shape[1] - 1):
            if list(image[i, j]) != list(color_dict['black']):
                neighbour_weighted_list.append([(i, j), 1])

    # neighbours with cost sqrt(2)
    for i, j in check_list_2:
        if i >= 0 and i <= (image.shape[0] - 1) and j >=0 and j <= (image.shape[1] - 1):
            if list(image[i, j]) != list(color_dict['black']):
                neighbour_weighted_list.append([(i, j), np.sqrt(2)])
    
    return neighbour_weighted_list