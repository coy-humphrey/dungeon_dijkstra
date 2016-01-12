# Authors: Coy Humphrey and Jake Preston

from p1_support import load_level, show_level, save_level_costs
from math import inf, sqrt
from heapq import heappop, heappush


def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    pq = []
    dist = {initial_position: 0}
    prev = {initial_position: None}
    heappush(pq,(0,initial_position))
    while pq:
        d,cord = heappop(pq)
        if (cord == destination):
            break
        for x,weight in adj(graph,cord):
            alt = d + weight
            if alt < dist.get(x,alt+1):
                dist[x] = alt
                prev[x] = cord
                heappush(pq,(dist[x],x))
    result = []
    curr = destination
    while curr in prev:
        result.insert(0, curr)
        curr = prev[curr] 
    return result



def dijkstras_shortest_path_to_all(initial_position, graph, adj):
    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """
    pq = []
    dist = {initial_position: 0}
    prev = {initial_position: None}
    heappush(pq,(0,initial_position))
    while pq:
        d,cord = heappop(pq)
        for x,weight in adj(graph,cord):
            alt = d + weight
            if alt < dist.get(x,alt+1):
                dist[x] = alt
                prev[x] = cord
                heappush(pq,(dist[x],x))
    return dist



def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    def getWeight(src, dest):
        """ Given two adjacent cells, gives the cost of navigating the edge between them

        Args:
            src: The source cell
            dest: The destination cell adjacent to src

        Returns:
            A floating point number representing the cost to travel from src to dest

        E.g. 1.4142135623730951
        """
        # pull x and y coordinates from both src and dest
        x,y = src
        i,j = dest
        weight = 0
        spaces = level['spaces']
        # If neither x nor y remain the same (both change) we're moving diagonally
        if not((x==i) or (y==j)):
            weight = .5*sqrt(2)*spaces[src]+.5*sqrt(2)*spaces[dest]
        else:
            weight = .5*spaces[src]+.5*spaces[dest]
        return weight

    x,y = cell
    spaces = level['spaces']
    # Generates a list of tuples [(-1,-1), (-1,0), (0,1)...] that will be used to get the 8 adjacent cells
    modifiers = [(i,j) for i in range(-1, 2) for j in range(-1,2) if not (i == 0 and j ==0)]
    # Uses the tuples from modifier to get the 8 adjacent cells. Filters out invalid coordinates.
    validCoords = [(x+i, y+j) for i,j in modifiers if (x+i, y+j) in spaces]
    # Return a list of tuples in the form [(adjacentCell, weightTocell)]
    return [(t, getWeight(cell, t)) for t in validCoords]


def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")


def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from 
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.

    """
    
    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]
    
    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'my_maze.txt', 'a','d'

    # Use this function call to find the route between two waypoints.
    #test_route(filename, src_waypoint, dst_waypoint)

    # Use this function to calculate the cost to all reachable cells from an origin point.
    cost_to_all_cells(filename, src_waypoint, 'my_maze_costs.csv')
