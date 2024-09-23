import heapq
import math

def restaurantFinder(d,site_list):
    """
    Function description: This function returns the max revenue that one can get when choosing certain restaurants from a list, each restaturant chosen being d distance apart.

    Approach description (if main function): Let's assume we choose to place restaurant at site i, this means that we can't place a restaurant in between i and i - d, and therefore
    must place restaurants along [1, ..., i - 1 - d]. Therefore given a list max_revenue, we can populate this in two ways, max_revenue[i] can be max_revenue [i - 1] if we decide 
    to not place a restaurant on site i, or max_revenue[i-1-d] + the next site revenue (i-1) if we decide to place a restaurant on site i.
    Then we can backtrack to find which restaurants were chosen.

    Input:
        d: minimum distance each restaurant has to be apart from each other.
        site_list: list of restaurant revenues, with the distance between site i and site i + x being x.

    Output: a tuple containing the max revenue and the sites chosen for the restaurants.

    Time complexity: O(N), where n is the number of elements in the site_list input array.
    Aux space complexity: O(N), where n is the number of elements in the site_list input array.
    """
    N = len(site_list)
    max_revenue = [0] * (N + 1)
    restaurants = []

    next_restaurant = 0

    for i in range(1, N + 1):

        prev = site_list[i - 1]

        if i > d:
            prev += max_revenue[i-d-1] 

        max_revenue[i] = max(max_revenue[i - 1], prev)

    i = N
    while i > 0:
        if max_revenue[i] > max_revenue[i-1]:
            restaurants.append(i)
            i = i - d -1
        else:
            i = i - 1
    restaurants.reverse()

    return (max_revenue[N], restaurants)

class FloorGraph:
    
    def __init__(self, paths, keys):
        """
        Initial function that creates class variables for FloorGraph, including the set of keys, paths (in an adjacency list) and number of vertices.

        Input:
            self: the class object.
            paths: a list of paths represented as a list of tuples (u, v, x), where u is the starting location ID for a path (non-negative), v is the ending location ID (non-negative)
                   x ios the amount of time needed to travel down the path from u to v (non-negative).
            keys: a list of keys represented as a list of tuples (k, y), where k is the location ID where a key is found (non-negative) and y is the amount of time needed to retrieve the key.

        Time complexity: O(|E|+|V|), where E is the set paths and V is the set of unique locations in paths.
        Aux space complexity: O(|E|+|V|), where E is the set paths and V is the set of unique locations in paths.
        """
        self.keys = keys
        self.paths, self.V = self.parse_paths(paths)

    def parse_paths(self, paths):
        """
        Function that converts the list of paths given in the initial function into an adjacency list.

        Input:
            self: the class object.
            paths: a list of paths represented as a list of tuples (u, v, x), where u is the starting location ID for a path (non-negative), v is the ending location ID (non-negative)
                   x ios the amount of time needed to travel down the path from u to v (non-negative).

        Output:
            A list of lists, where each index i representes a vertex, and output[i] is a list of lists [v, w], where v is the location ID of the vertex adjacent to i and w is the time taken to travel u to v.
                   
        Time complexity: O(|E|+|V|), where E is the set paths and V is the set of unique locations in paths.
        Aux space complexity: O(|E|+|V|), where E is the set paths and V is the set of unique locations in paths.
        """

        vertices  = [[] for i in paths]

        if len(paths) == 1:
            vertices.append([]) 

        no_vertices = paths[0][0]

        for edge in paths:
            max_edge = max(edge[0], edge[1])
            if max_edge > no_vertices: 
                no_vertices = max_edge
            vertices[edge[0]].append([edge[1], edge[2]])
        
        if no_vertices + 1 > len(vertices):
            for i in range(no_vertices + 1 - len(vertices)):
                vertices.append([])

        return vertices[:no_vertices + 1], no_vertices + 1
    
    def dijkstra(self, S):
        """
        Function that implements dijkstra's algorithm to find the shortest distance between the start node (S) and every other vertex.

        Input:
            self: the class object.
            S: the starting vertex
        Output:
            A tuple containing a list of the shortest distance to each vertex, and a list of lists containing the paths traversed to get to that vertex.

        Time complexity: O(|E|log|V|), where E is the set paths and V is the set of unique locations in paths.
        Aux space complexity: O(|V| + |E|), where E is the set paths and V is the set of unique locations in paths.
        """

        parents = [-1 for i in self.paths]

        distance = [math.inf] * self.V
        distance[S] = 0
        
        minHeap = []
        heapq.heapify(minHeap)
        
        heapq.heappush(minHeap, (0,S))
        
        while minHeap:
            dist , node = heapq.heappop(minHeap)

            neighbor = self.paths[node]
            
            for nei in neighbor:
                adj_nei , weight = nei
                
                update_dist = weight + dist
                
                if update_dist < distance[adj_nei] :
                    distance[adj_nei] = update_dist
                    heapq.heappush(minHeap , (update_dist , adj_nei))
                    parents[adj_nei] = node

        all_paths = self.get_total_paths(S, parents)

        return distance, all_paths
    
    
    def get_total_paths(self, S, parents):
        """
        Function that returns all paths traversed to get to certain vertices.

        Input:
            self: the class object.
            S: the starting vertex
            parents: a list integers, where if parents[i] == -1, then the vertex has no parent, otherwise parents[i] is the parent vertex of i.
        Output:
            A list of lists containing the paths traversed to get to that all vertices.

        Time complexity: O(|V|), V is the set of unique locations in paths.
        Aux space complexity: O(1).
        """

        all_paths = []
        for i in range(self.V):
            if i != S:
                path_list = []
                path_list = self.get_path(i, parents, i, path_list)
                all_paths.append(path_list)
            else:
                all_paths.append([S])

        return all_paths


    def get_path(self, current_vertex, parents, original_vertex, path_list):
        """
        A recursive function that iterates through the parents of each vertex to get the path traversed to get to the original vertex.

        Input:
            self: the class object.
            current_vertex: the current vertex to find the path to.
            parents: a list integers, where if parents[i] == -1, then the vertex has no parent, otherwise parents[i] is the parent vertex of i.
            original_vertex: the original vertex we are finding the path to (same as current_vertex on the first iteration).
            path_list: a list of the path travelled up to current_vertex.
        Output:
            A list containing the vertices travelled to get to current_vertex.

        Time complexity: O(|V|), V is the set of unique locations in paths.
        Aux space complexity: O(1).
        """

        if current_vertex == -1:
            return path_list
        
        path_list = self.get_path(parents[current_vertex], parents, original_vertex, path_list)
        path_list.append(current_vertex)
        return path_list

    def climb(self, start, exits):
        """
        A function that returns the shortest route from the start to one of the exit points, this route includes passing through one of the keys in self.keys.

        Approach description: First we use dijkstra's algorithm to find the shortest distance between the start node (start) and every other vertex. Then we can 
        use dijkstra's algorithm again to find the shortest path from the each key to every other vertex. Once we have these paths we can simply search for the 
        shortest path that goes from the start to one of the keys and from these keys to the an exit point.

        Input:
            self: the class object.
            start: the vertex to start at.
            exits: a list of vertices that one can one to exit.
        Output:
            A tuple with the total time spent, and the route taken to exit.

        Time complexity: O(|E|log|V|), where E is the set paths and V is the set of unique locations in paths.
        Aux space complexity: O(|V| + |E|), where E is the set paths and V is the set of unique locations in paths.
        """
        distances, paths = self.dijkstra(start)

        key_list = []

        key_dist = []
        key_paths = []

        for key in self.keys:
            key_list.append(key[0])
            dist, keyp = self.dijkstra(key[0])
            key_dist.append([(i + key[1]) for i in dist])
            key_paths.append(keyp)


        min_path = []
        min_dist = math.inf

        #loops through all paths from keys to the exits and grabs the shortest path.
        for i in range(len(key_dist)):
            for j in range(len(key_dist[i])):
                if j in exits:
                    if distances[key_list[i]] + key_dist[i][j] < min_dist:
                        min_dist = distances[key_list[i]] + key_dist[i][j]
                        if len(paths[key_list[i]]) == 0:
                            min_path = key_paths[i][j]
                        else:
                            min_path = paths[key_list[i]] + key_paths[i][j][1:]

        if min_dist == math.inf:
            return None

        return min_dist, min_path



