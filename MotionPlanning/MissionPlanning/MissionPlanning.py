import osmnx as ox
import networkx as nx
import queue
import math
import priority_dict

map_graph = ox.graph_from_place('Berkeley, California', network_type='drive') ## Use osmnx ver 1.0.1
origin = ox.get_nearest_node(map_graph, (37.8743, -122.277))
destination = list(map_graph.nodes())[-1]

shortest_path = nx.shortest_path(map_graph, origin, destination, weight='length')
fig, ax = ox.plot_graph_route(map_graph, shortest_path)

# For a given graph, origin vertex key, and goal vertex key,
# computes the shortest path in the graph from the origin vertex
# to the goal vertex using Dijkstra's algorithm.
# Returns the shortest path as a list of vertex keys.
def dijkstras_search(origin_key, goal_key, graph):
    
    # The priority queue of open vertices we've reached.
    # Keys are the vertex keys, vals are the distances.
    open_queue = priority_dict.priority_dict({})
    
    # The dictionary of closed vertices we've processed.
    closed_list = []
    
    # The dictionary of predecessors for each vertex.
    predecessors = {}
    
    # Add the origin to the open queue.
    open_queue[origin_key] = 0.0

    # Iterate through the open queue, until we find the goal.
    # Each time, perform a Dijkstra's update on the queue.
    # TODO: Implement the Dijstra update loop.
    goal_found = False
    while (open_queue):
        node, u_cost = open_queue.pop_smallest()
        if node == goal_key:
            goal_found = True
            break
    
        out_edges_target = [v for i, v in list(graph.out_edges(node))]
        edge_data = graph.out_edges([node], data=True)
            
        for edge in edge_data:
            v = edge[1]
            if v in closed_list:
                continue
                
            uv_cost = edge[2]['length']
            if v in open_queue.keys():
                if u_cost + uv_cost < open_queue[v]:
                    open_queue[v] = u_cost + uv_cost
                    predecessors[v] = node
            else:
                open_queue[v] = u_cost + uv_cost
                predecessors[v] = node
            
            closed_list.append(node)
            
    # If we get through entire priority queue without finding the goal,
    # something is wrong.
    if not goal_found:
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    return get_path(origin_key, goal_key, predecessors)                


# This function follows the predecessor
# backpointers and generates the equivalent path from the
# origin as a list of vertex keys.
def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]

    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)

    return path

# Computes the Euclidean distance between two vertices.
# Assume that the earth is a sphere with radius 6371 km.
def distance_heuristic(state_key, goal_key, node_data):
    n1 = node_data[state_key]
    n2 = node_data[goal_key]

    # Get the longitude and latitude for each vertex.
    long1 = n1['x']*math.pi/180.0
    lat1 = n1['y']*math.pi/180.0
    long2 = n2['x']*math.pi/180.0
    lat2 = n2['y']*math.pi/180.0
    
    # Use a spherical approximation of the earth for
    # estimating the distance between two points.
    r = 6371000
    x1 = r*math.cos(lat1)*math.cos(long1)
    y1 = r*math.cos(lat1)*math.sin(long1)
    z1 = r*math.sin(lat1)

    x2 = r*math.cos(lat2)*math.cos(long2)
    y2 = r*math.cos(lat2)*math.sin(long2)
    z2 = r*math.sin(lat2)

    d = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5
    
    return d

# For a given graph, origin vertex key, and goal vertex key,
# computes the shortest path in the graph from the origin vertex
# to the goal vertex using A* search. 
# Returns the shortest path as a list of vertex keys.
def a_star_search(origin_key, goal_key, graph):
    # The priority queue of open vertices we've reached.
    # Keys are the vertex keys, vals are the accumulated
    # distances plus the heuristic estimates of the distance
    # to go.
    open_queue = priority_dict.priority_dict({})
    
    # The dictionary of closed vertices we've processed.
    closed_dict = {}
    
    # The dictionary of predecessors for each vertex.
    predecessors = {}
    
    # The dictionary that stores the best cost to reach each
    # vertex found so far.
    costs = {}
    
    # Get the spatial data for each vertex as a dictionary.
    node_data = graph.nodes(True)
    
    # Add the origin to the open queue and the costs dictionary.
    costs[origin_key] = 0.0
    open_queue[origin_key] = distance_heuristic(origin_key, goal_key, node_data)

    # Iterate through the open queue, until we find the goal.
    # Each time, perform an A* update on the queue.
    goal_found = False
    while (open_queue):
        node, _ = open_queue.pop_smallest()
        u_cost = costs[node]
        if node == goal_key:
            goal_found = True
            break
    
        out_edges_target = [v for i, v in list(graph.out_edges(node))]
        edge_data = graph.out_edges([node], data=True)

        for edge in edge_data:
            v = edge[1]
            if v in closed_dict:
                continue
                
            uv_cost = edge[2]['length']
            heuristic_distance = distance_heuristic(v, goal_key, node_data)
            if v in open_queue.keys():
                if u_cost + uv_cost + heuristic_distance < open_queue[v]:
                    open_queue[v] = u_cost + uv_cost + heuristic_distance
                    costs[v] = u_cost + uv_cost
                    predecessors[v] = node
            else:
                open_queue[v] = u_cost + uv_cost + heuristic_distance
                costs[v] = u_cost + uv_cost
                predecessors[v] = node
            
            closed_dict[node] = 1
            
    # If we get through entire priority queue without finding the goal,
    # something is wrong.
    if not goal_found:
        raise ValueError("Goal not found in search.")
    
    # Construct the path from the predecessors dictionary.
    return get_path(origin_key, goal_key, predecessors)   

path = dijkstras_search(origin, destination, map_graph)
fig, ax = ox.plot_graph_route(map_graph, path)

path = a_star_search(origin, destination, map_graph)
fig, ax = ox.plot_graph_route(map_graph, path)
