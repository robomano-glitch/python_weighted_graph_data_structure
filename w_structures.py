import sys

class Graph(object):
    def __init__(self, nodes, init_graph):
        self.nodes = nodes
        self.graph = self.construct_graph(nodes, init_graph)
        
    def construct_graph(self, nodes, init_graph):
        
        graph = {}
        for node in nodes:
            graph[node] = {}

        graph.update(init_graph)

        for node, edges in graph.items():
            for adjacent_node, value in edges.items():
                if graph[adjacent_node].get(node, False) == False:
                    graph[adjacent_node][node] = value
        
        return graph
    
    def get_nodes(self):
        return self.nodes

    def get_outgoing_edges(self, node):
        connections = []
        for out_node in self.nodes:
            if self.graph[node].get(out_node, False) != False:
                connections.append(out_node)
        return connections

    def value(self, node1, node2):
        return self.graph[node1][node2]
 
def dijkstra_algorithm(graph, start_node):
    
    unvisited_nodes = list(graph.get_nodes())
    
    shortest_path = {}
    previous_nodes = {}
    
    max_value = sys.maxsize
    for node in unvisited_nodes:
        shortest_path[node] = max_value
        
    shortest_path[start_node] = 0

    while unvisited_nodes:
        current_min_node = None
        for node in unvisited_nodes: 
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node
        
        neighbors = graph.get_outgoing_edges(current_min_node)
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node] + graph.value(current_min_node, neighbor)
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                
                previous_nodes[neighbor] = current_min_node
        
        unvisited_nodes.remove(current_min_node)
        
    return previous_nodes, shortest_path

def print_result(previous_nodes, shortest_path, start_node, target_node):
    path = []
    node = target_node

    while node != start_node:
        path.append(node)
        node = previous_nodes[node]

    path.append(start_node)

    print("We found the following best path with a value of {}.".format(shortest_path[target_node]))
    print(" -> ".join(reversed(path)))

nodes = ["Kasungu", "Mchinji", "Lilongwe", "Dowa", "Ntchisi", "Nkhotakota", "Salima", "Dedza", "Ntcheu"]

init_graph = {}
for node in nodes:
    init_graph[node] = {}

init_graph["Kasungu"]["Mchinji"] = 141
init_graph["Kasungu"]["Dowa"] = 117
init_graph["Kasungu"]["Ntchisi"] = 66
init_graph["Mchinji"]["Kasungu"] = 141
init_graph["Mchinji"]["Lilongwe"] = 109
init_graph["Lilongwe"]["Dedza"] = 92
init_graph["Lilongwe"]["Mchinji"] = 109
init_graph["Lilongwe"]["Dowa"] = 55
init_graph["Dowa"]["Kasungu"] = 117
init_graph["Dowa"]["Lilongwe"] = 55
init_graph["Dowa"]["Ntchisi"] = 38
init_graph["Dowa"]["Salima"] = 67
init_graph["Ntchisi"]["Dowa"] = 38
init_graph["Ntchisi"]["Kasungu"] = 66
init_graph["Ntchisi"]["Nkhotakota"] = 66
init_graph["Nkhotakota"]["Ntchisi"] = 66
init_graph["Nkhotakota"]["Salima"] = 112
init_graph["Salima"]["Dowa"] = 67
init_graph["Salima"]["Nkhotakota"] = 112
init_graph["Salima"]["Dedza"] = 96
init_graph["Dedza"]["Lilongwe"] = 92
init_graph["Dedza"]["Ntcheu"] = 74
init_graph["Dedza"]["Salima"] = 92
init_graph["Ntcheu"]["Dedza"] = 74

graph = Graph(nodes, init_graph)

previous_nodes, shortest_path = dijkstra_algorithm(graph=graph, start_node="Ntcheu")

print_result(previous_nodes, shortest_path, start_node="Ntcheu", target_node="Kasungu")