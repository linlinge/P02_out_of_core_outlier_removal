import networkx as nx
import metis

# Set up graph structure
G = nx.Graph()
G.add_edges_from([ (0,1), (0,2), (0,3), (1, 2), (3, 4) ])

# Add node weights to graph
for i, value in enumerate([1,3,2,4,3]):
    G.node[i]['node_value'] = value

# tell METIS which node attribute to use for 
G.graph['node_weight_attr'] = 'node_value' 

# Get at MOST two partitions from METIS
(cut, parts) = metis.part_graph(G, 2) 
# parts == [0, 0, 0, 1, 1]

# Assuming you have PyDot installed, produce a DOT description of the graph:
colors = ['red', 'blue']
for i, part in enumerate(parts):
    G.node[i]['color'] = colors[part]
nx.nx_pydot.write_dot(G, 'example.dot')