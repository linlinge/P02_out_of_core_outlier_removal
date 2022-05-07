import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from networkx.algorithms.community import kernighan_lin_bisection
from networkx.algorithms.community import greedy_modularity_communities
from networkx.algorithms.community import label_propagation_communities

G = nx.karate_club_graph()
colors = ["#00C98D", "#5030C0", "#50F0F0"]
pos = nx.spring_layout(G)

lst_m = greedy_modularity_communities(G)
color_map_b = ["black"] * nx.number_of_nodes(G)
counter = 0
for c in lst_m:
  for n in c:
    color_map_b[n] = colors[counter]
  counter = counter + 1
nx.draw_networkx_edges(G, pos)
nx.draw_networkx_nodes(G, pos, node_color=color_map_b)
nx.draw_networkx_labels(G, pos)
plt.axis("off")
plt.show()

lst_l = label_propagation_communities(G)
color_map_b = ["black"] * nx.number_of_nodes(G)
counter = 0
for c in lst_l:
  for n in c:
    color_map_b[n] = colors[counter]
  counter = counter + 1
nx.draw_networkx_edges(G, pos)
nx.draw_networkx_nodes(G, pos, node_color=color_map_b)
nx.draw_networkx_labels(G, pos)
plt.axis("off")
plt.show()