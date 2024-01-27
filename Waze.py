import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt

place_name = "Rennes, Bretagne, France"


# gdf : GeoDataFrame = Structure de données contenant des informations de localisation
area = ox.geocode_to_gdf(place_name)

# Display the area
ax = area.plot()
_ = ax.axis("off")
# plt.show()


# La fonction ox.graph_from_place permet d'obtenir un graphe à partir du nom d'un lieu
G = ox.graph_from_place(
    place_name, network_type="drive"
)  # drive spécifie ici uniquement les chemins praticables en voiture
# ox.plot_graph(G)

#  on affiche le nombre de sommets, aretes et longueur totale de route
print("Nombre de sommets : ", len(G.nodes))
print("Nombre d'aretes : ", len(G.edges))
print("Longueur totale de route : ", sum([G.edges[e]["length"] for e in G.edges]))


# ID de deux sommets adjacents
vertice_ID = 2483704435
vertice_ID2 = 60219063

# Accès au sommet par ID
G.nodes[vertice_ID]

# Accès aux informations de l'arête reliant deux sommets
G.get_edge_data(vertice_ID, vertice_ID2)


# Ajoute l'attribut "speed_kph" (=max speed limit in km/hour) à toutes les arêtes
G = ox.add_edge_speeds(G)

# Calcul et ajoute l'attribut "travel_time" (seconds) à toutes les arêtes
G = ox.add_edge_travel_times(G)


# Permet de visualiser les informations du graphe de manière interactive
nodes, edges = ox.graph_to_gdfs(G)
m = edges.explore(color="skyblue", tiles="cartodbpositron")
nodes.explore(m=m, color="pink", marker_kwds={"radius": 6})


""" On souhaite trouver le plus court chemin nous menant de l'ISTIC au centre ville.
3. Localiser les sommets dont l'osmid est de 59980909 (près de l'accueil de l'ISTIC) et de 60822097 (près du centre ville) """

# On récupère les coordonnées des sommets
node_ISTIC = G.nodes[59980909]
node_centre_ville = G.nodes[60822097]

""" Utiliser les fonctions de Networkx pour déterminer le chemin de plus courte distance entre ces deux sommets. """

# On récupère le chemin le plus court
shortest_path = nx.shortest_path(G, 59980909, 60822097, weight="length")


"""AComparer les temps d'exécution de Dijkstra et Bellman-Ford. Un code à compléter vous est fourni pour cela. Est-il pertinent d'utiliser Bellman-Ford dans ce cas ? """


import time

# On récupère les temps d'exécution de Dijkstra et Bellman-Ford
start_time = time.time()
shortest_path = nx.shortest_path(G, 59980909, 60822097, weight="length")
print("--- %s seconds ---" % (time.time() - start_time))

start_time = time.time()
shortest_path = nx.bellman_ford_path(G, 59980909, 60822097, weight="length")
print("--- %s seconds ---" % (time.time() - start_time))


""" 5. Afficher le chemin obtenu sur la carte. """
# ox.plot_graph_route(G, shortest_path, route_color="green")


"""On suppose maintenant que la position de départ est obtenu à l'aide d'une technologie GPS.
8. Obtenir le plus court chemin pour se rendre à l'ISTIC pour une position de départ de (latitude : 48.10697825211165 ; longitude : -1.6768960560938104). Vous pourrez utiliser la fonction nearest_nodes pour obtenir le sommet le plus proche d'une position GPS. """

# On récupère le sommet le plus proche de la position GPS
origin_node = ox.nearest_nodes(G, 48.10697825211165, -1.6768960560938104)

# On récupère le chemin le plus court
shortest_path = nx.shortest_path(G, origin_node, 59980909, weight="length")


""" 9. Afficher le chemin obtenu sur la carte. """
ox.plot_graph_route(G, shortest_path, route_color="green")
