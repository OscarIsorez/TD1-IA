import numpy as np
from scipy.sparse.csgraph import dijkstra, shortest_path
from scipy.sparse import csr_matrix

""" 
    0 = Angers
    1 = Bordeaux
    2 = Brest
    3 = Clermont-Ferrand
    4 = Le Havre
    5 = Lille
    6 = Lyon
    7 = Marseille
    8 = Paris
    9 = Rennes
    10 = Strasbourg
    11 = Toulouse

 """


# Angers Bordeaux Brest Clermont-Ferrand LeHavre Lille Lyon Marseille Paris Rennes Strasbourg Toulouse
matrice_cout = np.array(
    [
        [0, 350, 0, 0, 0, 0, 0, 0, 300, 130, 0, 0],  # Angers
        [350, 0, 0, 370, 0, 0, 0, 0, 0, 0, 0, 250],  # Bordeaux
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 240, 0, 0],  # Brest
        [0, 370, 0, 0, 0, 0, 170, 0, 425, 0, 0, 375],  # Clermont-Ferrand
        [0, 0, 0, 0, 0, 0, 0, 0, 200, 0, 0, 0],  # Le Havre
        [0, 0, 0, 0, 0, 0, 0, 0, 225, 0, 0, 0],  # Lille
        [0, 0, 0, 170, 0, 0, 0, 0, 0, 0, 0, 0],  # Lyon (modifié)
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 400],  # Marseille
        [300, 0, 0, 425, 200, 225, 0, 0, 0, 0, 500, 0],  # Paris
        [130, 0, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # Rennes
        [0, 0, 0, 0, 0, 0, 0, 0, 500, 0, 0, 0],  # Strasbourg
        [0, 250, 0, 375, 0, 0, 0, 400, 0, 0, 0, 0],  # Toulouse
    ]
)

# Convertir la matrice en une matrice sparse
sparse_matrice_cout = csr_matrix(matrice_cout)

# Exécuter l'algorithme de Dijkstra
dist_matrix, predecessors = dijkstra(
    csgraph=sparse_matrice_cout, directed=False, return_predecessors=True
)


def getAretes(matrice_cout):
    aretes = []
    for i in range(len(matrice_cout)):
        for j in range(len(matrice_cout)):
            if matrice_cout[i][j] != 0:
                aretes.append([i, j, matrice_cout[i][j]])
    return aretes


def getSommet(matrice_cout):
    sommets = []
    for i in range(len(matrice_cout)):
        sommets.append(i)
    return sommets


def assocPoid(arete, poid, matrice_cout):
    for i in range(len(matrice_cout)):
        for j in range(len(matrice_cout)):
            if matrice_cout[i][j] == arete[2]:
                return arete[2]
    return 0


def fillWithInfinity():
    """
    we fill every case of the matrix with 999

    """

    matrix_out = []
    for i in range(len(matrice_cout)):
        matrix_out.append([])
        for j in range(len(matrice_cout)):
            matrix_out[i].append(999)
    return matrix_out


def getAdjacent(sommet, matrice_cout):
    """
    return every adjacent of a vertex
    """
    adjacent = []
    for i in range(len(matrice_cout[sommet])):
        if matrice_cout[sommet][i] != 0:
            adjacent.append(i)

    print("adjacent of ", sommet, " : ", adjacent)
    return adjacent


def dijkstrzzza(matrice_cout):
    map_of_marked = {
        0: False,
        1: False,
        2: False,
        3: False,
        4: False,
        5: False,
        6: False,
        7: False,
        8: False,
        9: False,
        10: False,
        11: False,
    }

    aretes = getAretes(matrice_cout)
    V = getSommet(matrice_cout)

    matrix_out = fillWithInfinity()

    T = V

    map_of_marked[0] = True

    def getmin(list_sommets):
        # [1, 8, 9]
        # dans la matrice on regarde les valeurs pour entre 0 et 1, 0 et 8, 0 et 9
        #  on retourne le plus petit sommet de la liste qui est a false dans la map
        min = 10 ^ 10
        minSommet = 0

        for sommet in list_sommets:
            if map_of_marked[sommet] == False:
                if matrix_out[0][sommet] < min:
                    min = matrix_out[0][sommet]
                    minSommet = sommet
        print("minSommet : ", minSommet)
        return minSommet

    matrix_out[0][0] = 0
    source = 0
    while len(V) != 0:
        minSommet = getmin(getAdjacent(source, V[0], matrice_cout))
        map_of_marked[minSommet] = True
        V.remove(minSommet)
        for adj in getAdjacent(minSommet, matrice_cout):
            if (
                matrix_out[0][minSommet] + matrice_cout[minSommet][adj]
                < matrix_out[0][adj]
            ):
                matrix_out[0][adj] = (
                    matrix_out[0][minSommet] + matrice_cout[minSommet][adj]
                )
                matrix_out[1][adj] = minSommet

    return matrix_out


def dijkstra(cost_matrix, start_vertex):
    num_vertices = len(cost_matrix)

    shortest_distances = [float("inf")] * num_vertices
    shortest_distances[start_vertex] = 0

    visited_vertices = [False] * num_vertices

    for _ in range(num_vertices):
        min_distance = float("inf")
        for vertex in range(num_vertices):
            if (
                not visited_vertices[vertex]
                and shortest_distances[vertex] < min_distance
            ):
                min_distance = shortest_distances[vertex]
                min_distance_vertex = vertex

        visited_vertices[min_distance_vertex] = True

        for vertex in range(num_vertices):
            if (
                not visited_vertices[vertex]
                and cost_matrix[min_distance_vertex][vertex]
                and shortest_distances[min_distance_vertex]
                + cost_matrix[min_distance_vertex][vertex]
                < shortest_distances[vertex]
            ):
                shortest_distances[vertex] = (
                    shortest_distances[min_distance_vertex]
                    + cost_matrix[min_distance_vertex][vertex]
                )

    return shortest_distances
    """ 
    matrice_cout : matrice des coûts
        @return  matrice passée par l'algorithme de Dijkstra fait maison
    """


# def calculDiktraEntreDeuxPoints(matrice_cout, pointA, pointB):
#     currentRow = matrice_cout[pointA]
#     allNeighbors = []
#     for i in range(len(currentRow)):
#         if currentRow[i] != 0:
#             if i == pointB:
#                 return currentRow[i]

#             allNeighbors.append([i, currentRow[i]])

#     currentMin = allNeighbors[0]
#     for i in range(len(allNeighbors)):
#         if allNeighbors[i][1] < currentMin[1]:
#             currentMin = allNeighbors[i]


print("Distance entre les villes :")
print(dist_matrix)
print("Matrice des coûts :")

print(matrice_cout)
print("Matrice des prédécesseurs :")


print(predecessors)


dist_matrix, predecessors = shortest_path(
    csgraph=sparse_matrice_cout, directed=False, return_predecessors=True
)


print()
print()
print()
print()
print()

print("-----------------SHORTEST PATH-----------------")
print()
print()
print()
print()

print("Distance entre les villes :")
print(dist_matrix)
print("predecessors : ")
print(predecessors)


def get_shortest_path(start_vertex, end_vertex, predecessors):
    distance = 0
    path = []
    i = end_vertex
    while i != start_vertex:
        path.append(i)
        distance += matrice_cout[i][predecessors[start_vertex][i]]
        i = predecessors[start_vertex][i]
    path.append(start_vertex)
    path.reverse()
    return path, distance


path, distance = get_shortest_path(2, 6, predecessors)
# print("Le chemin le plus court est :", path)
# print("La distance est de :", dist_matrix[2][6])

print()
print()
print()
print()
print("-----------------DIJKSTRA-----------------")

mat_out = dijkstra(matrice_cout, 2)

print(mat_out)
