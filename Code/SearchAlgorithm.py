# This file contains all the required routines to make an A* search algorithm.
#
__authors__ = '1494936'
__group__ = 'DL.15'

# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2016- 2017
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
    # fi = path.last
    path_list = []
    # Creamos una lista auxiliar donde introduciremos los paths expandidos
    aux = path.route
    # Creamos una lista auxiliar en la que guardar la ruta del path a expandir
    aux_g = path.g
    aux_h = path.h
    aux_f = path.f
    # Almacenamos los parámetros del path para reasignarlos en el bucle y que el path extendido también los tenga
    for station in map.connections[path.last]:
        # Iteramos sobre las conexiones de la última estación y añadimos a la ruta la estación de cada iteración
        # concatenándola al auxiliar que guarda el path. Esta concatenación se convierte en path para añadirla
        # finalmente a la lista de paths expandidos
        path_list.append(Path(aux + [station]))
        path_list[-1].g = aux_g
        path_list[-1].h = aux_h
        path_list[-1].f = aux_f
        # Reasignamos el coste, la heurística y el total antiguo al path expandido
    return path_list


"""
ANTIGUA IMPLEMENTACIÓN:
    path_list = []
    # Creamos una lista auxiliar donde introduciremos los paths expandidos
    for conn in map.connections[path.last]:
        # Iteramos sobre las conexiones de la última estación y usamos add_route para añadir cada nueva estación
        # a un "subpath" distinto que se introduce en path_list
        aux = copy.deepcopy(path)  # Originalmente se hacía solo aux=path, pero eso igualaba las posiciones de memoria.
        aux.add_route(conn)
        path_list.append(aux)

    return path_list
"""


def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """

    for path in reversed(path_list):
        # Iteramos sobre la lista de paths expandidos al revés para poder hacer remove con seguridad
        i = 0
        n_estacions = len(path.route)
        fi = False
        while i < n_estacions and fi is False:
            # Comprobamos que ninguna estación aparezca más de una vez en la lista
            if path.route[i] in path.route[i + 1:]:
                # Si un path contiene una estación repetida, eliminamos el path
                fi = True
                path_list.remove(path)
            i = i + 1
    return path_list


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    # Concatenamos la lista de paths a visitar a la lista de paths expandidos (EXP+VISIT)
    # EN BFS ES AL REVÉS
    list_of_path = expand_paths + list_of_path
    return list_of_path


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """

    list_of_path = [Path(origin_id)]  # Creamos una lista que contenga un path formado únicamente por la estación origen

    while list_of_path[0].last != destination_id and list_of_path is not None:
        # Mientras el último elemento del primer path de la lista no sea
        # la estación de destino y la lista no esté vacía:
        c = list_of_path.pop(0)  # Sacamos el primer path de la lista y lo guardamos en c
        e = expand(c, map)  # Expandimos el path c y guardamos la lista resultante en e
        e = remove_cycles(e)  # Eliminamos los ciclos de los paths contenidos en e

        list_of_path = insert_depth_first_search(e, list_of_path)
        # Insertamos los paths expandidos en los paths a visitar siguiendo el criterio DFS

    if list_of_path[0].last == destination_id:
        # Si el último elemento del primer path es el destino,
        # devolvemos el primer path
        print(list_of_path[0].route)
        return list_of_path[0]
    else:
        return "No existeix solució"


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    # Concatenamos la lista de paths expandidos a la lista de paths a visitar (VISIT+EXP)
    # EN DFS ES AL REVÉS
    list_of_path = list_of_path + expand_paths
    return list_of_path


def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    list_of_path = [Path(origin_id)]  # Creamos una lista que contenga un path formado únicamente por la estación origen

    while list_of_path[0].last != destination_id and list_of_path is not None:
        # Mientras el último elemento del primer path de la lista no sea
        # la estación de destino y la lista no esté vacía:
        c = list_of_path.pop(0)  # Sacamos el primer path de la lista y lo guardamos en c
        e = expand(c, map)  # Expandimos el path c y guardamos la lista resultante en e
        e = remove_cycles(e)  # Eliminamos los ciclos de los paths contenidos en e

        list_of_path = insert_breadth_first_search(e, list_of_path)
        # Insertamos los paths expandidos en los paths a visitar siguiendo el criterio BFS

    if list_of_path[0].last == destination_id:
        # Si el último elemento del primer path es el destino,
        # devolvemos el primer path
        return list_of_path[0]
    else:
        return "No existeix solució"


def calculate_cost(expand_paths, map, type_preference):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """

    if type_preference == 0:  # ADYACENCIA => Menor número de estaciones
        for p in expand_paths:
            p.update_g(1)
            # Si expand ha podido añadir la última estación es evidente que están conectadas.
            # Por tanto, en términos de adyacencia simplemente aumentamos 1.

    if type_preference == 1:  # TIEMPO MÍNIMO
        for p in expand_paths:
            # El tiempo viene dado por el diccionario map.connections:
            temps = map.connections[p.penultimate][p.last]
            p.update_g(map.connections[p.penultimate][p.last])

    if type_preference == 2:  # DISTANCIA MÍNIMA => v*t
        for p in expand_paths:
            if map.stations[p.penultimate]['name'] != map.stations[p.last]['name']:
                # El tiempo viene dado por el diccionario map.connections:
                # temps = map.connections[p.penultimate][p.last]

                # La velocidad está en map.velocity [estación][línea],
                # obtenemos la velocidad de map.velocity en función de la velocidad de la
                # línea de la penúltima estación y la multiplicamos por el tiempo:
                # dist = map.velocity[map.stations[p.penultimate]['line']] * map.connections[p.penultimate][p.last]

                p.update_g(map.velocity[map.stations[p.penultimate]['line']] * map.connections[p.penultimate][p.last])

    if type_preference == 3:  # TRANSBORDOS MÍNIMOS
        for p in expand_paths:
            if map.stations[p.penultimate]['line'] != map.stations[p.last]['line']:
                # Si las estaciones no están en la misma línea, sumamos un transbordo
                p.update_g(1)


    return expand_paths


def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """

    # El propósito de esta función es encontrar paths dentro de expand_paths que tengan menor g que las de list_of_path
    # y añadirlos a list_of_path

    list_of_path = list_of_path + expand_paths
    return sorted(list_of_path, key=lambda path: path.g)


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path(origin_id)]  # Creamos una lista que contenga un path formado únicamente por la estación origen

    while list_of_path[0].last != destination_id and list_of_path is not None:
        # Mientras el último elemento del primer path de la lista no sea
        # la estación de destino y la lista no esté vacía:
        c = list_of_path.pop(0)  # Sacamos el primer path de la lista y lo guardamos en c
        e = expand(c, map)  # Expandimos el path c y guardamos la lista resultante en e
        e = remove_cycles(e)  # Eliminamos los ciclos de los paths contenidos en e
        e = calculate_cost(e, map, type_preference)  # Calculamos el coste (g) de cada elemento de e
        list_of_path = insert_cost(e, list_of_path)  # Introducimos los paths de e en llista en función del coste

    if list_of_path[0].last == destination_id:
        # Si el último elemento del primer path es el destino,
        # devolvemos el primer path
        print("LLLLLLLLLLL")
        print(map.connections[list_of_path[0].penultimate][list_of_path[0].last])
        return list_of_path[0]
    else:
        return "No existeix solució"


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            destination_id: 'ID' de la estación que queremos visitar
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """
    ##########################################################################################################
    # RECORDATORIO: LA HEURÍSTICA ES UNA ESTIMACIÓN DEL COSTE QUE FALTA DESDE UNA ESTACIÓN HASTA LA SOLUCIÓN #
    #     | LA H NO PUEDE PASARSE DE LARGO EN NINGÚN CASO => SIEMPRE NOS PONEMOS EN LA MEJOR SITUACIÓN |     #
    #    EN TODOS LOS CASOS LA HEURÍSTICA ES 0 SI EL DESTINO ESTÁ EN EL PATH PORQUE PODRÍAMOS ESTAR EN ÉL    #
    ##########################################################################################################

    if type_preference == 0:  # ADYACENCIA => Destino en path = 0 | Destino NO en path = 1
        for p in expand_paths:
            if destination_id not in p.route:
                # Si el destino no está en el path, la heurística es 1: No podemos estar en esa estación, pero
                # puede estar a un salto de distancia
                p.update_h(1)
            else:  # Si el destino está en el path, la heurística es 0: Podríamos estar en el destino
                p.update_h(0)

    if type_preference == 1:  # TIEMPO MÍNIMO => Destino en path = 0 | Destino fuera de path = t_min = espacio/vel_max
        # Para la entrega final se ha optimizado la forma de obtener la velocidad máxima:
        vel_max = max(map.velocity.values())
        for p in expand_paths:
            if destination_id in p.route:  # Si el destino está en el path, la heurística es 0
                p.update_h(0)
            else:
                # Antigua forma de obtener la velocidad máxima:
                """
                vel_max = map.stations[1]['velocity']  # Establecemos una velocidad máxima
                for i in range(2, len(map.stations)):
                    # Iteramos sobre la velocidad de cada estación para obtener
                    # la velocidad máxima, igual que en coord2station
                    vel = map.stations[i]['velocity']
                    if vel > vel_max:
                        vel_max = vel
                """
                """
                temps = euclidean_dist([map.stations[p.last]['x'], map.stations[p.last]['y']],
                                       [map.stations[destination_id]['x'], map.stations[destination_id]['y']]) / vel_max
                """
                # Finalmente calculamos el tiempo mínimo como espacio/vel_max y actualizamos la heurística
                p.update_h(euclidean_dist([map.stations[p.last]['x'], map.stations[p.last]['y']],
                                          [map.stations[destination_id]['x'],
                                           map.stations[destination_id]['y']]) / vel_max)

    if type_preference == 2:  # DISTANCIA MÍNIMA => Destino en path = 0 | Destino fuera de path = Distancia euclidiana
        for p in expand_paths:
            if destination_id in p.route:
                p.update_h(0)
            else:
                # La heurística es la distancia euclidiana, la mínima distancia posible:
                p.update_h(euclidean_dist([map.stations[p.last]['x'], map.stations[p.last]['y']],
                                          [map.stations[destination_id]['x'], map.stations[destination_id]['y']]))

    if type_preference == 3:  # TRANSBORDOS MÍNIMOS => Destino en path = 0
        # Destino fuera de path y estaciones penúltima y última de DISTINTAS LÍNEAS = 1
        # Destino fuera de path y estaciones penúltima y última de la MISMA LÍNEA = pass
        for p in expand_paths:
            if destination_id in p.route:
                p.update_h(0)  # Si el destino está en el path, la heurística es 0
            else:
                if map.stations[p.last]['line'] != map.stations[p.penultimate]['line']:
                    p.update_h(1)
                    # Si el destino NO está en el path y las estaciones penúltima y última
                    # están en líneas distintas (hay que hacer al menos un transbordo),
                    # la heurística es 1
                else:
                    pass
                    # ________________________________________________________________________ #
                    # ¡OJO! Si el destino NO está en el path pero las estaciones penúltima y   |
                    # última están en la misma línea no hay heurística de transbordos válida:  |
                    # ------------------------------------------------------------------------ #
                    # Es imposible que sea la estación actual (heurística 0) pero tampoco      |
                    # podríamos hacer transbordo (heurística 1) => Error de cómputo            |
                    # ------------------------------------------------------------------------ #
                    # Es una situación que se produce varias veces en el test, con esta        |
                    # configuración el resultado es correcto.                                  |
                    # ________________________________________________________________________ #
    return expand_paths


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    # f = Coste total de un path (cost + heuristics)
    # Ellos nos dan una función exactamente igual hecha en SubwayMap.py pero sin retorno, no sé por qué.
    for path in expand_paths:
        path.f = path.g + path.h
    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not an optimal solution!
      If a station is visited and have a lower g in this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
    """


    for p1 in reversed(expand_paths):
        # Iteramos sobre la lista de paths expandidos al revés para poder hacer remove sobre el bucle con seguridad
        if p1.last not in visited_stations_cost:
            # Si la última estación de un path no está en el diccionario de costes g por
            # estación visitada, añadimos el coste al diccionario (p1.g)
            visited_stations_cost[p1.last] = p1.g
        elif visited_stations_cost[p1.last] <= p1.g:
            # Como indica el enunciado superior, si una estación visitada tiene un coste menor o igual que
            # el de algún path expandido, tenemos que eliminar dicho path de la lista expand_paths
            expand_paths.remove(p1)
        else:
            # Si la última estación del path está visitada y tiene un coste mayor que la de algún
            # path expandido, sustituimos el valor anterior por el del path expandido.
            visited_stations_cost[p1.last] = p1.g
            for p2 in reversed(list_of_path):
                # Adicionalmente, iteramos sobre los paths no expandidos para eliminar cualquier path (p2) que contenga
                # la última estación del path expandido (p1), pues ya hemos conseguido una forma de llegar
                # que por fuerza es más eficiente => ELIMINAMOS REDUNDANCIA
                if p1.last in p2.route:
                    list_of_path.remove(p2)
    return expand_paths, list_of_path, visited_stations_cost


def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    for p in expand_paths:
        # Iteramos sobre la lista de paths expandidos
        trobat = False
        i = 0
        if len(list_of_path) != 0:
            while i < len(list_of_path) and trobat is not True:
                # Iteramos sobre la lista de paths visitados. Si el path expandido que
                # se está tratando tiene menor coste total f que un path visitado,
                # se inserta el path expandido en la posición i de la lista de visitados:
                if p.f < list_of_path[i].f:
                    # Si lo encontramos, insertamos el path en la posición i de la lista de visitados y termina el while
                    list_of_path.insert(i, p)
                    trobat = True
                else:
                    # Si no lo encontramos, i crece y pasamos al siguiente elemento de la lista de paths
                    i = i + 1

        if trobat is False:
            # Si no encontramos ningún path VISITADO con MAYOR coste total f que el path
            # EXPANDIDO en cuestión, añadimos el path expandido al final de la lista de visitados
            list_of_path.append(p)

    return list_of_path


def coord2station(coord, map):
    """
        From coordinates, it searches the closest station.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            possible_origins (list): List of the Indexes of stations, which corresponds to the closest station
    """

    min_dist = euclidean_dist([map.stations[1]['x'], map.stations[1]['y']], coord)
    # Asignamos la distancia mínima a la que existe entre la primera estación y las coordenadas proporcionadas,
    # consiguiendo así un valor para min_dist y ahorrándonos de paso una iteración
    possible_origins = [1]
    # Por si fuera un posible origen, añadimos la primera estación a la lista possible_origins

    for i in range(2, len(map.stations)+1):
        # Iteramos sobre las IDs de las estaciones, desde 2 hasta el número de estaciones + 1 para recorrer todas
        # Puesto que las IDs son números, "range" es una herramienta perfecta para estas iteraciones
        dist = euclidean_dist([map.stations[i]['x'], map.stations[i]['y']], coord)
        # La distancia viene dada por la función euclidean_dist, entre una lista inline formada por las componentes
        # x e y de cada estación y las coordenadas proporcionadas (otra lista con x e y)
        if dist < min_dist:
            # Si la distancia calculada es menor a la distancia mínima establecida, borramos la lista de posibles
            # estaciones de origen y añadimos la id de la nueva estación, que lógicamente coincide con i
            # Además, asignamos la distancia calculada a la variable min_dist para establecerla como mínima
            possible_origins.clear()
            possible_origins.append(i)
            min_dist = dist
        elif dist == min_dist:
            # Si la distancia calculada es igual a la distancia mínima, añadimos la estación cuya id viene dada por i
            # a la lista de posibles orígenes (RECORDATORIO: possible_origins es una lista que contiene las
            # estaciones a situadas menor distancia)
            possible_origins.append(i)

    return possible_origins


def Astar(origin_coor, dest_coor, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:

            # NOTA IMPORTANTE DE 1494936, ANTERIORMENTE LAS DOS PRIMERAS LÍNEAS ERAN:
            # origin_id (list): Starting station id
            # destination_id (int): Final station id
            # ¡PERO NO RECIBIMOS ESTO!

            # LAS HE CAMBIADO POR LO QUE REALMENTE RECIBIMOS:
            origin_coor (list): Starting station coordinates
            dest_coor (list): Final station coordinates

            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    origin_id = coord2station(origin_coor, map)[0]
    # Tomamos como origin_id la primera estación devuelta por coord2station con las coordenadas origin_coor
    destination_id = coord2station(dest_coor, map)[0]
    # Tomamos como destination_id la primera estación devuelta por coord2station con las coordenadas dest_coor
    # En ambos casos se toma la primera simplemente por convenio

    c = Path(origin_id)  # Guardamos el origin_id como única estación en el path c
    list_of_path = [c]  # Creamos una lista que contenga un path formado únicamente por la estación origen en path (c)
    visited_stations_cost = {origin_id: 0}
    # Creamos visited_stations_cost del remove_redundant_paths como un diccionario que inicialmente solo incluye
    # el coste total f hasta la estación origen (coste 0)
    while c.route[-1] != destination_id and list_of_path is not None:
        # Mientras el último elemento del primer path de la lista no sea
        # la estación de destino y la lista no esté vacía:
        c = list_of_path.pop(0)  # Sacamos el primer path de la lista y lo guardamos en c
        e = expand(c, map)  # Expandimos el path c y guardamos la lista resultante en e
        e = remove_cycles(e)  # Eliminamos los ciclos de los paths contenidos en e
        e = calculate_cost(e, map, type_preference)
        # Calculamos el coste g (sin tener en cuenta destino) a partir de e y lo volvemos a guardar en e
        e = calculate_heuristics(e, map, destination_id, type_preference)
        # Calculamos la heurística h (conociendo el destination_id) a partir de e y lo volvemos a guardar en e
        e = update_f(e)  # Almacenamos en e el contenido de e con el coste total f actualizado

        expand_list, list_of_path, visited_stations_cost = remove_redundant_paths(e, list_of_path,
                                                                                  visited_stations_cost)
        # Eliminamos los paths redundantes de las tres listas de paths

        list_of_path = insert_cost_f(e, list_of_path)
        # Finalmente, insertamos e en list of path según el contenido de f

    if c.route[-1] == destination_id:
        # Si la última estación contenida en c es el destino, devolvemos c
        return c
    else:
        return "No existeix solució"


