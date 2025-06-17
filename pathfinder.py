import math

def Lenght(v1,v2):
    return math.sqrt((v1.x - v2.x) ** 2 + (v1.y - v2.y) ** 2)

def choose_node (reachable,stop):# вспомогательная функция для А* (выбор узла)
    min_cost = float('inf')
    best_node = None

    for node in reachable:
        cost_start_to_node = node.cost
        cost_node_to_goal = Lenght(node,stop)
        total_cost = cost_start_to_node + cost_node_to_goal

        if min_cost > total_cost:
            min_cost = total_cost
            best_node = node

    return best_node

def find_edge(v1, v2):
    for edge in v1.edges:
        if edge.V1 == v2 or edge.V2 == v2:
            return edge
    return None

def build_path(to_node):#возврат к изначальному узлу.полный путь
    path = []
    edges = []
    while to_node != None:
        path.append(to_node)
        if to_node.previous is not None:
            edges.append(find_edge(to_node, to_node.previous))
        to_node = to_node.previous
    return path, edges

def Find(node,explored):#ищем верщины ,которые не исследованные ,из данной сейчас вершины
    found=[]
    for edge in node.edges:
        v=None
        if node != edge.V1:
            v = edge.V1
        else:
            v = edge.V2
        if v not in explored:
            found.append(v)
    return found

def A_star(start, stop):#алгоритм А*
    start.cost = 0
    reachable = [start]
    explored = []

    while len(reachable) > 0:
        node = choose_node(reachable,stop)

        if node == stop:
            print(node.cost)
            return build_path(stop)


        reachable.remove(node)
        explored.append(node)

        new_reachable = Find(node,explored)

        for adjacent in new_reachable:
            if adjacent not in reachable:
                reachable.append(adjacent)

            if node.cost + Lenght(node,adjacent) < adjacent.cost:
                adjacent.previous = node
                adjacent.cost = node.cost + Lenght(node,adjacent)
    return None

