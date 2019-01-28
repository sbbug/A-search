
#这是一个英文版的作业:DFS IDDFS A*
class Node:
    """
    This class describes a single node contained within a graph.
    It has the following instannce level attributes:

    ID: An integer id for the node i.e. 1
    heuristic_cost: A float value representing the estimated
                    cost to the goal node
    """

    def __init__(self, ID, heuristic_cost):
        self.ID = ID
        self.connected_nodes = []
        self.heuristic_cost = heuristic_cost

    def __repr__(self):
        ID = self.ID
        hx = self.heuristic_cost
        if len(self.connected_nodes) == 0:
            nodes = 'None'
        else:
            nodes = ','.join(str(cn[1].ID) for cn in self.connected_nodes)
        return 'Node:{}\nh(n):{}\nConnected Nodes:{}'.format(ID, hx, nodes)

    def set_connected_nodes(self, connected_nodes):
        """
        Adds edges that lead from this node to other nodes:

        Parameters:
        - connected_nodes: A list of tuples consisting of (cost, Node),
                           where 'cost' is a floating point value
                           indicating the cost to get from this node
                           to 'Node' and 'Node' is a Node object
        """
        self.connected_nodes = connected_nodes


def build_graph():
    """
    Builds the graph to be parsed by the search algorithms.
    Returns: The starting node, which is the entry point into the graph
    """
    ids = range(13)
    coords = [(0, 0), (1, 1), (1, 0), (1, 1), (5, 2), (3, 1), (3, 0),
              (3, -1), (5, 1), (4, 1), (4, 0), (4, -2), (7, 0)]

    # https://en.wikipedia.org/wiki/Euclidean_distance
    euclidean_distance = lambda x1y1, x2y2: ((x1y1[0] - x2y2[0]) ** 2 + (x1y1[1] - x2y2[1]) ** 2) ** (0.5)

    def build_connected_node_list(from_id, to_ids):
        starting_coords = coords[from_id]

        connected_nodes = []
        for to_id in to_ids:
            connected_nodes.append((euclidean_distance(starting_coords, coords[to_id]), all_nodes[to_id]))

        return connected_nodes

    goal_coords = (7, 0)
    all_nodes = [Node(_id, euclidean_distance(coord, goal_coords)) for _id, coord in zip(ids, coords)]

    all_nodes[8].set_connected_nodes(build_connected_node_list(8, [12]))
    all_nodes[10].set_connected_nodes(build_connected_node_list(10, [12]))
    all_nodes[5].set_connected_nodes(build_connected_node_list(5, [8]))
    all_nodes[6].set_connected_nodes(build_connected_node_list(6, [9, 10]))
    all_nodes[7].set_connected_nodes(build_connected_node_list(7, [11]))
    all_nodes[1].set_connected_nodes(build_connected_node_list(1, [4, 5]))
    all_nodes[2].set_connected_nodes(build_connected_node_list(2, [5, 6]))
    all_nodes[3].set_connected_nodes(build_connected_node_list(3, [7]))
    all_nodes[0].set_connected_nodes(build_connected_node_list(0, [1, 2, 3]))

    return all_nodes[0]


def depth_first_search(starting_node, goal_node):
    """
    This function implements the depth first search algorithm

    Parameters:
    - starting_node: The entry node into the graph
    - goal_node: The integer ID of the goal node.

    Returns:
    A list containing the visited nodes in order they were visited with starting node
    always being the first node and the goal node always being the last
    """
    visited_nodes_in_order = []
    frontier = [starting_node]
    while frontier != []:
        current = frontier.pop(0)

        visited_nodes_in_order.append(current.ID)
        # print(visited_nodes_in_order)
        if current.ID == goal_node:
            #print(visited_nodes_in_order)
            return visited_nodes_in_order
        for n in reversed(current.connected_nodes):
            if n not in frontier and visited_nodes_in_order:
                frontier.insert(0, n[1])

    return None
# 来自给定源的“src”
paths = []
def DLS(src, target, maxDepth):
    paths.append(src.ID)
    if src.ID == target: return True

    # 如果达到最大深度，停止递归
    if maxDepth <= 0:
        return False

    # 对所有与这个顶点相邻的顶点重复
    for node in src.connected_nodes:
        if (DLS(node[1], target, maxDepth - 1)):
            return True

    return False

    # 搜索目标是否可以从v中到达
    # 它使用递归DLS()
def iterative_deepening_depth_first_search(starting_node,goal_node):

    # 重复深度限制搜索，直到最大深度
    for i in range(5):
        DLS(starting_node, goal_node, i)

    return paths


def a_star_search(starting_node, goal_node):
    """
    This function implements the A* search algorithm

    Parameters:
    - starting_node: The entry node into the graph
    - goal_node: The integer ID of the goal node.

    Returns:
    A list containing the visited node ids in order they were visited with starting node
    always being the first node and the goal node always being the last
    """
    visited_nodes_in_order = []
    #开列表
    open_lists = []
    #闭列表
    close_lists = []

    #将开始值加入闭列表
    close_lists.append(starting_node)
    visited_nodes_in_order.append(starting_node.ID)
    min_node = starting_node
    now_id = min_node.ID

    for node in min_node.connected_nodes:
        open_lists.append(node)

    #开始搜索
    while goal_node!=now_id:
        # 从开列表中寻找代价最小的值
        min = 1000.0
        temp = None
        for n in open_lists:
            if (n[0]+n[1].heuristic_cost)<min:
                min = n[0]+n[1].heuristic_cost
                temp = n

        if temp!=None:
            close_lists.append(temp)
            open_lists.remove(temp)
            visited_nodes_in_order.append(temp[1].ID)
            now_id = temp[1].ID

            for node in temp[1].connected_nodes:
                open_lists.append(node)


    return visited_nodes_in_order
if __name__ == "__main__":
    goal_node = 12
    depth_first_search_answer = [0, 1, 4, 5, 8, 12]
    iterative_deepening_depth_first_search_answer = [0, 0, 1, 2, 3, 0, 1,
                                                     4, 5, 2, 5, 6, 3, 7,
                                                     0, 1, 4, 5, 8, 2, 5,
                                                     8, 6, 9, 10, 3, 7, 11,
                                                     0, 1, 4, 5, 8, 12]
    a_star_search_answer = [0, 2, 6, 10, 12]

    assert (depth_first_search(build_graph(), goal_node) == depth_first_search_answer)
    assert (iterative_deepening_depth_first_search(build_graph(),goal_node) == iterative_deepening_depth_first_search_answer)
    assert (a_star_search(build_graph(), goal_node) == a_star_search_answer)