'''
Gordian Bruns, Dipesh Poudel
CS365 Lab A
'''

''' File:    main.py
 *  Purpose: Find shortest path in a maze
 *
 *  Input:   filename, search-algorithm
 *  Output:  Solved maze, number of expanded node, and cost of the path
 *
 *  Usage: python main.py [filename] [search-algorithm]
 *
 *  Note:  search-algorithm = {dfs, bfs, gbfs, astar, m_astar}
 '''

import sys  # for reading the command line
import networkx as nx  # graph for the state_representation
import queue  # for the searching algorithms
import copy  # for deepcopy
from mouse import Mouse  # the mouse we are moving around


def main(argv):
    try:
        filename = sys.argv[1]
        search = sys.argv[2]
    except IndexError:
        print("Give two command line arguments! filename search-algorithm")
        exit(0)

    if search == "dfs":
        single_dfs(filename)
    elif search == "bfs":
        single_bfs(filename)
    elif search == "gbfs":
        single_gbfs(filename)
    elif search == "astar":
        single_astar(filename)
    elif search == "m_astar":
        multi_astar(filename)
    else:
        print("Error! Choose one of the following search algorithms: dfs, bfs, gbfs, astar, m_astar.")
        exit(0)
'''  main  '''


''' Function:    file_read
 *  Purpose:     reads in given file
 *  Input args:  filename <String>
 *  Return val:  representation of the maze <nested array>, start <tuple>, goals <array of tuples>
'''
def file_read(filename):
    try:
        f = open(filename, "r")
    except IOError:
        print("File does not exist")
        exit(0)
    line = f.readline()

    line_num = file_len(filename)
    column_num = len(line)
    content_list = [[0 for i in range(column_num)] for j in range(line_num)]
    linenum = 0
    start = ()
    goals = []

    while True:
        if not line:
            break

        for sign in range(len(line)):
            if line[sign] == "P":
                start = (linenum, sign)
            if line[sign] == ".":
                goal = (linenum, sign)
                goals.append(goal)
            if line[sign] != '\n':
                content_list[linenum][sign] = line[sign]

        line = f.readline()
        linenum += 1

    for i in range(linenum):
        content_list[i].remove(0)

    f.close()
    return content_list, start, goals
'''  file_read  '''


''' Function:    file_len
 *  Purpose:     determines number of lines of a file (helper function of file_read)
 *  Input args:  filename <String>
 *  Return val:  number of lines <int>
'''
def file_len(filename):
    with open(filename) as f:
        for i, l in enumerate(f):
            pass
    return i + 1
'''  file_len  '''


''' Function:    transition_function
 *  Purpose:     moves the mouse object
 *  Input args:  mouse <object Mouse>, path <list>, goals <list>
 *  Return val:  mouse <object Mouse> with updated attributes
'''
def transition_function(mouse, path, goals):
    mouse = Mouse(path[0])
    for i in range(len(path)):
        if i == 0:
            continue
        previous = path[i - 1]
        position = path[i]

        if position == (previous[0] - 1, previous[1]):
            mouse.goNorth()

        if position == (previous[0], previous[1] + 1):
            mouse.goEast()

        if position == (previous[0] + 1, previous[1]):
            mouse.goSouth()

        if position == (previous[0], previous[1] - 1):
            mouse.goWest()

        if position in goals:
            mouse.get_goal(position)
    return mouse
'''  transition_function  '''


''' Function:    state_representation
 *  Purpose:     converts given nested array into a graph
 *  Input args:  maze <nested list>
 *  Return val:  graph <networkx graph>
'''
def state_representation(maze):  # state representation using networkx's graph
    graph = nx.Graph()
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] != '%' and maze[i][j] not in graph:
                graph.add_node((i, j))
                if maze[i - 1][j] != '%' and maze[i - 1][j] not in graph:
                    graph.add_node((i - 1, j))
                    graph.add_edge((i, j), (i - 1, j))
                if maze[i][j + 1] != '%' and maze[i][j + 1] not in graph:
                    graph.add_node((i, j + 1))
                    graph.add_edge((i, j), (i, j + 1))
                if maze[i + 1][j] != '%' and maze[i + 1][j] not in graph:
                    graph.add_node((i + 1, j))
                    graph.add_edge((i, j), (i + 1, j))
                if maze[i][j - 1] != '%' and maze[i][j - 1] not in graph:
                    graph.add_node((i, j - 1))
                    graph.add_edge((i, j), (i, j - 1))
    return graph
'''  state_representation  '''


''' Function:    singledfs
 *  Purpose:     find solution for a maze
 *  Input args:  filename <String>
 *  Return val:  mouse <object Mouse> with updated attributes
'''
def singledfs(filename):
    maze, start, goals = file_read(filename)
    graph = state_representation(maze)
    mouse = Mouse(start)

    visited, stack = set(), [start]
    path = []
    path_dict = {}
    count = 0
    while stack:
        vertex = stack.pop()
        if vertex not in visited:
            visited.add(vertex)
            neighborhood = graph.neighbors(vertex)
            for neighbor in neighborhood:
                count += 1
                if neighbor not in visited:
                    path_dict[neighbor] = vertex
                    stack.append(neighbor)
                if neighbor in goals:
                    position = neighbor
                    path.append(neighbor)
                    while position != start:
                        position = path_dict[position]
                        path.append(position)

    mouse = transition_function(mouse, path[::-1], goals)
    solution(maze, path[::-1], count)
    return mouse
'''  singledfs  '''


''' Function:    single_dfs
 *  Purpose:     find solution for a maze
 *  Input args:  filename <String>
 *  Return val:  mouse <object Mouse> with updated attributes
'''
def single_dfs(filename):
    maze, start, goals = file_read(filename)
    graph = state_representation(maze)
    mouse = Mouse(start)

    visited, stack = set(), [start]
    path = []
    path_dict = {}
    count = 0
    while stack:
        vertex = stack.pop()
        if vertex in goals:
            position = vertex
            path.append(vertex)
            while position != start:
                position = path_dict[position]
                path.append(position)
        count += 1
        if vertex in visited:
            continue
        visited.add(vertex)
        neighborhood = graph.neighbors(vertex)
        for neighbor in neighborhood:
            if neighbor not in visited:
                path_dict[neighbor] = vertex
                stack.append(neighbor)

    mouse = transition_function(mouse, path[::-1], goals)
    solution(maze, path[::-1], count)
    return mouse
'''  single_dfs  '''


''' Function:    single_bfs
 *  Purpose:     find solution for a maze
 *  Input args:  filename <String>
 *  Return val:  mouse <object Mouse> with updated attributes
'''
def single_bfs(filename):
    maze, start, goals = file_read(filename)
    graph = state_representation(maze)
    mouse = Mouse(start)

    visited, queue = [], [start]
    path_dict = {}
    count = 0
    while queue:
        vertex = queue.pop(0)
        visited.append(vertex)
        neighborhood = graph.neighbors(vertex)
        for neighbor in neighborhood:
            count += 1
            if neighbor not in visited and neighbor not in queue:
                path_dict[neighbor] = vertex
                queue.append(neighbor)
            if neighbor in goals:
                position = neighbor
                path = [neighbor]
                while position != start:
                    position = path_dict[position]
                    path.append(position)

    mouse = transition_function(mouse, path[::-1], goals)
    solution(maze, path[::-1], count)
    return mouse
'''  single_bfs  '''


''' Function:    single_gbfs
 *  Purpose:     find solution for a maze
 *  Input args:  filename <String>
 *  Return val:  mouse <object Mouse> with updated attributes
'''
def single_gbfs(filename):
    maze, start, goals = file_read(filename)
    graph = state_representation(maze)
    mouse = Mouse(start)

    visited = set()
    path_dict = {}
    pqueue = queue.PriorityQueue(maxsize=1000000)
    pqueue.put((0, start))
    count = 0
    goal = goals[0]
    while pqueue:
        element = pqueue.get()
        vertex = element[1]
        visited.add(vertex)
        if vertex == goal:
            position = vertex
            path = [vertex]
            while position != start:
                position = path_dict[position]
                path.append(position)
            break
        for neighbor in set(graph.neighbors(vertex)) - visited:
            count += 1
            visited.add(neighbor)
            path_dict[neighbor] = vertex
            pqueue.put((m_heuristic(neighbor, goal), neighbor))

    mouse = transition_function(mouse, path[::-1], goals)
    solution(maze, path[::-1], count)
    return mouse
'''  single_gbfs  '''


''' Function:    single_astar
 *  Purpose:     find solution for a maze
 *  Input args:  filename <String>
 *  Return val:  mouse <object Mouse> with updated attributes
'''
def single_astar(filename):
    maze, start, goals = file_read(filename)
    graph = state_representation(maze)
    mouse = Mouse(start)

    goal = goals[0]
    open_list = [(start, 0, m_heuristic(start, goal))]
    closed_list = []
    came_from = {start: None}
    count = 0
    while open_list:
        myList = []
        for i in open_list:
            myList.append((i[0], i[1] + i[2]))
        position = min(myList, key=lambda t: t[1])[0]

        if position in goals:
            path = [position]
            while position != start:
                prev_loc = position
                position = came_from[prev_loc]
                path.append(position)
            break

        for i in open_list:
            if i[0] == position:
                position_cost = i[1]

        open_list = [(node, g, h) for (node, g, h) in open_list if node != position]

        closed_list.append(position)

        for i in graph.neighbors(position):
            count += 1
            if i in closed_list:
                continue
            if i in open_list:
                new_g = position_cost + 1
                if new_g < i[1]:
                    i[1] = new_g
                    came_from[i] = position
            else:
                g_val = position_cost + 1
                h_val = m_heuristic(i, goal)
                came_from[i] = position
                open_list.append((i, g_val, h_val))

    mouse = transition_function(mouse, path[::-1], goals)
    solution(maze, path[::-1], count)
    return mouse
'''  single_astar  '''


''' Function:    multi_astar
 *  Purpose:     find solution for a maze
 *  Input args:  filename <String>
 *  Return val:  mouse <object Mouse> with updated attributes
'''
def multi_astar(filename):
    maze, start, goals = file_read(filename)
    graph = state_representation(maze)
    mouse = Mouse(start)

    came_from = {start: (None, copy.deepcopy(goals))}
    goals_obtained = {start: (None, [])}

    open_list = [(start, 0, gs_heuristic(start, came_from[start][1]))]
    closed_list = []
    count = 0
    while open_list:
        position = min(open_list, key=lambda t: t[1] + t[2])[0]
        position_cost = min(open_list, key=lambda t: t[1] + t[2])[1]
        count += 1

        if position in came_from[position][1]:
            came_from[position][1].remove(position)
            goals_obtained[position][1].append(position)

            if len(came_from[position][1]) == 0:
                if len(goals_obtained[position][1]) < 10:
                    for i in range(len(maze)):
                        for j in range(len(maze[i])):
                            for k in range(len(goals_obtained[position][1])):
                                if goals_obtained[position][1][k] == (i, j):
                                    maze[i][j] = str(k + 1)
                else:
                    for i in range(len(maze)):
                        for j in range(len(maze[i])):
                            for k in range(len(goals_obtained[position][1])):
                                if goals_obtained[position][1][k] == (i, j):
                                    if k < 10:
                                        maze[i][j] = str(k)
                                    elif k >= 10:
                                        maze[i][j] = chr(k + 87)
                                    elif k >= 36:
                                        maze[i][j] = chr(k + 29)
                for i in maze:
                    print(''.join(i))
                print("Total cost of the path: ", position_cost)
                print("Total number of nodes explored: ", count)

                mouse.position = position
                mouse.goals_obtained = goals_obtained[position][1]
                return mouse

        open_list = [(node, g, h) for (node, g, h) in open_list if node != position]
        closed_list.append((position, came_from[position][1]))

        for i in graph.neighbors(position):

            if (i, came_from[position][1]) in closed_list:
                continue

            for j in open_list:
                new_g = position_cost + 1
                if len(came_from[j[0]][1]) > len(came_from[position][1]) and j[1] < new_g:
                    open_list.remove(j)
                    closed_list.append((j[0], came_from[j[0]][1]))
                    continue
            else:
                g_val = position_cost + 1
                h_val = gs_heuristic(position, came_from[position][1])
                came_from[i] = (position, copy.deepcopy(came_from[position][1]))
                goals_obtained[i] = (position, copy.deepcopy(goals_obtained[position][1]))
                open_list.append((i, g_val, h_val))
'''  multi_astar  '''


''' Function:    m_heuristic
 *  Purpose:     calculate estimated distance to a goal
 *  Input args:  vertex <tuple>, goal <tuple>
 *  Return val:  heuristic <int>
'''
def m_heuristic(vertex, goal):
    heuristic = abs(goal[0] - vertex[0]) + abs(goal[1] - vertex[1])
    return heuristic
'''  m_heuristic  '''


''' Function:    closest_goal
 *  Purpose:     calculate which goal is the closest one (helper function of gs_heuristic)
 *  Input args:  vertex <tuple>, goals <list>
 *  Return val:  closest goal <tuple>
'''
def closest_goal(vertex, goals):
    closest_list = []
    for goal in goals:
        closest_list.append((goal, m_heuristic(vertex, goal)))
    return min(closest_list, key=lambda t: t[1])[0]
'''  closest_goal  '''


''' Function:    gs_heuristic
 *  Purpose:     compute the best option for multi_astar
 *  Input args:  position <tuple>, goals <list>
 *  Return val:  heuristic <int>
'''
def gs_heuristic(position, goals):
    node = position
    goal_list = copy.deepcopy(goals)
    heuristic_counter = 0
    while goal_list:
        for goal in goal_list:
            closest = closest_goal(node, goal_list)
            heuristic_counter += m_heuristic(node, closest)
            node = closest
            goal_list.remove(closest)
    return heuristic_counter
'''  gs_heuristic  '''


''' Function:    solution
 *  Purpose:     print the solved maze
 *  Input args:  maze <nested list>, path <list>, nodes <int>
 *  Return val:  None
'''
def solution(maze, path, nodes):
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            for k in path:
                if k == (i, j):
                    maze[i][j] = '#'
    for box in maze:
        print(''.join(box))
    print("Cost of the path:", len(path))
    print("Number of nodes expanded:", nodes)
'''  solution  '''

if __name__ == "__main__":
    main(sys.argv[:2])
