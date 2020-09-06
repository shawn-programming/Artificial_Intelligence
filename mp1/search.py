# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)

import heapq as hq


def backTrace(state, destination, start):
    answer = [destination]

    curr = destination

    if destination != start:
        while(state[curr[0]][curr[1]] != start):
            curr = state[curr[0]][curr[1]]
            answer.insert(0, curr)
        answer.insert(0, state[curr[0]][curr[1]])

    return answer


def backTraceArray(recorded, start, end):
    answer = []
    curr = end
    
    answer.insert(0, curr)

    while(recorded):
        if(recorded[-1][1] == curr):
            curr = recorded[-1][0]
            answer.insert(0, curr[:2])
        recorded.pop()
    
    return answer[1:]
    
def manhanttan(curr, dest):
    return abs(curr[0]-dest[0]) + abs(curr[1]-dest[1])

def find_nearest(curr, goals):

    if not goals:
        return [-1,-1]

    nearest_val = manhanttan(curr, goals[0])
    answer = goals[0]

    for goal in goals[1:]:
        if(nearest_val > manhanttan(curr, goal)):
            nearest_val = manhanttan(curr, goal)
            answer = goal
    
    return answer

def mst(fruits):
    if(not fruits):
        return 0
    cost = []
    total = 0
    visited = [False] * len(fruits)

    for f1 in fruits:
        row = []
        for f2 in fruits:
            row.append(manhanttan(f1, f2))
        cost.append(row)
    
    heap_q = []

    hq.heappush(heap_q, (0, 0))

    while heap_q:
        dist, curr = hq.heappop(heap_q)

        if not visited[curr]:
            total += dist
            visited[curr] = True
            for i in range(len(fruits)):
                if not visited[i]:
                    hq.heappush(heap_q, (cost[curr][i], i))
                    
    return total


def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_corner": astar_corner,
        "astar_multi": astar_multi,
        "fast": fast,
    }.get(searchMethod)(maze)


def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here

    path = []
    # [row][col]
    state = []

    for _ in range(maze.rows):
        row = []
        for _ in range(maze.cols):
            row.append([-1, -1])
        state.append(row)

    queue = []
    goals = maze.getObjectives()

    curr = maze.getStart()
    r = curr[0]
    c = curr[1]

    queue.append(curr)
    # change the starting node's state
    state[r][c] = curr

    while(queue):
        curr = queue.pop(0)

        r = curr[0]
        c = curr[1]
        neighbors = maze.getNeighbors(r,c)

        # found the goal
        if(curr in goals):
            path = backTrace(state, curr, maze.getStart())
            return path

        # add possible paths
        for node in neighbors:
            r = node[0]
            c = node[1]
            if(maze.isValidMove(r, c) and state[r][c] == [-1, -1]):
                state[r][c] = [curr[0], curr[1]]
                queue.append(node)
        
    return path

# def astar(maze):
#     """
#     Runs A star for part 1 of the assignment.

#     @param maze: The maze to execute the search on.

#     @return path: a list of tuples containing the coordinates of each state in the computed path
#     """
#     # TODO: Write your code here

#     path = []
#     state = []
#     visited = []

#     for _ in range(maze.rows):
#         row = []
#         visited_row = []
#         for _ in range(maze.cols):
#             row.append([-1,-1])
#             visited_row.append(False)
#         state.append(row)
#         visited.append(visited_row)
    
#     heap_q = []
    
#     goals = maze.getObjectives()

#     start = maze.getStart()

#     curr = start
#     state[curr[0]][curr[1]] = curr
#     hq.heappush(heap_q, (0 + manhanttan(curr, goals[0]), curr))

#     while heap_q:
#         dist, curr = hq.heappop(heap_q)

#         # if visited, ignore it
#         if(visited[curr[0]][curr[1]]):
#             continue
#         # make it visited
#         visited[curr[0]][curr[1]] = True

#         # if the goal is found, return the path
#         if(curr in goals):
#             path = backTrace(state, curr, maze.getStart())
#             return path

#         neighbors = maze.getNeighbors(curr[0], curr[1])

#         for node in neighbors:
#             r = node[0]
#             c = node[1]
#             if(maze.isValidMove(r, c) and visited[r][c] == False):
#                 state[r][c] = [curr[0], curr[1]]
#                 hq.heappush(heap_q, (dist+1+manhanttan(node, goals[0]), node))

#     return path

def astar(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    path = []
    record = []
    state = {}
    heap_q = []
    
    goals = maze.getObjectives()
    prev = maze.getStart()
    start = maze.getStart()
    goal = goals[0]

    curr = start
    hq.heappush(heap_q, (manhanttan(curr, goals[0]), 0, curr, prev))

    while heap_q:

        _, dist, curr, prev = hq.heappop(heap_q)

        curr_state = curr+goal

        if curr_state in state:
            continue
        else:
            state[curr_state] = True
        
        record.append([prev, curr])

        # if the goal is found, return the path
        if(curr == goal):
            path = backTraceArray(record, start, goal)
            return path

        neighbors = maze.getNeighbors(curr[0], curr[1])

        for node in neighbors:
            if(maze.isValidMove(node[0], node[1])):
                hq.heappush(heap_q, (dist+1, dist+1+manhanttan(node, goal), node, curr))

    return path



def astar_corner(maze):
    """
    Runs A star for part 2 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    # TODO: Write your code here
    return astar_multi(maze)

def astar_multi(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    path = []
    record = []
    state = {}
    mst_info = {}
    heap_q = []
    
    goals = maze.getObjectives()
    prev = maze.getStart()
    start = maze.getStart()

    curr = start
    curr_goals = goals
    hq.heappush(heap_q, (0, 0, curr, prev, curr_goals))

    while heap_q:
        _, dist, curr, prev, prev_goals = hq.heappop(heap_q)

        curr_goals = prev_goals[:]

        if(curr in curr_goals):
            curr_goals.remove(curr)
        
        # print(dist, curr, curr_goals)
        prev_state = prev+tuple(prev_goals)
        curr_state = curr+tuple(curr_goals)

        if curr_state in state:
            continue
        else:
            state[curr_state] = True
        
        record.append((prev_state, curr_state))

        # if all of the goals are found, return the path
        if(not curr_goals):
            path = backTraceArray(record, start, curr)

            return path

        neighbors = maze.getNeighbors(curr[0], curr[1])

        for node in neighbors:
            if(maze.isValidMove(node[0], node[1])):
                if(tuple(curr_goals) not in mst_info):
                    mst_info[tuple(curr_goals)] = mst(curr_goals)
                dist_to_nearest = manhanttan(node, find_nearest(node, curr_goals))
                mst_val = mst_info[tuple(curr_goals)]

                hq.heappush(heap_q, (dist+1+dist_to_nearest+mst_val ,dist+1, node, curr, curr_goals))

    return path

def fast(maze):
    """
    Runs suboptimal search algorithm for part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []
