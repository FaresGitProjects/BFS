
import time
import queue
from slideproblem import *
from collections import deque
import sys

## you likely need to inport some more modules to do the serach


class Searches:
    def graph_bfs(self, problem):
        # reset the node counter for profiling
        # the serach should return the result of 'solution(node)'
        "*** YOUR CODE HERE ***"
        print(problem.initialState)
        Node.nodeCount = 0
        root = Node(state=problem.initialState)
        fringe = deque([root])
        if problem.goalTest(root):
            return solution(root)
        explored_states = []
        while len(fringe) != 0:
            node = fringe.popleft()
            explored_states.append(node.state)
            for action in problem.applicable(node.state):
                new_child = child_node(node, action, problem)
                if new_child.state not in explored_states and new_child not in fringe:
                    if problem.goalTest(new_child.state):
                        return solution(new_child)
                    fringe.append(new_child)
        return "None"

    def recursiveDL_DFS(self, lim, problem):
        n = Node(None, None, 0, problem.initialState)
        return self.depthLimitedDFS(n, lim, problem)

    def depthLimitedDFS(self, n, lim, problem):
        # reset the node counter for profiling
        # the serach should return the result of 'solution(node)'
        "*** YOUR CODE HERE ***"
        Node.nodeCount = 0
        if problem.goalTest(n.state):
            return solution(n)
        elif lim == 0:
            return 'cutoff'
        else:
            cutoff = False
            for action in problem.applicable(n.state):
                new_child = child_node(n, action, problem)
                result = self.depthLimitedDFS(new_child, lim-1, problem)
                if result == 'cutoff':
                    cutoff = True
                elif result is not 'None':
                    return result
            return 'cutoff' if cutoff else 'None'

    def id_dfs(self, problem):
        # reset the node counter for profiling
        # the serach should return the result of 'solution(node)'
        "*** YOUR CODE HERE ***"
        Node.nodeCount = 0
        for depth in range(sys.maxsize):
            result = self.recursiveDL_DFS(depth, problem)
            if result != 'cutoff':
                return result

    # START: DEFINED ALREADY
    def poseList(self, s):
        poses = list(range(s.boardSize * s.boardSize))

        for tile in range(s.boardSize * s.boardSize):
            for row in range(s.boardSize):
                for col in range(s.boardSize):
                    poses[s.board[row][col]] = [row, col]
        return poses

    def heuristic(self, s0, sf):
        pl0 = self.poseList(s0)
        plf = self.poseList(sf)

        h = 0
        for i in range(1, s0.boardSize * s0.boardSize):
            h += abs(pl0[i][0] - plf[i][0]) + abs(plf[i][1] - plf[i][1])
        return h

    # END: DEFINED ALREADY

    def a_star_tree(self, problem: Problem) -> str:
        # reset the node counter for profiling
        # the serach should return the result of 'solution(node)'
        "*** YOUR CODE HERE ***"

        def get_cheapest(fringe):
            min_n = fringe[0]
            for node in fringe:
                if node.f < min_n.f:
                    min_n = node
            return min_n

        f = lambda node: node.cost + self.heuristic(node.state, problem.goalState)

        Node.nodeCount = 0
        root = Node(state=problem.initialState)
        fringe = [root]
        cheapest_node = root
        while len(fringe) != 0:
            fringe.remove(cheapest_node)
            if problem.goalTest(cheapest_node.state):
                return solution(cheapest_node)
            for action in problem.applicable(cheapest_node.state):
                new_child = child_node(cheapest_node, action, problem)
                new_child.f = f(new_child)
                fringe.append(new_child)
            cheapest_node = get_cheapest(fringe)
        return "None"

    def a_star_graph(self, problem: Problem) -> tuple:
        # reset the node counter for profiling
        # the serach should return the result of 'solution(node)'
        "*** YOUR CODE HERE ***"
        Node.nodeCount = 0

        def get_cheapest(fringe):
            min_n = fringe[0]
            for node in fringe:
                if node.f < min_n.f:
                    min_n = node
            return min_n

        f = lambda node: node.cost + self.heuristic(node.state, problem.goalState)

        root = Node(state=problem.initialState)
        fringe = [root]
        cheapest_node = root
        explored_states = [root.state]

        while len(fringe) != 0:
            fringe.remove(cheapest_node)
            if problem.goalTest(cheapest_node.state):
                return solution(cheapest_node)
            for action in problem.applicable(cheapest_node.state):
                new_child = child_node(cheapest_node, action, problem)
                new_child.f = f(new_child)
                if new_child.state not in explored_states:
                    fringe.append(new_child)
                    explored_states.append(new_child.state)
            cheapest_node = get_cheapest(fringe)
        return "None", 0

    # EXTRA CREDIT (OPTIONAL)
    def solve4x4(self, p: Problem) -> tuple:
        # reset the node counter for profiling
        # the serach should return the result of 'solution(node)'
        "*** YOUR CODE HERE ***"
        return "Fake return value"


if __name__ == '__main__':
    p = Problem()
    s = State()
    n = Node(None, None, 0, s)
    n2 = Node(n, None, 0, s)

    searches = Searches()

    p.goalState = State(s)

    p.apply('R', s)
    p.apply('R', s)
    p.apply('D', s)
    p.apply('D', s)
    p.apply('L', s)

    p.initialState = State(s)

    print(p.initialState)

    si = State(s)
    # change the number of random moves appropriately
    # If you are curious see if you get a solution >30 moves. The 
    apply_rnd_moves(15, si, p)
    p.initialState = si

    startTime = time.perf_counter()

    print('\n\n=== BFS ===\n')
    startTime = time.perf_counter()
    res = searches.graph_bfs(p)
    print(time.perf_counter() - startTime)
    print(Node.nodeCount)
    print(res)

    print('\n\n=== Iterative Deepening DFS ===\n')
    startTime = time.perf_counter()
    res = searches.id_dfs(p)
    print(time.perf_counter() - startTime)
    print(Node.nodeCount)
    print(res)

    print('\n\n=== A*-Tree ===\n')
    startTime = time.perf_counter()
    res = searches.a_star_tree(p)
    print(time.perf_counter() - startTime)
    print(Node.nodeCount)
    print(res)

    print('\n\n=== A*-Graph ===\n')
    startTime = time.perf_counter()
    res = searches.a_star_graph(p)
    print(time.perf_counter() - startTime)
    print(Node.nodeCount)
    print(res)

    # EXTRA CREDIT (OPTIONAL)
    # UN-COMMENT the code below when you test this
    # change the 'boardSize' variable into 4 from slideproblem.py file
    """
    print('\n\n=== A*-solve4x4 ===\n')
    startTime = time.clock()
    res = searches.solve4x4(p)
    print(time.clock() - startTime)
    print(Node.nodeCount)
    print(res)
    """
