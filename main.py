from search import *

# STOCK FUNCTION
def best_first_graph_search(problem, f):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = memoize(f, 'f')
    node = Node(problem.initial)
    if problem.goal_test(node.state):
        return node
    frontier = PriorityQueue(min, f)
    frontier.append(node)
    explored = []
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node
        explored.append(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
    return None

# STOCK FUNCTION
def astar_search(problem, h=None):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n))

class Problem(Problem):
    def __init__(self):
        # world defined by 1-D array
        initial = [[1, 1, 1, 1, 1,
                          0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0], 20]

        # goal state
        goal = [[[0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0], i] for i in range(0, 24)]
        super().__init__(initial, goal)

    def actions(self, state):
        # Returns list of possible actions for agent at location x
        # state[x], (0 = world status) (1 = agent location)

        loc = state[1]
        if state[0][loc] == 1:
            return ["SUCK"]
        elif loc == 0:
            return ["DOWN", "RIGHT"]
        elif loc == 1:
            return ["LEFT", "DOWN", "RIGHT"]
        elif loc == 2:
            return ["LEFT", "DOWN", "RIGHT"]
        elif loc == 3:
            return ["LEFT", "DOWN", "RIGHT"]
        elif loc == 4:
            return ["LEFT", "DOWN"]
        elif loc == 5:
            return ["UP", "RIGHT", "DOWN"]
        elif loc == 6:
            return ["LEFT", "UP", "RIGHT", "DOWN"]
        elif loc == 7:
            return ["LEFT", "UP", "RIGHT", "DOWN"]
        elif loc == 8:
            return ["LEFT", "UP", "RIGHT", "DOWN"]
        elif loc == 9:
            return ["LEFT", "UP", "DOWN"]
        elif loc == 10:
            return ["UP", "RIGHT", "DOWN"]
        elif loc == 11:
            return ["LEFT", "UP", "RIGHT", "DOWN"]
        elif loc == 12:
            return ["LEFT", "UP", "RIGHT", "DOWN"]
        elif loc == 13:
            return ["LEFT", "UP", "RIGHT", "DOWN"]
        elif loc == 14:
            return ["LEFT", "UP", "DOWN"]
        elif loc == 15:
            return ["UP", "RIGHT", "DOWN"]
        elif loc == 16:
            return ["LEFT", "UP", "RIGHT", "DOWN"]
        elif loc == 17:
            return ["LEFT", "UP", "RIGHT", "DOWN"]
        elif loc == 18:
            return ["LEFT", "UP", "RIGHT", "DOWN"]
        elif loc == 19:
            return ["LEFT", "UP", "DOWN"]
        elif loc == 20:
            return ["UP", "RIGHT"]
        elif loc == 21:
            return ["LEFT", "UP", "RIGHT"]
        elif loc == 22:
            return ["LEFT", "UP", "RIGHT"]
        elif loc == 23:
            return ["LEFT", "UP", "RIGHT"]
        elif loc == 24:
            return ["LEFT", "UP"]
        else:
            return None

    def result(self, state, action):
        world = list(state[0])

        loc = state[1]
        agent_state = []
        agent_state.append(world)
        agent_state.append(loc)

        if action == "SUCK":
            agent_state[0][agent_state[1]] = 0
            return agent_state

        elif action == "UP":
            agent_state[1] -= 5
            return agent_state

        elif action == "DOWN":
            agent_state[1] += 5
            return agent_state

        elif action == "LEFT":
            agent_state[1] -= 1
            return agent_state

        elif action == "RIGHT":
            agent_state[1] += 1
            return agent_state

    def path_cost(self, cost, state1, action, state2):
        if action == "SUCK":
            iteration_cost = abs(-2 * state2[0].count(1) - 1)
            final_cost = cost + iteration_cost # Calculate final cost using old cost plus new iterative cost
            return final_cost
        else:
            iteration_cost = abs(-2 * state1[0].count(1) - 1)
            final_cost = cost + iteration_cost
            return final_cost

    def goal_test(self, state):
        for g in self.goal:
            if g == state:
                return True

    def h(self, n):
        h_cost = 0
        min_dist = 10000000
        num_actions = 0
        dirty = 0
        pos = n.state[1]
        count = 0
        for s in n.state[0]:
            if s == 1:
                dirty += 1
                dist = abs(pos - count)
                if min_dist > dist:
                    min_dist = dist
            count += 1
        num_actions = min_dist + 1
        h_cost = dirty + min_dist + num_actions
        return h_cost

h1 = astar_search(Problem())
for path in h1.path():
    print('(',  ((path.state[1]) % 5).__ceil__(), ', ', (abs((path.state[1] / 5).__floor__() - 4)), sep='', end=') | ')
    print(f"{'Action:'} {'{}'.format(path.action):<5} {'|'} {'Path Cost:' } {'{}'.format(path.path_cost)}")
