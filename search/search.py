import time  # Importing time module to measure execution time
import util

# ===================================================
# CLASS: SearchProblem
# ===================================================
class SearchProblem:
    """
    This class outlines the structure of a search problem but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
    """

    def getStartState(self):
        """Returns the start state for the search problem."""
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
        Returns a list of triples, (successor, action, stepCost), where:
        - 'successor' is the next state
        - 'action' is the action required to get there
        - 'stepCost' is the incremental cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
        Returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

# ===================================================
# FUNCTION: Path Logger
# ===================================================
def log_path(path):
    """
    Logs the path in a step-by-step manner for better readability.
    """
    print("\nPath Taken (Step-by-Step):")
    if len(path) == 0:
        print("No solution found.")
    else:
        for step_num, action in enumerate(path, start=1):
            print(f"Step {step_num}: {action}")
    print(f"\nTotal Steps: {len(path)}")

# ===================================================
# FUNCTION: Depth-First Search (DFS)
# ===================================================
def depthFirstSearch(problem):
    """
    Implements Depth-First Search (DFS) algorithm.
    """
    start_time = time.time()  # Start timing the algorithm
    startState = problem.getStartState()
    
    if problem.isGoalState(startState):
        print("\nDepth-First Search:")
        log_path([])  # Goal is the start state itself (no actions)
        print(f"Execution Time: {time.time() - start_time:.4f} seconds")
        return []

    frontier = util.Stack()  # Using a stack for DFS
    frontier.push((startState, []))
    explored = set()

    while not frontier.isEmpty():
        state, actions = frontier.pop()

        if problem.isGoalState(state):
            elapsed_time = time.time() - start_time
            print("\nDepth-First Search:")
            log_path(actions)
            print(f"Execution Time: {elapsed_time:.4f} seconds")
            return actions

        if state not in explored:
            explored.add(state)
            for successor, action, stepCost in problem.getSuccessors(state):
                if successor not in explored:
                    newActions = actions + [action]
                    frontier.push((successor, newActions))

    print("\nDepth-First Search:")
    log_path([])  # No solution
    print(f"Execution Time: {time.time() - start_time:.4f} seconds")
    return []

# ===================================================
# FUNCTION: Breadth-First Search (BFS)
# ===================================================
def breadthFirstSearch(problem):
    """
    Implements Breadth-First Search (BFS) algorithm.
    """
    start_time = time.time()  # Start timing the algorithm
    currState = problem.getStartState()
    currPath = []

    if problem.isGoalState(currState):
        print("\nBreadth-First Search:")
        log_path(currPath)
        print(f"Execution Time: {time.time() - start_time:.4f} seconds")
        return currPath

    frontier = util.Queue()  # Using a queue for BFS
    frontier.push((currState, currPath))
    explored = set()

    while not frontier.isEmpty():
        currState, currPath = frontier.pop()
        if problem.isGoalState(currState):
            elapsed_time = time.time() - start_time
            print("\nBreadth-First Search:")
            log_path(currPath)
            print(f"Execution Time: {elapsed_time:.4f} seconds")
            return currPath

        explored.add(currState)
        frontierStates = [t[0] for t in frontier.list]  # Avoid revisiting states
        for s in problem.getSuccessors(currState):
            if s[0] not in explored and s[0] not in frontierStates:
                frontier.push((s[0], currPath + [s[1]]))

    print("\nBreadth-First Search:")
    log_path([])  # No solution
    print(f"Execution Time: {time.time() - start_time:.4f} seconds")
    return []

# ===================================================
# FUNCTION: Uniform-Cost Search (UCS)
# ===================================================
def uniformCostSearch(problem):
    """
    Implements Uniform-Cost Search (UCS) algorithm.
    """
    start_time = time.time()  # Start timing the algorithm
    from util import PriorityQueue
    frontier = PriorityQueue()
    frontier.push(problem.getStartState(), 0)
    visited = set()
    pathToState = {}

    # Push the initial state into pathToState with an empty path
    pathToState[problem.getStartState()] = []

    while not frontier.isEmpty():
        state = frontier.pop()
        path = pathToState[state]  # Get the path associated with the state

        if problem.isGoalState(state):
            elapsed_time = time.time() - start_time
            print("\nUniform-Cost Search:")
            log_path(path)
            print(f"Execution Time: {elapsed_time:.4f} seconds")
            return path

        if state not in visited:
            visited.add(state)
            for successor, action, stepCost in problem.getSuccessors(state):
                newCost = problem.getCostOfActions(path + [action])
                frontier.push(successor, newCost)
                pathToState[successor] = path + [action]

    print("\nUniform-Cost Search:")
    log_path([])  # No solution
    print(f"Execution Time: {time.time() - start_time:.4f} seconds")
    return []

# ===================================================
# FUNCTION: A* Search
# ===================================================
def aStarSearch(problem, heuristic=lambda state, problem: 0):
    """
    Implements A* Search algorithm.
    """
    start_time = time.time()  # Start timing the entire search process
    from util import PriorityQueue
    frontier = PriorityQueue()
    startState = problem.getStartState()
    frontier.push(startState, 0)  # Push the start state with 0 cost

    visited = set()
    pathToState = {}

    # Push the initial state into pathToState with an empty path
    pathToState[startState] = []

    while not frontier.isEmpty():
        state = frontier.pop()
        path = pathToState[state]  # Get the path associated with the state

        if problem.isGoalState(state):
            elapsed_time = time.time() - start_time
            print("\nA* Search:")
            log_path(path)  # Log the path found
            print(f"Execution Time (Total Time Taken): {elapsed_time:.4f} seconds")
            return path

        if state not in visited:
            visited.add(state)  # Mark the state as visited
            for successor, action, stepCost in problem.getSuccessors(state):
                newCost = problem.getCostOfActions(path + [action]) + heuristic(successor, problem)
                frontier.push(successor, newCost)  # Push to priority queue with f = g + h
                pathToState[successor] = path + [action]  # Update the path

    # No solution found
    elapsed_time = time.time() - start_time
    print("\nA* Search:")
    log_path([])  # No solution found
    print(f"Execution Time (Total Time Taken): {elapsed_time:.4f} seconds")
    return []

# ===================================================
# ABBREVIATIONS
# ===================================================
bfs = breadthFirstSearch
dfs = depthFirstSearch
ucs = uniformCostSearch
astar = aStarSearch
