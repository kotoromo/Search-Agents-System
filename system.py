#-*- coding: utf-8 -*-
"""
This graph solving system was created by Nicky García Fierros
for PROTECO's Inteligencia Artificial 2017-2 course.

Licensing Information:  You are free to use or extend these projects for
educational purposes provided that (1) you do not distribute or publish
solutions, (2) you retain this notice, and (3) you provide clear
attribution to Nicky García Fierros, including a link to
https://github.com/kotoromo.

"""

import util, solutions

class Problem:

    graph_2 = {
        'A':[('B', 2)],
        'B':[('C', 1), ('D', 5)],
        'D':[('E', 1), ('F', 3)],
        'E':[('D', 5)],
        'F':[('D', 5)]
    }

    """
    Grafo con costos
    """
    graph_1 = {
        'A':[('B', 1)],
        'B':[('C', 1), ('D', 1), ('A', 1)],
        'C':[('B', 1)],
        'D':[('E', 1), ('F', 1), ('B', 1)],
        'E':[('D', 1)],
        'F':[('D', 1)]
    }

    graph_3 = {
        'A':[('B', 3), ('C', 1)],
        'B':[('C', 1), ('D', 2), ('A', 3)],
        'C':[('B', 1), ('A', 1), ('F', 7)],
        'D':[('E', 2), ('F', 1), ('B', 2)],
        'E':[('D', 2)],
        'F':[('D', 1), ('C', 7)]
    }

    """
    Problem abstraction.
    Arguments:
        'space' = state space
        'goal' = goal state
        'start' = start state
        'goal_fn' = custom goal function, if not specified
        uses one that checks for equality between the state given
        and the goal state defined within the problem.
        'heur' = heuristic function to use.
    """

    def __init__(self, *args, **kwargs):
        # Dictionary which states the available default graphs.
        spaces = {'g1': self.graph_1, 'g2': self.graph_2, 'g3': self.graph_3}

        if kwargs is not None:
            if kwargs.get('space') is not None:
                if kwargs.get('space') in spaces.keys():
                    self.state_space = spaces.get(kwargs.get('space'))
                else:
                    self.state_space = kwargs.get('space')
            else:
                self.state_space = spaces['g1']

            self.goal_state = kwargs.get('goal')
            self.start_state = kwargs.get('start')

            if(kwargs.get('goal_fn') is None):
                self.goal_function = self.defaultIsGoal
            else:
                self.goal_function = kwargs.get('goal_fn')

            if kwargs.get('heur') is None:
                self.heuristic = self.nullHeuristic
            else:
                self.heuristic = kwargs.get('heur')


        self.nodes_expanded = 0

    """
    Successor function.
    Given a node, returns the list of successors associated with it.
    """
    def getSuccessors(self, state):
        self.nodes_expanded += 1
        #print("Node expanded:" + state) #DEBUG
        #print "self.state_space"
        #print (self.state_space) #DEBUG
        return self.state_space[state]

    """
    Trivial heuristic function
    """
    def nullHeuristic(self, *args, **kwargs):
        pass

    """
    Sets the heuristic function
    """
    def setHeuristic(self, h):
        self.heuristic = h

    """
    Default goal check function.
    Given a node, returns whether the node given is the same as the goal state.
    """
    def defaultIsGoal(self, state):
        return self.goal_state == state

    """
    Custom goal check function.
    Returns true if the given state is a goal.
    """
    def isGoal(self, state):
        return self.goal_function(state)

    """
    Start state getter.
    Returns the start state as defined by the problem.
    """
    def getStartState(self):
        return self.start_state

    """
    System is probably badly designed. This is the soulution I came up with.
    Method which finds the appropiate key for the given state.
    """
    def findKey(self, node_letter):
        keys = self.state_space.keys()
        for tuple in keys:
            if node_letter == tuple[0]:
                return tuple

    def restartCounter(self):
        self.nodes_expanded = 0

    def getNodesExpanded(self):
        a = self.nodes_expanded
        self.restartCounter()
        return a

    def getSolutionCost(self, solution):
        cost = 0
        if solution is None:
            return 0

        for i in range(0, len(solution)-1):
            #('A', 0):[('B', 3), ('C', 1)]
            # state_space = {(NODE, COST): [LIST OF TUPLES]}
            current = solution[i]
            successors = self.getSuccessors(current)

            #finding tuple with value
            for tuple in successors:
                if solution[i+1] == tuple[0]:
                    cost+=tuple[1]

        return cost



class Solver:
    """
    dfs ha sido implementado en tu lugar.
    """
    def dfs(self, problem):
        problem.restartCounter() #NO BORRAR!
        #Escribe tu código aquí

        fringe = util.Stack()
        visited = []
        plan = [problem.getStartState()]

        fringe.push( (problem.getStartState(), plan) )
        while(not fringe.isEmpty()):
            current, planToNode = fringe.pop()
            if problem.isGoal(current):
                #print("Visited: " + str(visited)) #DEBUG
                return planToNode
            if current not in visited:
                visited.append(current)
                for child in problem.getSuccessors(current):
                    fringe.push( (child[0], planToNode + [child[0]] ) )

    """
    Función que implementa Búsqueda por Amplitud (Breadth First Search)
    """
    def bfs(self, problem):
        problem.restartCounter() #NO BORRAR!
        #Escribe tu código aquí

        return solutions.Algorithms().bfs(problem)
        util.raiseNotDefined()

    """
    Función que implementa Búsqueda de Coste Uniforme
    (conocido como Dijkstra o Uniform Cost Search)
    """
    def ucs(self, problem):
        problem.restartCounter() #NO BORRAR!
        #Escribe tu código aquí

        return solutions.Algorithms().ucs(problem)
        util.raiseNotDefined()
    """
    Función que implementa A* (A estrella).
    """
    def astar(self, problem):
        problem.restartCounter()
        #Escribe tu código aquí
        pathCost = 0 + problem.heuristic((problem.getStartState(), [problem.getStartState()], 0))
        curr_node = (problem.getStartState(), [problem.getStartState()], pathCost)
        fringe = util.PriorityQueue()
        fringe.update(curr_node, 0)
        visited = []
        cost = 0
        while not fringe.isEmpty():
            curr_node = fringe.pop()
            if problem.isGoal(curr_node[0]):
                #print("cost: " + str(curr_node[2])) #DEBUG
                return curr_node[1]
                if curr_node[0] not in visited:
                    visited.append(curr_node[0])
                    for child in problem.getSuccessors(curr_node[0]):
                        child = (child[0], curr_node[1] + [child[0]], child[1] + curr_node[2])
                        #print(child) ->
                        #('B', ['A', 'B'], 3)
                        #('C', ['A', 'C'], 1)
                        if child[0] not in visited:
                            fringe.update(child, child[2] + problem.heuristic(child[0]))
                            #print fringe.heap #DEBUG

def main():
    """
    Objetivo: Implementar todos los algoritmos de búsqueda vistos en clase.
    Plantear adecuadamente el grafo de búsqueda anexado y encontrar la solución
    de dicho grafo utilizando cada una de las funciones implementadas
    (bfs, ucs y a*); DFS ha sido implementado por ti.

    Además, presentar los recorridos realizados por sus algoritmos paso a paso
    en hojas aparte para demostrar la correcta implementación de estos.

    Para ejecutar un problema y probar su solución es necesario hacer lo siguiente:

    # Crear una instancia de la clase 'Problem'. Verificar la definición
    del método constructor para los parámetros que puede tomar.

    # Definimos un problema con inicio en el nodo 'F' y meta en 'C' utilizando
    # El espacio de estados G3 (O el grafo 3)

    problem = Problem(goal='C', start = 'F', space = 'g3')

    # Después, debes crear una instancia de la clase 'Solver'.

    solver = Solver()

    # Debes imprimir la salida de los métodos 'bfs', 'ucs' y 'astar'
    # tal y como se muestra en las siguientes lineas

    print("Solución: " + str(solver.dfs(problem)))
    print("Nodos expandidos: " + str(problem.getNodesExpanded()))

    # Para pasar un grafo declarado en cualquier otra parte, puedes hacer uso del
    # parámetro 'use_graph' del método constructor de la clase problem,
    # el cual es un diccionario como los demás grafos.

    # ej.
    problema = Problem(start = 'X', goal = 'Y', use_graph = mi_grafo)
    solver = Solver()
    solucion = solver.dfs(problema)
    print "Resultado: %s"%(str(solucion))
    print "Nodos expandidos: %i"%(problem.getNodesExpanded())
    print "Costo: %i"%(problem.getSolutionCost(solucion))

    #Para implementar A*, necesitas desarrollar una heuristica admisible
    ( f(s) = g(s) + h(s) admisible <->  0 < h(s)<= g(s) )
    # Una vez hayas escrito tu heuristica, definela en el método constructor
    # del problema mediante el parámetro 'heur'
    # ej.
        def mi_heuristica(estado):
            ...

        problema = Problem(start = 'A', goal = 'F', space = 'g3', heur='mi_heuristica')
        solucionador = Solver(problema)
        solucionador.astar()
            ...


    Éxito y que te diviertas :)
    """

    """
    Takes the total path cost and splits it in half
    """
    def dist_heur(node):
        if isinstance(node, tuple()):
            return problem.getSolutionCost(node[1])/2
        else:
            return 0


    problem = Problem(start = 'A', goal='F', space='g3', heur=dist_heur)
    solv = Solver()
    print("--------------DFS----------------------")
    sol = solv.dfs(problem)
    print "Solution: %s" % (str(sol))
    print "Nodes expanded: %s" % (str(problem.getNodesExpanded()))
    print "Cost: %i" % (problem.getSolutionCost(sol))

    print("--------------BFS----------------------")
    sol = solv.bfs(problem)
    print "Solution: %s" % (str(sol))
    print "Nodes expanded: %s" % (str(problem.getNodesExpanded()))
    print "Cost: %i" % (problem.getSolutionCost(sol))


    print("--------------UCS----------------------")
    sol = solv.ucs(problem)
    print "Solution: %s" % (str(sol))
    print "Nodes expanded: %s" % (str(problem.getNodesExpanded()))
    print "Cost: %i" % (problem.getSolutionCost(sol))

    print("--------------A*----------------------")
    sol = solv.astar(problem)
    print "Solution: %s" % (str(sol))
    print "Nodes expanded: %s" % (str(problem.getNodesExpanded()))
    print "Cost: %i" % (problem.getSolutionCost(sol))

    print("--------------A* TSP----------------------")
    space_solution = solutions.SpaceSolution()
    problem_2 = space_solution.getSolutionProblem()
    problem_2.setHeuristic(dist_heur)
    #sol = solv.bfs(problem_2) THIS CAUSES ERROR

    sol = solv.dfs(problem_2)
    print "Solution: %s" % (str(sol))
    print "Nodes expanded: %s" % (str(problem_2.getNodesExpanded()))
    print "Cost: %i" % (problem_2.getSolutionCost(sol))


if __name__ == '__main__':
    main()
