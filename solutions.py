#-*- coding: utf-8 -*-
import system, util

class Algorithms:
    def dfs(self, problem):
        problem.restartCounter()
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

    def dfs2(self, problem):
        problem.restartCounter()
        #Escribe tu código aquí

        pila = util.Stack()
        visitados = []
        plan = [problem.getStartState()]

        pila.push( (problem.getStartState(), plan) )

        while( not pila.isEmpty() ):
            edo_act, plan = pila.pop()

            if not problem.isGoal(edo_act):
                if edo_act not in visitados:
                    visitados.append(edo_act)

                    for hijo in problem.getSuccessors(edo_act):
                        pila.push( (hijo[0], plan + [hijo[0]]) )
            else:
                break
        return plan

    def bfs(self, problem):
        problem.restartCounter()
        #Escribe tu código aquí
        cola = util.Queue()
        visitados = []
        plan = [problem.getStartState()]

        cola.push( (problem.getStartState(), plan) )

        while( not cola.isEmpty() ):
            edo_act, plan = cola.pop()

            if not problem.isGoal(edo_act):
                if edo_act not in visitados:
                    visitados.append(edo_act)

                    for hijo in problem.getSuccessors(edo_act):
                        cola.push( (hijo[0], plan + [hijo[0]]) )
            else:
                break
        #print visitados #DEBUG
        return plan

    def ucs(self, problem):
        problem.restartCounter()
        #Escribe tu código aquí
        pathCost = 0
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
                        fringe.update(child, child[2])
                    #print fringe.heap #DEBUG

        #util.raiseNotDefined()

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

class SpaceSolution:
    #Base goal world model
    """
    The costs are based on how many stations there are inbetween.
    The cost for any route that connects to Hidalgo and Bellas Artes is added 10
    extra for the hassle that it is traversing to those stations.
    """
    world = {
        'El Rosario':[ ('Tacuba', 3), ("Instituto del Petróleo", 5) ],
        "Instituto del Petróleo":[ ('La Raza', 1), ("Deportivo 18 de Marzo", 1), ('El Rosario', 5) ],
        "Deportivo 18 de Marzo":[ ('La Raza', 1), ("Instituto del Petróleo", 1), ('Martin Carrera', 1) ],
        'Martin Carrera':[ ('Deportivo 18 de Marzo', 1), ('Consulado', 2) ],
        "La Raza": [("Deportivo 18 de Marzo", 1), ("Instituto del Petróleo", 1), ('Consulado', 2), ("Guerrero", 1)],
        "Consulado": [("Martin Carrera", 2), ("La Raza", 2), ("Morelos", 1)],
        "Guerrero": [('Garibaldi', 0), ('La Raza', 1), ("Hidalgo", 10)], #Ir a Hidalgo es bastante molesto, por eso el costo es 10
        "Garibaldi": [('Guerrero', 0), ('Morelos', 2), ('Bellas Artes', 10)], #Ir a Bellas Artes es bastante molesto, por eso el costo es 10
        "Morelos": [("Garibaldi", 2), ("Consulado", 1), ("Candelaria", 0), ("San Lázaro", 0)],
        "Tacuba": [('Hidalgo', 16), ("Tacubaya", 4), ("El Rosario", 3)],
        "Hidalgo": [("Tacuba", 16), ("Balderas", 11), ("Guerrero", 10), ("Bellas Artes", 10)],
        "Bellas Artes": [("Hidalgo", 10), ("Garibaldi", 10), ("Pino Suárez", 12)],
        "Tacubaya": [("Tacuba", 4), ("Mixcoac", 2)],
        "Balderas": [("Hidalgo", 11), ("Centro Médico", 2), ("Pino Suárez", 2), ("Tacubaya", 5)],
        "Pino Suárez": [("Bellas Artes", 2), ("Chabacano", 1), ("Candelaria", 1)],
        "Candelaria": [("Pino Suárez", 1), ("Jamaica", 1), ("San Lázaro", 0), ("Morelos", 0)],
        "San Lázaro": [("Candelaria", 0), ("Morelos", 0)],
        "Mixcoac": [("Tacubaya", 2), ("Zapata", 2)],
        "Centro Médico": [("Chabacano", 1), ("Balderas", 2), ("Zapata", 3)],
        "Chabacano": [("Jamaica", 0), ("Pino Suárez", 1), ("Ermita", 5), ("Centro Médico", 1)],
        "Jamaica": [("Chabacano", 0), ("Candelaria", 1), ("Santa Anita", 0)],
        "Zapata": [("Mixcoac", 2), ("Centro Médico", 3), ("Ermita", 2)],
        "Ermita": [("Zapata", 2), ("Chabacano", 5)],
        "Santa Anita" : [("Jamaica", 0)]
    }

    def __init__(self):
        self.problem = system.Problem( start = 'El Rosario',
                                  goal = 'Chabacano',
                                  space = self.world,
                                )
    def getSolutionProblem(self):
        return self.problem
