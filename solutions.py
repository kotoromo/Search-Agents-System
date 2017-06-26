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
                            #util.raiseNotDefined()

class SpaceSolution:
    #Base goal world model
    world = {
        "Instituto del Petróleo":[ ('La Raza', 0), ("Deportivo 18 de Marzo", 0), ('El Rosario', 0) ],
        'El Rosario':[ ('Tacuba', 0), ("Instituto del Petróleo", 0) ],
        "Deportivo 18 de Marzo":[ ('La Raza', 0), ("Instituto del Petróleo", 0), ('Martin Carrera', 0) ],
        'Martin Carrera':[ ('Deportivo 18 de Marzo', 0), ('Consulado', 0) ],
        "La Raza": [("Deportivo 18 de Marzo", 0), ("Instituto del Petróleo", 0), ('Consulado', 0), ("Guerrero", 0)],
        "Consulado": [("Martin Carrera", 0), ("La Raza", 0), ("Morelos", 0)],
        "Guerrero": [('Garibaldi', 0), ('La Raza', 0), ("Hidalgo", 0)],
        "Garibaldi": [('Guerrero', 0), ('Morelos', 0), ('Bellas Artes', 0)],
        "Morelos": [("Garibaldi", 0), ("Consulado", 0), ("Candelaria", 0), ("San Lázaro", 0)],
        "Tacuba": [('Hidalgo', 0), ("Tacubaya", 0), ("El Rosario", 0)],
        "Hidalgo": [("Tacuba", 0), ("Balderas", 0), ("Guerrero", 0), ("Bellas Artes", 0)],
        "Bellas Artes": [("Hidalgo", 0), ("Garibaldi", 0), ("Pino Suárez", 0)],
        "Tacubaya": [("Tacuba", 0), ("Mixcoac", 0)],
        "Balderas": [("Hidalgo", 0), ("Centro Médico", 0), ("Pino Suárez", 0), ("Tacubaya", 0)],
        "Pino Suárez": [("Bellas Artes", 0), ("Chabacano", 0), ("Candelaria", 0)],
        "Candelaria": [("Pino Suárez", 0), ("Jamaica", 0), ("San Lázaro", 0), ("Morelos", 0)],
        "San Lázaro": [("Candelaria", 0), ("Morelos", 0)],
        "Mixcoac": [("Tacubaya", 0), ("Zapata", 0)],
        "Centro Médico": [("Chabacano", 0), ("Balderas", 0), ("Zapata", 0)],
        "Chabacano": [("Jamaica", 0), ("Pino Suárez", 0), ("Ermita", 0), ("Centro Médico", 0)],
        "Jamaica": [("Chabacano", 0), ("Candelaria", 0), ("Santa Anita", 0)],
        "Zapata": [("Mixcoac", 0), ("Centro Médico", 0), ("Ermita", 0)],
        "Ermita": [("Zapata", 0), ("Chabacano", 0)]
    }

    def __init__(self):
        self.problem = system.Problem( start = 'El Rosario',
                                  goal = 'Hidalgo',
                                  space = self.world,
                                )
    def getSolutionProblem(self):
        return self.problem
