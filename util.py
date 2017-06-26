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

"""
En este archivo están definidas distintas estructuras de datos
"""

import heapq

class Stack:
    aux = []

    """
    Sigue la política de inserción LIFO
    """
    def __init__(self):
        pass

    def push(self, obj):
        self.aux.insert(0, obj)

    def pop(self):
        return self.aux.pop(0)

    def peek(self):
        return self.aux[0]

    def isEmpty(self):
        return (self.aux == [])

    def toString(self):
        return str(self.aux)

class Queue:
    aux = []

    """
    Sigue la política de inserción FIFO
    """
    def __init__(self):
        pass

    def push(self, obj):
        self.aux.append(obj)

    def pop(self):
        return self.aux.pop(0)

    def peek(self):
        return self.aux[0]

    def isEmpty(self):
        return self.aux == []

    def toString(self):
        return str(self.aux)

class PriorityQueue:
    def __init__(self):
        self.heap = []
        heapq.heapify(self.heap)

    def push(self, obj, priority):
        heapq.heappush(self.heap, (priority, obj))
        return 0

    def pop(self):
        p, item = heapq.heappop(self.heap)
        return item

    def update(self, obj, priority):
        for index, (pty, item) in enumerate(self.heap):
            if item == obj:
                # Si el objeto item ya se encuentra en la cola de prioridad con prioridad igual o menor, no hace nada.
                if pty <= priority:
                    break
                # Si el objeto item ya se encuentra en la cola de prioridad con prioridad más alta, actualiza su prioridad y reconstruye el heap
                del self.heap[index]
                self.heap.append((priority, obj))
                heapq.heapify(self.heap)
                break
        else:
            # Si el objeto item no se encuentra dentro de la cola de prioridad, hace lo mismo que self.push
            self.push(obj, priority)


    def isEmpty(self):
        return len(self.heap) == 0

def raiseNotDefined():
    print("Method has not been implemented")
"""
def main():
    stack = Stack()
    queue = Queue()

    stack.push('hola')
    stack.push('adiós')

    print(stack.toString())
    stack.pop()
    print(stack.toString())

    queue.push('hola')
    queue.push('adiós')
    print(queue.toString())
    queue.pop()
    print(queue.toString())

if __name__ == '__main__':
    main()
"""
