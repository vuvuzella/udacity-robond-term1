# MouseTrapTest.py

import sys
from State import State
from StateMachine import StateMachine
from mouse.MouseAction import MouseAction

# States:
#   waiting
#   luring
#   trapping
#   holding

class Waiting(State):
    def run(self):
        print("Waiting: Broadcasting Cheese Smell")

    def next(self, inputs):
        if inputs == MouseAction.appears:
            return MouseTrap.luring
        return MouseTrap.waiting

class Luring(State):
    def run(self):
        print("Luring, Presenting Cheese, door open")

    def next(self, inputs):
        if inputs == MouseAction.runsAway:
            return MouseTrap.waiting
        elif inputs == MouseAction.enters:
            return MouseTrap.trapping
        else:
            return MouseTrap.luring

class Trapping(State):
    def run(self):
        print("Trapping: Close door")

    def next(self, inputs):
        if inputs == MouseAction.escapes:
            return MouseTrap.waiting
        elif inputs == MouseAction.trapped:
            return MouseTrap.holding
        else:
            return MouseTrap.trapping

class Holding(State):
    def run(self):
        print("Holding: mouse caught")

    def next(self, inputs):
        if inputs == MouseAction.removed:
            return MouseTrap.waiting
        else:
            return MouseTrap.holding

class MouseTrap(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, MouseTrap.waiting)

MouseTrap.waiting = Waiting()
MouseTrap.luring = Luring()
MouseTrap.trapping = Trapping()
MouseTrap.holding = Holding()

f = open("./mouse/MouseMoves.txt", "r")
moves = []
for line in f:
    moves.append(line.strip())
print(moves)
MouseTrap().runAll(map(MouseAction, moves))
