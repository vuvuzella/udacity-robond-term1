# MouseAction.py
class MouseAction:
    def __init__(self, action):
        self.action = action

    def __str__(self):
        return self.action

    # def __cmp__(self, other):
    # # other - another MouseAction type object
    #     if self.action == other.action:
    #         return 0
    #     elif self.action > other.action:
    #         return 1
    #     return -1
    #     # return cmp(self.action, other.action) # does not work in python 3

    def __eq__(self, other):
        return self.action == other.action

    def __gt__(self, other):
        return self.action > other.action

    def __lt__(self, other):
        return self.action < other.action

    def __hash__(self):
        return hash(self.action)

MouseAction.appears = MouseAction("mouse appears")
MouseAction.runsAway = MouseAction("mouse runs away")
MouseAction.enters = MouseAction("mouse enters trap")
MouseAction.escapes = MouseAction("mouse escapes")
MouseAction.trapped = MouseAction("mouse trapped")
MouseAction.removed = MouseAction("mouse removed")
