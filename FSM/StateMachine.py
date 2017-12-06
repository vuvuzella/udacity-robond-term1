# StateMachine.py
class StateMachine:
    def __init__(self, initialState):
        self.currentState = initialState
        self.currentState.run()
    # template method
    def runAll(self, inputs):
        for i in inputs:
            print(i)
            self.currentState = self.currentState.next(i)
            self.currentState.run()
