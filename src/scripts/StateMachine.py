#!/usr/bin/env python

# States class will control all the states and call the do_action


class StateMachine:

    def __init__(self, state):
        self.currentState = state

    def start(self):
        # return self.currentState.next_state()
        self.currentState.do_action()

    def do_next_state(self):
        # print("Before trans, state is" + str(self.currentState.__class__.__name__))
        self.currentState = self.currentState.next_state()
        self.currentState.do_action()
