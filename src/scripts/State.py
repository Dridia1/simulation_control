#!/usr/bin/env python

# Will help to identify something as a State and acts as an interface


class State:
    def do_action(self):
        assert 0, "do method not implemented"

    def next_state(self):
        assert 0, "next_state method not implemented"
