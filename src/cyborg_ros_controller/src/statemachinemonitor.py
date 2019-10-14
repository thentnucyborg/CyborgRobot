#!/usr/bin/env python
"""Created by Thomas Rostrup Andersen on 11/11/2016.
Copyright (C) 2016 Thomas Rostrup Andersen. All rights reserved."""

import os
import smach
import pygraphviz

__author__ = "Thomas Rostrup Andersen"
__copyright__ = "Copyright (C) 2016 Thomas Rostrup Andersen"
#__license__ = ""
__version__ = "0.0.2"
__all__ = ['StateMachineMonitor']

class StateMachineMonitor():
    """StateMachineMonitor takes in a SMACH state machine and crates a graph of it and keeps updating the graph.

    StateMachineMonitor creates a graph of a SMACH state machine. When a state change occurs, the graph will be updated. Active states will be displayed in green color. States that contains a hiarchy of states will be displayed in gray color.

        If display_all is Ture, the graph will display all state hiarchies, othervise it will only display full state hiarchies of active states. This is usefull when dealing with large graphs to create a more compact view.

        The created graph is an image in png format, and the file is located in the users home folder.
    """

    graph = pygraphviz.AGraph(strict=False, directed=True)

    def __init__(self, state_machine, display_all=False):
        self.state_machine = state_machine
        self.display_all = display_all
        state_machine.register_transition_cb(transition_cb=self.transition_cb)
        self.update()

    # This function is called every time there is a transition in the state machine
    # Calls the update() function
    def transition_cb(self, *cb_args):
        self.update()

    # This function updates the graph and creates an image
    def update(self):
        homedir = os.path.expanduser("~")
        path = homedir + "/controller.db"

        self.graph.clear()
        self.traverse_state_machine(self.state_machine)
        #print(self.graph.string()) # print to screen
        self.graph.write(path + 'graph.dot') # write to graph.dot
        image=pygraphviz.AGraph(path + 'graph.dot') # create a new graph from file
        image.layout(prog='dot')
        image.draw(path + 'graph.png') # draw png

    # This function goes trough the state machine and adds it to the graph
    def traverse_state_machine(self, state_machine):
        sub_state_machines = []
        for state_name in state_machine.get_children():
            state = state_machine.__getitem__(key = str(state_name)) # Get the state class or state machine class
            if (type(state) is smach.StateMachine): # If it is a state machine class (with sub states), it must be handled after regular states, due to the lib used (edges add nodes and removal).
                sub_state_machines.append(state_name)
            else:
                if (state_name in state_machine.get_active_states()):
                    self.graph.add_node(state_name, style='filled', color='green')
                else:
                    self.graph.add_node(state_name)
                transitions = [t for t in state_machine.get_internal_edges() if t[1] is state_name]
                for t in transitions:
                    self.graph.add_edge(state_name, t[2], label=t[0])

            for state_name in sub_state_machines:
                state = state_machine.__getitem__(key = str(state_name)) # Get the state class or state machine class
                if (state_name in state_machine.get_active_states() or self.display_all): # If it is an ective state, add the sub state to the graph
                    self.traverse_state_machine(state)
                    self.graph.add_subgraph(nbunch=state.get_children(), name='cluster', rank='sink', style='filled', color='lightgray') # HERE - Need cluster?

                    # Get the transition about entering this sub state machine, and connect it to the initial state(s)
                    transitions = [t for t in state_machine.get_internal_edges() if t[2] is state_name]
                    for t in transitions:
                        for initzial_state in state.get_initial_states():
                            self.graph.add_edge(t[1], initzial_state, label=t[0])

                    # Get the transition about leaving this sub state machine,
                    sub_transitions = [t for t in state_machine.get_internal_edges() if t[1] is state_name]
                    for sub_t in sub_transitions:
                        # Get the last states before leaving the substate (the state that leads to sub_t0)
                        for state_transition in state.get_internal_edges():
                            if state_transition[2] is sub_t[0]:
                                self.graph.add_edge(state_transition[1], sub_t[2], label=sub_t[0])
                                self.graph.remove_node(sub_t[0])
                    self.graph.remove_node(state_name)
                else:
                    self.graph.add_node(state_name, style='filled', color='lightgray')
