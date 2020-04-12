#!/usr/bin/env python

import itertools
import pydot
import rospy

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return itertools.izip(a, b)

class RosBTDotcodeGenerator(object):
    def __init__(self, data_provider):
        self.data_provider = data_provider

    def generate_dotcode(self, max_depth=-1):
        G, active_nodes = self.data_provider.get_graph()

        # Remove nodes that are too deep
        for node in G.nodes():
            node.attr['shape'] = 'box'

            # Remove nodes deeper than the given draw depth
            if max_depth != -1 and int(node.attr['depth']) > max_depth:
                G.remove_node(node)

        G.layout(prog='dot')

        # Highlight active nodes
        for node in active_nodes:
            if node in G.nodes():
                n = G.get_node(node)
                n.attr['color'] = 'green'

        for (start, end) in pairwise(active_nodes):
            if start in G.nodes() and end in G.nodes():
                try:
                    e = G.get_edge(start, end)
                    e.attr['style'] = 'dashed'
                    e.attr['penwidth'] = 2

                    e.attr['colorR'] = 255
                    e.attr['colorG'] = 0
                    e.attr['colorB'] = 0
                except KeyError:
                    pass

        dot = G.string()

        return dot
