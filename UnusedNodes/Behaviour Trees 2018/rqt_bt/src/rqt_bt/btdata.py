#!/usr/bin/env python

import json
import networkx as nx
import pygraphviz
import rospy

from cyborg_msgs.msg import BehaviorTree, BehaviorTreeNodes
from networkx.utils.misc import make_str

_attrs = dict(id='id', children='children')
def tree_graph(data, attrs=_attrs):
    """Return graph from tree data format.

    Parameters
    ----------
    data : dict
        Tree formatted graph data

    Returns
    -------
    G : NetworkX OrderedDiGraph

    attrs : dict
        A dictionary that contains two keys 'id' and 'children'. The
        corresponding values provide the attribute names for storing
        NetworkX-internal graph data. The values should be unique. Default
        value: :samp:`dict(id='id', children='children')`.

    Examples
    --------
    >>> from networkx.readwrite import json_graph
    >>> G = nx.OrderedDiGraph([(1,2)])
    >>> data = json_graph.tree_data(G,root=1)
    >>> H = json_graph.tree_graph(data)

    Notes
    -----
    The default value of attrs will be changed in a future release of NetworkX.

    See Also
    --------
    tree_graph, node_link_data, adjacency_data
    """
    graph = nx.OrderedDiGraph()
    id_ = attrs['id']
    children = attrs['children']

    def add_children(parent, children_):
        for data in children_:
            child = data[id_]
            graph.add_edge(parent, child)
            grandchildren = data.get(children, [])
            if grandchildren:
                add_children(child, grandchildren)
            nodedata = dict((make_str(k), v) for k, v in data.items()
                            if k != id_ and k != children)
            graph.add_node(child, **nodedata)

    root = data[id_]
    children_ = data.get(children, [])
    nodedata = dict((make_str(k), v) for k, v in data.items()
                    if k != id_ and k != children)
    graph.add_node(root, **nodedata)
    add_children(root, children_)
    return graph

class BTData(object):
    def __init__(self, data_sub_name=None, update_sub_name=None):
        if data_sub_name:
            rospy.Subscriber(data_sub_name, BehaviorTree, self._bt_cb)

        if update_sub_name:
            rospy.Subscriber(update_sub_name, BehaviorTreeNodes, self._bt_update_cb)

        self.tree = None
        self.active_nodes = []

    def get_graph(self):
        try:
            graph = tree_graph(self.tree)
        except:
            graph = nx.OrderedDiGraph()

        G = nx.drawing.nx_agraph.to_agraph(graph)

        return G, self.active_nodes

    def _bt_cb(self, msg):
        self.tree = json.loads(msg.tree)

    def _bt_update_cb(self, msg):
        self.active_nodes = msg.ids
