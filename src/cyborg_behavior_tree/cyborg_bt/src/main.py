#!/usr/bin/env python

import os
import b3
import json
import rospy

from collections import OrderedDict
from cyborg_msgs.msg import BehaviorTree, BehaviorTreeNodes
from cyborg_bt_nodes.actions import MoveTo
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
import networkx as nx

homedir = os.path.expanduser("~")

NAME = 'behavior_tree_manager'


class BehaviorTreeManager():
    def __init__(self, start_running=False):
        rospy.init_node(NAME)

        rospy.loginfo('Initializing: %s' % NAME)

        bt_pub_name = '/cyborg/bt/behavior_tree'
        self.bt_pub = rospy.Publisher(bt_pub_name, BehaviorTree, latch=True, queue_size=1)

        bt_update_pub_name = '/cyborg/bt/behavior_tree_updates'
        self.bt_update_pub = rospy.Publisher(bt_update_pub_name, BehaviorTreeNodes, latch=True, queue_size=1)

        bt_enabled_pub_name = '/cyborg/bt/enabled'
        self.bt_enabled_pub = rospy.Publisher(bt_enabled_pub_name, Bool, latch=True, queue_size=1)

        self.enabled = True

        bt_enable_srv_name = '/cyborg/bt/enable'
        rospy.Service(bt_enable_srv_name, SetBool, self._enable_cb)

        names = {'MoveTo': MoveTo}

        with open(homedir + '/.ros/project.json') as f:
            rospy.loginfo('Loading project from %s' % f.name)
            data = json.load(f, object_pairs_hook=OrderedDict)

        self.root = self._load_project(data, names)
        if not self.root:
            raise AttributeError

        # Publish Behavior Tree structure
        self._publish_bt(self.root)

        self.target = None
        self.blackboard = b3.Blackboard()
        self._prev_open_nodes = list()

        # Set known locations
        locations = {
                'el5':         [-33.768, -33.545, 0],
                'waitingarea': [-33.581,  10.627, 0],
                'info':        [-33.505,  1.139, 0],
                'cafeteria':   [-33.075, -55.776, 0],
                'stairs':      [-31.494, -71.056, 0],
                'el6':         [-30.397, -12.826, 0],
                'hallway':     [-30.251, -79.089, 0],
                'home':        [-29.552,  8.747, 0],
                'elevator1':   [-29.414, -50.281, 0],
                'underbridge': [-28.161, -63.308, 0],
                'elevator2':   [-21.912, -42.451, 0],
                'entrance':    [-18.422,  6.518, 0],
                'entrance2':   [-18.246, -65.898, 0]
        }

        self.blackboard.set('locations', locations)

        rospy.loginfo('Running %s' % self.root)

        self.enabled = start_running
        if start_running:
            self._publish_bt_status()
            self.run()

    def __str__(self):
        if not hasattr(self, '_dot'):
            rospy.loginfo("Creating dot repr")
            self._dot = self._to_pydot(self.root).to_string()
        return self._dot

    def _enable_cb(self, msg):
        self.enabled = msg.data
        self._publish_bt_status()

        message = "enabled" if self.enabled else "disabled"
        rospy.loginfo("Behavior tree {}".format(message))
        return SetBoolResponse(success=1, message=message)

    def _publish_bt(self, data):
        """
        Return JSON representation of the behavior tree
        """
        G = self._to_networkx(data)
        tree = json.dumps(G)

        msg = BehaviorTree(tree=tree)
        self.bt_pub.publish(msg)

    def _publish_bt_status(self):
        """
        Return enabled status of the behavior tree
        """
        msg = Bool(data=self.enabled)
        self.bt_enabled_pub.publish(msg)

    def _publish_bt_update(self, data):
        """
        Return list of active nodes in the behavior tree
        """
        msg = BehaviorTreeNodes(ids=data)
        self.bt_update_pub.publish(msg)

    def _load_project(self, data, names=None):
        names = names or {}
        trees = {}

        rospy.loginfo('Creating BehaviorTree objects')

        # Create the BehaviorTree objects
        for tree in data['trees']:
            tmptree = b3.BehaviorTree()

            tmptree.id = tree['id'] or tree.id
            tmptree.title = tree['title'] or tree.title
            tmptree.description = tree['description'] or tmptree.description
            tmptree.properties = tree['properties'] or tmptree.properties

            trees[tree['id']] = tmptree
            names[tree['id']] = tmptree

        rospy.loginfo('Populating BehaviorTrees with nodes')

        # Create the nodes in each tree
        for tree in data['trees']:
            rospy.loginfo('Creating nodes for: %s' % tree['title'])

            nodes = tree['nodes']
            tmpnodes = {}

            for key in nodes:
                spec = nodes[key]
                name = spec['name']

                rospy.loginfo('Creating node: %s' % name)

                if name in trees:
                    # Node is a tree
                    tmpnodes[key] = trees[name]
                else:
                    if name in names:
                        cls = names[name]
                    elif hasattr(b3, name):
                        cls = getattr(b3, name)
                    else:
                        rospy.logerror('Invalid node name: %s' % name)
                        AttributeError('BehaviorTree.load_project: Invalid node name "%s"' % name)

                    params = spec['properties']
                    node = cls(**params)
                    node.id = spec['id'] or node.id
                    node.title = spec['title'] or node.title
                    node.description = spec['description'] or node.description
                    node.properties = spec['properties'] or node.properties
                    tmpnodes[key] = node

            rospy.loginfo('Connecting nodes')

            # Connect the nodes
            for key in nodes:
                spec = nodes[key]
                node = tmpnodes[key]

                if node.category == b3.COMPOSITE and 'children' in spec:
                    for cid in spec['children']:
                        node.children.append(tmpnodes[cid])
                elif node.category == b3.DECORATOR and 'child' in spec:
                    node.child = tmpnodes[spec['child']]

            rospy.loginfo('Connecting root node')

            # Connect the root node for the tree
            trees[tree['id']].root = tmpnodes[tree['root']]

        root_tree = trees[data['selectedTree']]

        return root_tree

    def _to_networkx(self, tree):
        """
        Generate a networkx graph.

        Parameters
        ----------
        tree: the behavior tree

        Returns
        -------
        networkx.DiGraph: graph
        """
        from networkx.readwrite import json_graph

        def process_tree(G, tree, depth=0):
            G.add_node(tree.id, depth=depth, label=str(tree), category=str(tree.category))
            G.add_node(tree.root.id, depth=depth+1, label=str(tree.root), category=str(tree.root.category))
            G.add_edge(tree.id, tree.root.id)

            process_node(G, tree.root, depth=depth+1)

        def process_node(G, node, depth=0):
            if isinstance(node, b3.BehaviorTree):
                process_tree(G, node, depth=depth)

            if isinstance(node, b3.Composite):
                for c in node.children:
                    G.add_node(c.id, depth=depth, label=str(c), category=str(c.category))
                    G.add_edge(node.id, c.id)
                    process_node(G, c, depth=depth)

        G = nx.OrderedDiGraph()
        G.set_name = tree.id

        process_tree(G, tree)

        nx.drawing.nx_agraph.write_dot(G, homedir + '/catkin_ws/src/cyborg_behavior_tree/cyborg_bt/src/tree.dot')

        return json_graph.tree_data(G, root=tree.id)

    def _to_pydot(self, tree):
        """
        Generate a pydot graph.

        Parameters
        ----------
        tree: the behavior tree

        Returns
        -------
        pydot.Dot: graph
        """
        import pydot

        def process_tree(graph, tree):
            root = tree.root
            graph.set_name = tree.title
            graph.add_node(root.graph_node)
            process_node(graph, root)

        def process_node(graph, node):
            if isinstance(node, b3.Composite):
                for c in node.children:
                    if isinstance(c, b3.BehaviorTree):
                        subgraph = pydot.Subgraph(c.title)
                        graph.add_subgraph(subgraph)
                        graph.add_edge(node.graph_edge(c.root))
                        process_tree(subgraph, c)
                    else:
                        graph.add_node(c.graph_node)
                        graph.add_edge(node.graph_edge(c))
                        process_node(graph, c)

        graph = pydot.Dot(graph_type='digraph')
        process_tree(graph, tree)

        graph.write_png(homedir + '/catkin_ws/src/cyborg_behavior_tree/cyborg_bt/src/test.png')

        return graph

    @property
    def open_nodes(self):
        nodes = []
        for node in self._prev_open_nodes:
            nodes.append(node)

            # Check if there are any behavior trees that are active as well
            if isinstance(node, b3.Composite):
                for child in node.children:
                    if isinstance(child, b3.BehaviorTree) and child.root in self._prev_open_nodes:
                        nodes.append(child)
        return nodes

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            while not self.enabled:
                rate.sleep()

            open_nodes = self.blackboard.get('open_nodes', self.root.id)

            if set(open_nodes) != set(self._prev_open_nodes):
                self._prev_open_nodes = open_nodes
                self._publish_bt_update([node.id for node in self.open_nodes])

            self.root.tick(self.target, self.blackboard)
            rate.sleep()


if __name__ == "__main__":
    bt = BehaviorTreeManager(start_running=True)
