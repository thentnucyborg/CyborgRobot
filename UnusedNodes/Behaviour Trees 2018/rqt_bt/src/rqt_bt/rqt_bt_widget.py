#!/usr/bin/env python

import os
import rospy
import rospkg
import rosgraph.impl.graph

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsScene, QWidget

from rqt_graph.interactive_graphics_view import InteractiveGraphicsView
from rqt_py_common.topic_completer import TopicCompleter
from qt_dotgraph.dot_to_qt import DotToQtGenerator

from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from .btdata import BTData
from .dotcode import RosBTDotcodeGenerator

class BTWidget(QWidget):
    _redraw_interval = 40
    _deferred_fit_in_view = Signal()

    def __init__(self):
        super(BTWidget, self).__init__()

        self.setObjectName('BTWidget')

        self._graph = None
        self._current_dotcode = None
        self._initialized = False

        # dot_to_qt transforms into Qt elements using dot layout
        self.dot_to_qt = DotToQtGenerator()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_bt'), 'resource', 'rqt_bt.ui')
        loadUi(ui_file, self, {'InteractiveGraphicsView': InteractiveGraphicsView})

        self.refresh_timer = QTimer(self)
        self.refresh_timer.start(self._redraw_interval)
        self.refresh_timer.timeout.connect(self._refresh_rosgraph)

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self.graphics_view.setScene(self._scene)

        self.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self.refresh_graph_push_button.clicked.connect(self._update_rosgraph)

        self.highlight_connections_check_box.toggled.connect(self._redraw_graph_view)
        self.auto_fit_graph_check_box.toggled.connect(self._redraw_graph_view)

        self.fit_in_view_push_button.setIcon(QIcon.fromTheme('zoom-original'))
        self.fit_in_view_push_button.clicked.connect(self._fit_in_view)

        self.depth_spin_box.setMinimum(-1)
        self.depth_spin_box.valueChanged.connect(self._refresh_rosgraph)

        self.save_dot_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self.save_dot_push_button.clicked.connect(self._save_dot)

        self.save_as_svg_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self.save_as_svg_push_button.clicked.connect(self._save_svg)

        self.save_as_image_push_button.setIcon(QIcon.fromTheme('image'))
        self.save_as_image_push_button.clicked.connect(self._save_image)

        self.run_push_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.run_push_button.clicked.connect(self._run_bt)

        self._update_rosgraph()
        self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()

        # generator builds tree graph
        bt_sub_name = '/cyborg/bt/behavior_tree'
        bt_update_sub_name = '/cyborg/bt/behavior_tree_updates'
        bt_enabled_sub_name = '/cyborg/bt/enabled'
        bt_enable_srv_name = '/cyborg/bt/enable'

        self._bt_enabled = True
        rospy.Subscriber(bt_enabled_sub_name, Bool, self._bt_enabled_cb)
        self._bt_enable_srv = rospy.ServiceProxy(bt_enable_srv_name, SetBool)

        bt_data = BTData(bt_sub_name, bt_update_sub_name)
        self.dotcode_generator = RosBTDotcodeGenerator(bt_data)

    def _bt_enabled_cb(self, msg):
        self._bt_enabled = msg.data

        if self._bt_enabled:
            self.run_push_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        else:
            self.run_push_button.setIcon(QIcon.fromTheme('media-playback-start'))

    def _run_bt(self):
        enable_req = False if self._bt_enabled else True

        if self._bt_enable_srv(data=enable_req).success:
            self._bt_enabled = enable_req

    def _update_rosgraph(self):
        self._graph = rosgraph.impl.graph.Graph()
        self._graph.set_master_stale(5.0)
        self._graph.set_node_stale(5.0)
        self._graph.update()
        self._refresh_rosgraph()

    def _refresh_rosgraph(self):
        if not self._initialized:
            return
        self._update_graph_view(self._generate_dotcode())

    def _generate_dotcode(self):
        depth = self.depth_spin_box.value()

        return self.dotcode_generator.generate_dotcode(
                max_depth=depth)

    def _update_graph_view(self, dotcode):
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._redraw_graph_view()

    def _generate_tool_tip(self, url):
        if url is not None and ':' in url:
            item_type, item_path = url.split(':', 1)
            if item_type == 'node':
                tool_tip = 'Node:\n  %s' % (item_path)
                service_names = rosservice.get_service_list(node=item_path)
                if service_names:
                    tool_tip += '\nServices:'
                    for service_name in service_names:
                        try:
                            service_type = rosservice.get_service_type(service_name)
                            tool_tip += '\n  %s [%s]' % (service_name, service_type)
                        except rosservice.ROSServiceIOException as e:
                            tool_tip += '\n  %s' % (e)
                return tool_tip
            elif item_type == 'topic':
                topic_type, topic_name, _ = rostopic.get_topic_type(item_path)
                return 'Topic:\n  %s\nType:\n  %s' % (topic_name, topic_type)
        return url

    def _redraw_graph_view(self):
        self._scene.clear()

        if self.highlight_connections_check_box.isChecked():
            highlight_level = 3
        else:
            highlight_level = 1

        # layout graph and create qt items
        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                            highlight_level=highlight_level,
                                                            same_label_siblings=True)

        for node_item in nodes.values():
            self._scene.addItem(node_item)
        for edge_items in edges.values():
            for edge_item in edge_items:
                edge_item.add_to_scene(self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())
        if self.auto_fit_graph_check_box.isChecked():
            self._fit_in_view()

    def _fit_in_view(self):
        self.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

    def _save_dot(self):
        file_name, _ = QFileDialog.getSaveFileName(self, self.tr('Save as DOT'), 'rosgraph.dot', self.tr('DOT graph (*.dot)'))
        if file_name is None or file_name == '':
            return

        handle = QFile(file_name)
        if not handle.open(QIODevice.WriteOnly | QIODevice.Text):
            return

        handle.write(self._current_dotcode)
        handle.close()

    def _save_svg(self):
        file_name, _ = QFileDialog.getSaveFileName(self, self.tr('Save as SVG'), 'rosgraph.svg', self.tr('Scalable Vector Graphic (*.svg)'))
        if file_name is None or file_name == '':
            return

        generator = QSvgGenerator()
        generator.setFileName(file_name)
        generator.setSize((self._scene.sceneRect().size() * 2.0).toSize())

        painter = QPainter(generator)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()

    def _save_image(self):
        file_name, _ = QFileDialog.getSaveFileName(self, self.tr('Save as image'), 'rosgraph.png', self.tr('Image (*.bmp *.jpg *.png *.tiff)'))
        if file_name is None or file_name == '':
            return

        img = QImage((self._scene.sceneRect().size() * 2.0).toSize(), QImage.Format_ARGB32_Premultiplied)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()
        img.save(file_name)

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('auto_fit_graph_check_box_state', self.auto_fit_graph_check_box.isChecked())
        instance_settings.set_value('highlight_connections_check_box_state', self.highlight_connections_check_box.isChecked())
        instance_settings.set_value('depth_spin_box_value', self.depth_spin_box.value())

    def restore_settings(self, plugin_settings, instance_settings):
        self.auto_fit_graph_check_box.setChecked(instance_settings.value('auto_fit_graph_check_box_state', True) in [True, 'true'])
        self.highlight_connections_check_box.setChecked(instance_settings.value('highlight_connections_check_box_state', True) in [True, 'true'])
        self.depth_spin_box.setValue(int(instance_settings.value('depth_spin_box_value', -1)))

        self._initialized = True
        self._refresh_rosgraph()
