from __future__ import division

from python_qt_binding.QtCore import QAbstractListModel, QFile, QIODevice, QTimer, Qt, Signal, Slot
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from python_qt_binding.QtWidgets import QCompleter, QFileDialog, QGraphicsScene, QWidget
from python_qt_binding.QtSvg import QSvgGenerator

import rosgraph.impl.graph
import rosservice
import rostopic

from rqt_gui_py.plugin import Plugin

from .rqt_bt_widget import BTWidget

class RosBT(Plugin):
    def __init__(self, context):
        super(RosBT, self).__init__(context)
        self.setObjectName('ros_bt')

        self._widget = BTWidget()

        context.add_widget(self._widget)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
