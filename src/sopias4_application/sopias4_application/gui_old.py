#!/usr/bin/env python3
import sys
from threading import Thread

from nav_msgs.msg import OccupancyGrid
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QColor, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QGraphicsPixmapItem,
    QGraphicsScene,
    QGraphicsView,
)
from sensor_msgs.msg import BatteryState
from sopias4_framework.nodes.gui_node import GUINode
from sopias4_framework.tools.gui.gui_logger import GuiLogger
from sopias4_framework.tools.gui.label_subscription_handle import (
    LabelSubscriptionHandler,
)
from sopias4_framework.tools.ros2.node_tools import LaunchService as LS

# from sopias4_application.controller import RotationStraightController as RSC
from sopias4_application.layer_plugin import LayerPlugin as LP
from sopias4_application.planner_plugin import PlannerPlugin as PP
from sopias4_application.ui_object import Ui_MainWindow


class GUI(GUINode):
    started = False
    driving = False
    namespace: str

    def __init__(self) -> None:
        self.ui: Ui_MainWindow
        super().__init__(Ui_MainWindow())

    def namespace_register(self):
        self.namespace = self.ui.comboBox_ownRobi.currentText()

    def start_stopp(self):
        if (
            self.ui.comboBox_ownRobi.currentIndex() > 0
            and not self.started
            and (
                self.ui.comboBox_ownRobi.currentIndex()
                is not self.ui.comboBox_oppRobi.currentIndex()
            )
        ):
            self.started = True
            if not self.register_namespace(self.namespace):
                print("NAMESPACE ALREADY AVAILABLE")
                return
            #############################################
            # self.rsc = RSC(self.namespace)

            ############################################
            self.ui.comboBox_ownRobi.setEnabled(False)
            self.ui.comboBox_oppRobi.setEnabled(False)
            self.ui.start_pushButton.setText("Stop/aborted")
            if not self.ui.checkBox_ManualMode.isChecked():
                self.Pp = PP(self.namespace)
                self.Lp = LP(self.namespace)
                self.launch_nav_stack()
                self.load_ros2_node(self.Pp)
                self.load_ros2_node(self.Lp)
                self.elements_enable_while_driving(is_driving=True, is_manual=False)
            else:
                self.elements_enable_while_driving(is_driving=True, is_manual=True)
            self.ui.error_textBrowser.clear()
            self.ui.error_textBrowser.setText("Started...")

        elif self.ui.comboBox_ownRobi.currentIndex() == 0 and not self.started:
            self.ui.error_textBrowser.setText("Please choose your robot")

        elif (
            self.ui.comboBox_ownRobi.currentIndex()
            is self.ui.comboBox_oppRobi.currentIndex()
            and not self.started
        ):
            self.ui.error_textBrowser.clear()
            self.ui.error_textBrowser.setText(
                "Opponent robot and own robot can't be the identical"
            )
        else:
            if not self.ui.checkBox_ManualMode.isChecked():
                self.stop_nav_stack()
                self.unload_ros2_node(self.Pp)
                self.unload_ros2_node(self.Lp)
                self.elements_enable_while_driving(is_driving=False, is_manual=False)
            else:
                self.elements_enable_while_driving(is_driving=False, is_manual=True)
            # self.unload_ros2_node(self.rsc)

            self.unregister_namespace(self.namespace)
            self.ui.error_textBrowser.clear()
            self.started = False
            self.ui.comboBox_ownRobi.setEnabled(True)
            self.ui.comboBox_oppRobi.setEnabled(True)
            self.ui.start_pushButton.setText("Start")
            self.ui.error_textBrowser.clear()
            self.ui.error_textBrowser.setText(
                "Everything shut down, restart is available"
            )
            self.node.get_logger().info("\n IDLE, Restart possible\n")

    def elements_enable_while_driving(self, is_driving: bool, is_manual=False):
        if is_driving and not is_manual:
            self.ui.tab_3.setEnabled(False)
            self.ui.checkBox_ManualMode.setCheckable(False)
        if is_driving and is_manual:
            self.ui.tab_3.setEnabled(True)
            self.ui.checkBox_ManualMode.setCheckable(True)
            self.ui.pushButton_stop_nav2.setEnabled(True)
            self.ui.pushButton_start_nav2.setEnabled(True)
        else:
            self.ui.tab_3.setEnabled(False)
            self.ui.checkBox_ManualMode.setCheckable(True)

    def connect_ui_callbacks(self):
        # TODO: first implementation, test if it actually works with real robot
        self.ui.comboBox_ownRobi.currentIndexChanged.connect(self.namespace_register)
        self.ui.start_pushButton.pressed.connect(self.start_stopp)
        self.ui.comboBox_oppRobi.currentIndexChanged.connect(
            lambda: Thread(
                print(
                    "combBox_Opp: ",
                    self.ui.comboBox_oppRobi.currentText(),
                    self.ui.comboBox_oppRobi.currentIndex(),
                )
            ).start()
        )
        self.ui.pushButton_right.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="right",
                    vel_rel=float(self.ui.horizontalSlider_velo.value() / 100),
                )
            ).start()
        )
        self.ui.pushButton_left.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="left",
                    vel_rel=float(self.ui.horizontalSlider_velo.value() / 100),
                )
            ).start()
        )
        self.ui.pushButton_fwd.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="forward",
                    vel_rel=float(self.ui.horizontalSlider_velo.value() / 100),
                )
            ).start()
        )
        self.ui.pushButton_backw.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="backward",
                    vel_rel=float(self.ui.horizontalSlider_velo.value() / 100),
                )
            ).start()
        )
        self.ui.pushButton_stopdrive.pressed.connect(
            lambda: Thread(
                target=self.drive(
                    direction="stop",
                    vel_rel=float(self.ui.horizontalSlider_velo.value() / 100),
                )
            ).start()
        )

        self.ui.pushButton_dock.pressed.connect(self.dock)
        self.ui.pushButton_undock.pressed.connect(self.undock)
        self.ui.pushButton_stop_nav2.pressed.connect(self.test_stop)
        self.ui.pushButton_start_nav2.pressed.connect(self.test_start)

    def set_default_values(self):
        self.ui.comboBox_ownRobi.addItems(
            ["keine Auswahl", "/turtle1", "/turtle2", "/turtle3", "/turtle4"]
        )
        self.ui.comboBox_oppRobi.addItems(
            ["keine Auswahl", "/turtle1", "/turtle2", "/turtle3", "/turtle4"]
        )

        self.ui.tabWidget.setTabText(0, "Overview")
        self.ui.tabWidget.setTabText(1, "Logs")
        self.ui.tabWidget.setTabText(2, "Manual control")
        self.setWindowTitle("User Interface")
        self.setWindowIcon(
            QtGui.QIcon("/home/ws/src/sopias4_application/ui/group_icon.png")
        )
        self.ui.tab.setEnabled(True)
        self.ui.tab_2.setEnabled(True)
        self.ui.tab_3.setEnabled(False)
        self.ui.tabWidget.setCurrentIndex(0)
        self.ui.checkBox_ManualMode.setChecked(False)

    # def set_initial_enabled_elements(self):
    #     if hasattr(self.ui, "pushButton_example"):
    #         self.ui.pushButton_example.setEnabled(False)

    # def connect_ros2_callbacks(self):
    #     if hasattr(self.ui, "textEdit"):
    #         GuiLogger(
    #             widget=self.ui.textEdit,
    #             node=self.node,
    #             namespace_filter=self.node.get_namespace(),
    #         )

    def test_start(self):
        self.launch_nav_stack()
        # self.start_mapping()

    def test_stop(self):
        # self.stop_mapping()
        self.stop_nav_stack()

    def closeEvent(self, event):
        # Do stuff when the close button of the window is pressed
        self.unregister_namespace(self.namespace)
        super().closeEvent(event)

    def destroy_node(self):
        # Do stuff when the underlying node is destroyed
        super().destroy_node()


def main():
    app = QApplication(sys.argv)
    widget = GUI()
    widget.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
