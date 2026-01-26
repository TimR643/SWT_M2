#!/usr/bin/env python3
import os
import shutil
import sys
from threading import Thread

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QColor, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QGraphicsPixmapItem,
    QGraphicsScene,
    QGraphicsView,
)
from sensor_msgs.msg import BatteryState

# from sopias4_application.controller import RotationStraightController as RSC
from sopias4_application.layer_plugin import LayerPlugin as LP
from sopias4_application.planner_plugin import PlannerPlugin as PP
from sopias4_application.ui_object import Ui_MainWindow
from sopias4_framework.nodes.gui_node import GUINode
from sopias4_framework.tools.gui.gui_logger import GuiLogger
from sopias4_framework.tools.gui.label_subscription_handle import (
    LabelSubscriptionHandler,
)
from sopias4_framework.tools.ros2.node_tools import LaunchService as LS


class GUI(GUINode):
    started = False
    driving = False
    namespace: str

    def __init__(self) -> None:
        self.ui: Ui_MainWindow
        self.user_custom_yaml = None
        self.perform_trick_after_first_goal = False
        self.config_dir = os.path.join(
            get_package_share_directory("sopias4_application"), "config"
        )
        self.target_yaml_path = os.path.join(self.config_dir, "nav2.yaml")
        self.default_yaml_path = os.path.join(self.config_dir, "nav2_base.yaml")
        self.vs_yaml_path = os.path.join(self.config_dir, "nav2_vs.yaml")
        self.mode_yaml_map = {
            "Zeitrennen": self.default_yaml_path,
            "VS-Rennen": self.vs_yaml_path,
            "Schnitzeljagd": self.default_yaml_path,
        }
        super().__init__(Ui_MainWindow())

    def namespace_register(self):
        self.namespace = self.ui.comboBox_ownRobi.currentText()

    def start_stopp(self):
        if (
            self.ui.comboBox_ownRobi.currentIndex() > 0
            and not self.started
            and (self.ui.comboBox_ownRobi.currentIndex())
        ):
            self.started = True
            if not self.register_namespace(self.namespace):
                print("NAMESPACE ALREADY AVAILABLE")
                return
            #############################################
            # self.rsc = RSC(self.namespace)

            ############################################
            self.ui.comboBox_ownRobi.setEnabled(False)
            # Lock the checkbox so user can't uncheck it while driving
            # self.ui.spoofing_checkBox.setEnabled(False)
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

            # --- DOCKER START LOGIC ---
            # if self.ui.spoofing_checkBox.isChecked():
            #     try:
            #         self.ui.error_textBrowser.append("Starting Docker Spoofing...")
            #         # Using Popen prevents the GUI from freezing
            #         subprocess.Popen(
            #             ["docker", "compose", "-f", docker_compose_file, "up", "-d"],
            #             stdout=subprocess.PIPE,
            #             stderr=subprocess.PIPE,
            #         )
            #     except Exception as e:
            #         self.ui.error_textBrowser.append(f"Docker Start Error: {e}")

        elif self.ui.comboBox_ownRobi.currentIndex() == 0 and not self.started:
            self.ui.error_textBrowser.setText("Please choose your robot")
        else:
            # --- DOCKER STOP LOGIC ---
            # We check if it was checked before stopping everything
            # if self.ui.spoofing_checkBox.isChecked():
            #     try:
            #         self.ui.error_textBrowser.append("Stopping Docker Spoofing...")
            #         subprocess.Popen(
            #             ["docker", "compose", "-f", docker_compose_file, "down"],
            #             stdout=subprocess.PIPE,
            #             stderr=subprocess.PIPE,
            #         )
            #     except Exception as e:
            #         self.ui.error_textBrowser.append(f"Docker Stop Error: {e}")

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

            # Re-enable UI elements
            self.ui.comboBox_ownRobi.setEnabled(True)
            # self.ui.spoofing_checkBox.setEnabled(True)

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
        self.ui.comboBox_ownRobi.currentIndexChanged.connect(self.namespace_register)
        self.ui.comboBox_mode_select.currentIndexChanged.connect(
            lambda: self.apply_mode_selection(self.ui.comboBox_mode_select.currentText())
        )
        self.ui.start_pushButton.pressed.connect(self.start_stopp)
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

        self.ui.pushButton_Stop.pressed.connect(self.emergency_stop)

    def set_default_values(self):
        self.ui.comboBox_ownRobi.addItems(
            ["keine Auswahl", "/turtle1", "/turtle2", "/turtle3", "/turtle4"]
        )
        self.ui.comboBox_mode_select.clear()
        self.ui.comboBox_mode_select.addItems(
            ["Zeitrennen", "VS-Rennen", "Schnitzeljagd"]
        )
        self.ui.comboBox_mode_select.setCurrentIndex(0)
        self.apply_mode_selection(self.ui.comboBox_mode_select.currentText())

        self.ui.tabWidget.setTabText(0, "Overview")
        self.ui.tabWidget.setTabText(1, "Logs")
        self.ui.tabWidget.setTabText(2, "Manual control")
        self.setWindowTitle("User Interface")
        self.setWindowIcon(
            QtGui.QIcon(
                "/home/ws/src/sopias4_application/ui/group_icon.png"
            )  # TODO: Add icon path!!!
        )
        self.ui.tab.setEnabled(True)
        self.ui.tab_2.setEnabled(True)
        self.ui.tab_3.setEnabled(False)
        self.ui.tabWidget.setCurrentIndex(0)
        self.ui.checkBox_ManualMode.setChecked(False)

    def emergency_stop(self):
        """
        Executes a hard stop: cancels navigation, halts robot, and resets internal state.
        """
        self.node.get_logger().warn("!!! EMERGENCY STOP TRIGGERED !!!")

        # 1. Immediate Shutdown of Navigation
        # Stopping the stack first prevents new heavy path planning calculations
        self.stop_nav_stack()

        # 2. Overpower Nav2 Velocity (The "Spam Stop")
        # Nav2 nodes might take a few seconds to die, during which they keep sending cmd_vel.
        # We spawn a thread to flood the topic with 0-velocity commands to ensure the robot stops.
        def spam_stop():
            import time

            # Send stop command 20 times over 2 seconds
            for _ in range(20):
                self.drive(direction="stop", vel_rel=0.0)
                time.sleep(0.1)

        # Daemon=True ensures this thread dies if the main app closes
        Thread(target=spam_stop, daemon=True).start()

        # 3. Cleanup Nodes
        if hasattr(self, "Pp"):
            self.unload_ros2_node(self.Pp)
        if hasattr(self, "Lp"):
            self.unload_ros2_node(self.Lp)

        # 4. Reset UI/Logic State
        self.started = False
        self.driving = False

        self.ui.start_pushButton.setText("Start")
        self.ui.comboBox_ownRobi.setEnabled(True)
        # self.ui.spoofing_checkBox.setEnabled(True)

        # 5. Enable Manual Controls for Recovery
        # We explicitly check the manual box and call the enable function
        self.ui.checkBox_ManualMode.setChecked(True)
        self.elements_enable_while_driving(is_driving=False, is_manual=True)

    def apply_mode_selection(self, mode_name: str) -> None:
        config_path = self.mode_yaml_map.get(mode_name)
        if config_path is None:
            self.ui.error_textBrowser.append(f"Unbekannter Modus: {mode_name}")
            return
        if not os.path.exists(config_path):
            self.ui.error_textBrowser.append(
                f"Modus-Config nicht gefunden: {config_path}"
            )
            return
        try:
            shutil.copyfile(config_path, self.target_yaml_path)
        except Exception as exc:
            self.ui.error_textBrowser.append(
                f"Config konnte nicht geladen werden: {exc}"
            )
            return
        self.perform_trick_after_first_goal = mode_name == "Schnitzeljagd"
        if self.perform_trick_after_first_goal:
            self.ui.error_textBrowser.append(
                "Schnitzeljagd aktiv: Kunststück nach erstem Zielpunkt vorgesehen."
            )
        self.ui.error_textBrowser.append(
            f"Modus gesetzt: {mode_name} (Config: {os.path.basename(config_path)})"
        )

    # def upload_yaml(self):
    #     # Öffnet den Datei-Dialog
    #     options = QFileDialog.Options()
    #     file_path, _ = QFileDialog.getOpenFileName(
    #         None,
    #         "Choose Nav2 Config",
    #         "",
    #         "YAML Files (*.yaml *.yml);;All Files (*)",
    #         options=options,
    #     )

    #     if file_path and os.path.exists(file_path):
    #         self.user_custom_yaml = file_path
    #         self.ui.error_textBrowser.append(
    #             f"Config chosen: {os.path.basename(file_path)}"
    #         )
    #         self.ui.pushButton_load_yaml.setStyleSheet("background-color: lightgreen")

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
