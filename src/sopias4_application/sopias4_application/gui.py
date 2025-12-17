#!/usr/bin/env python3
"""This file was mainly edited by Nils Gerth"""
import logging
import os
import subprocess
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
from sopias4_application.layer import RobotLayer as RL
from sopias4_application.planner import Astar as AS
from sopias4_application.ui_object import Ui_MainWindow


class GUI(GUINode):
    started = False
    driving = False
    namespace: str

    def __init__(self) -> None:
        self.ui: Ui_MainWindow  # Needed for autocompletion
        super().__init__(Ui_MainWindow())
        self.node.get_logger().info("GUI Node is up")

        # self.unregister_namespace("/turtle1")
        # self.unregister_namespace("/turtle2")
        # self.unregister_namespace("/turtle3")

    def connect_ros2_callbacks(self):
        # Run everything that depends on ROS2 e.g. subscriptions
        GuiLogger(
            widget=self.ui.textEdit_log,
            node=self.node,
            namespace_filter=self.node.get_namespace(),
        )

        try:
            LabelSubscriptionHandler(
                widget=self.ui.label_battery, node=self.node, message_type=BatteryState
            )
        except Exception as e:
            self.node.get_logger().error(f"Couldn't add LabelSubscriptionHandler: {e}")

    def my_namespace_register(self):
        # self.unregister_namespace("/turtle1")
        # self.unregister_namespace("/turtle2")
        # self.unregister_namespace("/turtle3")
        """
        Liest die ComboDropdown Fläche aus und abhängig vom Index wir der Namespace angemedlet
        """
        self.namespace = self.ui.comboBox_ownRobi.currentText()

    def start_stopp(self):
        """
        Startet oder stoppt den Roboterbetrieb basierend auf der aktuellen Auswahl und dem Betriebsmodus.

        Diese Methode überprüft die Auswahl der Roboter in den ComboBoxen und startet oder stoppt den Betrieb
        entsprechend. Wenn der Betrieb gestartet wird, werden die entsprechenden ROS2-Knoten geladen und die
        Navigation gestartet. Im manuellen Modus werden nur die relevanten Elemente aktiviert. Beim Stoppen
        werden alle Knoten entladen und die Navigation gestoppt.

        Bedingungen für den Start:
        - Ein eigener Roboter muss ausgewählt sein (Index > 0).
        - Der eigene Roboter darf nicht der gleiche wie der gegnerische Roboter sein.
        - Der Betrieb darf noch nicht gestartet sein.

        Bedingungen für den Stopp:
        - Der Betrieb muss bereits gestartet sein.

        Fehlermeldungen werden im error_textBrowser angezeigt.

        """
        if (
            self.ui.comboBox_ownRobi.currentIndex() > 0
            and not self.started
            and (
                self.ui.comboBox_ownRobi.currentIndex()
                is not self.ui.comboBox_oppRobi.currentIndex()
            )
        ):  # mindestens der eigene Robi muss ausgewählt sein
            self.started = True
            if not self.register_namespace(self.namespace):
                print("NAMESPACE SCHON VERGEBEN")
                return
            #############################################
            # self.rsc = RSC(self.namespace)

            ############################################
            self.ui.comboBox_ownRobi.setEnabled(False)
            self.ui.comboBox_oppRobi.setEnabled(False)
            self.ui.start_pushButton.setText("Stopp/Abbruch")
            if not self.ui.checkBox_ManualMode.isChecked():
                self.As = AS(self.namespace)
                self.Rl = RL(self.namespace)
                self.launch_nav_stack()
                self.load_ros2_node(self.As)
                self.load_ros2_node(self.Rl)
                self.elements_enable_while_driving(is_driving=True, is_manual=False)
            else:
                self.elements_enable_while_driving(is_driving=True, is_manual=True)
            self.ui.error_textBrowser.clear()
            self.ui.error_textBrowser.setText("gestartet...")

        elif self.ui.comboBox_ownRobi.currentIndex() == 0 and not self.started:
            self.ui.error_textBrowser.setText(
                "Bitte Auswahl der eigenen Roboters treffen"
            )

        elif (
            self.ui.comboBox_ownRobi.currentIndex()
            is self.ui.comboBox_oppRobi.currentIndex()
            and not self.started
        ):
            self.ui.error_textBrowser.clear()
            self.ui.error_textBrowser.setText(
                "Es können nicht gegner und eigener Roboter der gleiche sein"
            )
        else:
            if not self.ui.checkBox_ManualMode.isChecked():
                self.stop_nav_stack()
                self.unload_ros2_node(self.As)
                self.unload_ros2_node(self.Rl)
                self.elements_enable_while_driving(is_driving=False, is_manual=False)
            else:
                self.elements_enable_while_driving(is_driving=False, is_manual=True)
            # self.unload_ros2_node(self.rsc)

            self.unregister_namespace(self.namespace)
            self.ui.error_textBrowser.clear()
            self.started = False
            self.ui.comboBox_ownRobi.setEnabled(True)
            self.ui.comboBox_oppRobi.setEnabled(True)
            self.ui.start_pushButton.setText("(Re)Start")
            self.ui.error_textBrowser.clear()
            self.ui.error_textBrowser.setText(
                "Alles heruntergefahren, Neustart möglich"
            )
            self.node.get_logger().info("\n IDLE, Neustart möglich\n")

    def elements_enable_while_driving(self, is_driving: bool, is_manual=False):
        """
        Aktiviert oder deaktiviert UI-Elemente basierend auf dem Fahrzustand.

        Diese Funktion steuert die Aktivierung bestimmter UI-Elemente, abhängig davon,
        ob das Fahrzeug fährt und ob der manuelle Modus aktiviert ist.

        Args:
            is_driving (bool): Gibt an, ob das Fahrzeug fährt.
            is_manual (bool, optional): Gibt an, ob der manuelle Modus aktiviert ist. Standard ist False.
        """
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
        # self.drive()
        # Connect the interactions of your UI with callback functions
        self.ui.comboBox_ownRobi.currentIndexChanged.connect(self.my_namespace_register)
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
        """
        Für Manuelle Steuerung die Befehle, Achtung, rotate_rigth/rotate_left sind im Framework seltsamerweise nicht implementert, obwohl es die Doku sagt...
        """
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

        # self.ui.pushButton_dock.pressed.connect(
        #     lambda: Thread(target=self.toggle_docking()).start()
        # )
        # self.ui.pushButton_undock.pressed.connect(
        #     lambda: Thread(target=self.toggle_docking()).start()
        # )
        self.ui.pushButton_dock.pressed.connect(self.dock)
        self.ui.pushButton_undock.pressed.connect(self.undock)
        self.ui.pushButton_stop_nav2.pressed.connect(self.test_stop)
        self.ui.pushButton_start_nav2.pressed.connect(self.test_start)

    def test_start(self):
        self.launch_nav_stack()
        # self.start_mapping()

    def test_stop(self):
        # self.stop_mapping()
        self.stop_nav_stack()

    def set_default_values(self):
        """
        Setzt Standardwerte für verschiedene UI-Elemente in der Anwendung.

        Diese Methode initialisiert die UI-Elemente mit Standardwerten wie vorgefertigtem Text für Textfelder,
        Elemente für Dropdown-Menüs, Tab-Namen, Fenstertitel, Fenster-Icon und den aktivierten/deaktivierten Zustand von Tabs und Checkboxen.

        - Fügt Elemente zu comboBox_ownRobi und comboBox_oppRobi hinzu.
        - Setzt die Texte für Tabs im tabWidget.
        - Setzt den Fenstertitel und das Fenster-Icon.
        - Aktiviert oder deaktiviert spezifische Tabs.
        - Setzt den initialen Tab-Index.
        - Setzt den ausgewählten Zustand der Checkbox für den manuellen Modus.
        """
        # self.unregister_namespace("/turtle1")
        # self.unregister_namespace("/turtle2")
        # self.unregister_namespace("/turtle3")
        # Set default values for UI elements like prefilled text of textfield's or elements of dropdown menus
        self.ui.comboBox_ownRobi.addItems(
            ["keine Auswahl", "/turtle1", "/turtle2", "/turtle3"]
        )
        self.ui.comboBox_oppRobi.addItems(
            ["keine Auswahl", "/turtle1", "/turtle2", "/turtle3"]
        )

        self.ui.tabWidget.setTabText(0, "Übersicht")
        self.ui.tabWidget.setTabText(1, "Log")
        self.ui.tabWidget.setTabText(2, "Manuelle Steuerung")
        self.setWindowTitle("GUI von Team 1 | TurtleTauben")
        self.setWindowIcon(QtGui.QIcon("/home/ws/src/sopias4_application/ui/taube.png"))
        self.ui.tab.setEnabled(True)
        self.ui.tab_2.setEnabled(True)
        self.ui.tab_3.setEnabled(False)
        self.ui.tabWidget.setCurrentIndex(0)
        self.ui.checkBox_ManualMode.setChecked(False)
        # self.ui.pushButton_undock.setEnabled(True)
        # self.ui.pushButton_dock.setEnabled(False)

    # def set_initial_disabled_elements(self):
    #     # Disable elements which shouldn't be interactable at initial startup of the window e.g. buttons which shouldn't be pressed before another condition is met
    #     self.ui.pushButton

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
    # widget.register_namespace("NILS_Namespace")
    widget.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
