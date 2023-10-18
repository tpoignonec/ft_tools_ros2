import sys
import time
from PyQt5.QtWidgets import QMainWindow, QApplication, QMenu, QAction, QStyle, qApp
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QTimer
from ft_gui.ft_calibration_window import Ui_MainWindow

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from std_srvs.srv import Trigger
from ft_msgs.srv import GetCalibration

# from https://github.com/tasada038/pyqt_ros2_app/tree/master

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.srv_calibration_namespace = '/ft_calibration_node'
        self.add_calibration_srv_name = '/add_calibration_sample'
        self.get_calibration_srv_name = '/get_calibration'
        self.save_calibration_srv_name = '/save_calibration'
        self.reset_calibration_srv_name = '/reset'

        self.icon_path = "image/qt_ros_logo.png"
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowIcon(QIcon(self.icon_path))
        self.create_menubars()
        self.show()

        self.ui.checkBox_calib_connect_ros.stateChanged.connect(self.connect_ros_calib)

        self.ui.pushButton_add_calib_sample.clicked.connect(self.callback_add_calibration_sample)
        self.ui.pushButton_reset.clicked.connect(self.callback_reset_calibration)
        self.ui.pushButton_get_calibration.clicked.connect(self.callback_get_calibration)
        self.ui.pushButton_save_calibration.clicked.connect(self.callback_save_calibration)

        self.ui.label_service_namespace.setText(
            self.srv_calibration_namespace
        )
        self.ui.lineEdit_label_service_namespace.editingFinished.connect(
            self.change_add_calibration_srv_namespace
        )

    def connect_ros_calib(self, state):
        if (Qt.Checked == state):
            try:
                print('Initializing ROS2 comms...')
                # ROS2 init
                self.node = Node('ft_calibration_gui')
                self.node.get_logger().info(f'Initializing ROS2 comms...')

                # register services
                add_calibration_srv_full_name = self.srv_calibration_namespace + self.add_calibration_srv_name
                self.node.get_logger().info(f'Registering service "{add_calibration_srv_full_name}"...')
                self.cli_add_calibration_srv = self.node.create_client(Trigger, add_calibration_srv_full_name)
                while not self.cli_add_calibration_srv.wait_for_service(timeout_sec=5.0):
                    self.node.get_logger().info(f'Waiting for {add_calibration_srv_full_name} service...')

                get_calibration_srv_full_name = self.srv_calibration_namespace + self.get_calibration_srv_name
                self.node.get_logger().info(f'Registering service "{get_calibration_srv_full_name}"...')
                self.cli_get_calibration_srv = self.node.create_client(GetCalibration, get_calibration_srv_full_name)
                while not self.cli_get_calibration_srv.wait_for_service(timeout_sec=5.0):
                    self.node.get_logger().info(f'Waiting for {get_calibration_srv_full_name} service...')

                save_calibration_srv_full_name = self.srv_calibration_namespace + self.save_calibration_srv_name
                self.node.get_logger().info(f'Registering service "{save_calibration_srv_full_name}"...')
                self.cli_save_calibration_srv = self.node.create_client(Trigger, save_calibration_srv_full_name)
                while not self.cli_save_calibration_srv.wait_for_service(timeout_sec=5.0):
                    self.node.get_logger().info(f'Waiting for {save_calibration_srv_full_name} service...')

                reset_calibration_srv_full_name = self.srv_calibration_namespace + self.reset_calibration_srv_name
                self.node.get_logger().info(f'Registering service "{reset_calibration_srv_full_name}"...')
                self.cli_reset_calibration_srv = self.node.create_client(Trigger, reset_calibration_srv_full_name)
                while not self.cli_reset_calibration_srv.wait_for_service(timeout_sec=5.0):
                    self.node.get_logger().info(f'Waiting for {reset_calibration_srv_full_name} service...')

                is_connected = True
                # Error Handle for rclpy timeout
                if is_connected:
                    self.ui.label_ros2_state_calib.setText("Connected")
                    self.ui.label_ros2_state_calib.setStyleSheet(
                        "color: rgb(255,255,255);"
                        "background-color: rgb(18,230,95);"
                        "border-radius:5px;"
                    )
                else :
                    self.ui.label_ros2_state_calib.setText("Couldn't Connect")
                    self.ui.label_ros2_state_calib.setStyleSheet(
                        "color: rgb(255,255,255);"
                        "background-color: rgb(255,0,51);"
                        "border-radius:5px;"
                    )
            except Exception as error:
                self.ui.label_ros2_state_calib.setText("Couldn't Connect")
                self.ui.label_ros2_state_calib.setStyleSheet(
                    "color: rgb(255,255,255);"
                    "background-color: rgb(255,0,51);"
                    "border-radius:5px;"
                )
                self.node.get_logger().warning(f"An error occurred:{error}")
                self.node.get_logger().error(f'Failled to initialize ROS2 comms!')
        else:
            self.node.destroy_node()
            rclpy.shutdown()

    def callback_add_calibration_sample(self, state):
        add_req = Trigger.Request()
        future_res = self.cli_add_calibration_srv.call_async(add_req)
        rclpy.spin_until_future_complete(self.node, future_res)
        # Read result
        res_service = future_res.result()
        self.ui.label_msg_string.setText(res_service.message)
        self.show()

    def callback_reset_calibration(self, state):
        add_req = Trigger.Request()
        future_res = self.cli_reset_calibration_srv.call_async(add_req)
        rclpy.spin_until_future_complete(self.node, future_res)
        # Read result
        res_service = future_res.result()
        self.ui.label_msg_string.setText("")
        self.show()

    def callback_get_calibration(self, state):
        get_req = GetCalibration.Request()
        future_res = self.cli_get_calibration_srv.call_async(get_req)
        rclpy.spin_until_future_complete(self.node, future_res)
        # Read result
        res_service = future_res.result()
        # Print parameters
        self.ui.label_calibration_mass.setText(str(res_service.ft_calibration.mass))
        self.ui.label_calibration_com.setText(
            str([
                res_service.ft_calibration.sensor_frame_to_com.x,
                res_service.ft_calibration.sensor_frame_to_com.y,
                res_service.ft_calibration.sensor_frame_to_com.z
            ])
        )
        self.ui.label_calibration_force_offsets.setText(
            str([
                res_service.ft_calibration.force_offset.x,
                res_service.ft_calibration.force_offset.y,
                res_service.ft_calibration.force_offset.z
            ])
        )
        self.ui.label_calibration_torque_offsets.setText(
            str([
                res_service.ft_calibration.torque_offset.x,
                res_service.ft_calibration.torque_offset.y,
                res_service.ft_calibration.torque_offset.z
            ])
        )
        #self.ui.label_msg_string.setText(res_service.message)
        self.show()

    def callback_save_calibration(self, state):
        save_req = Trigger.Request()
        future_res = self.cli_save_calibration_srv.call_async(save_req)
        rclpy.spin_until_future_complete(self.node, future_res)
        # Read result
        res_service = future_res.result()
        #self.ui.label_msg_string.setText("")
        self.show()

    def change_add_calibration_srv_namespace(self):
        self.srv_calibration_namespace = self.ui.lineEdit_label_service_namespace.text()
        self.ui.label_service_namespace.setText(
            self.srv_calibration_namespace
        )

    ### QMenu
    def create_menubars(self):
        menuBar = self.menuBar()
        # Creating menus using a QMenu object
        fileMenu = QMenu("&File", self)
        fileMenu.addAction(self.exit_action())
        fileMenu.addMenu(self.prefer_action())

        menuBar.addMenu(fileMenu)
        # Creating menus using a title
        editMenu = menuBar.addMenu("&Edit")
        editMenu.addMenu("Undo")
        helpMenu = menuBar.addMenu("&Help")
        helpMenu.addMenu("Get Started")

    def prefer_action(self):
        preferMenu = QMenu('Preferences', self)
        preferAct = QAction(QIcon('image/setting.jpg'),'Setting', self)
        preferMenu.addAction(preferAct)

        return preferMenu

    def exit_action(self):
       # Exit Action, connect
        exitAction = QAction(self.style().standardIcon(QStyle.SP_DialogCancelButton),
                             '&Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.setStatusTip('Exit application')
        exitAction.triggered.connect(qApp.quit)
        self.statusBar()
        return exitAction


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

if __name__=="__main__":
    main()