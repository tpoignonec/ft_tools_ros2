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

# from https://github.com/tasada038/pyqt_ros2_app/tree/master

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.srv_calibration_namespace = '/ft_calibration_node'
        self.add_calibration_srv_name = '/add_calibration_sample'
        self.get_calibration_srv_name = '/get_calibration_sample'
        self.save_calibration_srv_name = '/save_calibration_sample'
        self.reset_srv_name = '/reset'

        self.icon_path = "image/qt_ros_logo.png"
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowIcon(QIcon(self.icon_path))
        self.create_menubars()
        self.show()

        self.ui.checkBox_calib_connect_ros.stateChanged.connect(self.connect_ros_calib)
        self.ui.pushButton_add_calib_sample.clicked.connect(self.callback_add_calibration_sample)
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

                self.node.get_logger().info(f'Registering service "{self.get_calibration_srv_name}"...')
                self.cli_add_calibration_srv = self.node.create_client(Trigger, add_calibration_srv_full_name)
                while not self.cli_add_calibration_srv.wait_for_service(timeout_sec=5.0):
                    self.node.get_logger().info(f'Waiting for {add_calibration_srv_full_name} service...')

                is_connected = True

                '''
                # spin once, timeout_sec 5[s]
                timeout_sec_rclpy = 5
                timeout_init = time.time()
                rclpy.spin_once(self.node)
                timeout_end = time.time()
                ros_connect_time = timeout_end - timeout_init
                is_connected = ros_connect_time >= timeout_sec_rclpy
                '''

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
        '''
        if (Qt.Checked == state):
            # create timer
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.timer_float_update)
            self.timer.start(10)
        else:
            self.timer.stop()
        '''
        res_service = self.add_calibration_sample()
        self.ui.label_msg_string.setText(res_service.message)
        self.show()

    def add_calibration_sample(self):
        #rclpy.spin_once(self.node)
        '''
        self.update_float_data_label()
        self.show()
        self.timer.start(10)
        '''
        add_req = Trigger.Request()
        future_res = self.cli_add_calibration_srv.call_async(add_req)
        rclpy.spin_until_future_complete(self.node, future_res)
        return future_res.result()

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