import sys
import rospy
from std_msgs.msg import String
from PySide2 import QtWidgets, QtGui, QtCore
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry

class ROSSubscriber(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_ros()

        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self.update_all)
        self.update_rate = 1
        self.update_timer.start(self.update_rate * 1000)  # Update rates every 1 second

    def init_ros(self):
        rospy.init_node("pyside_ros_subscriber")
        # self.camera_prefixes = ['/camera_topic_1']
        self.camera_prefixes = ['/multisense/left/image_rect_color']
        self.lidar_prefixes = ['/velodyne_1/velodyne_points', '/velodyne_2/velodyne_points']
        # self.lidar_prefixes = ['/lidar_topic_1', '/lidar_topic_2']

        self.camera_rates = {prefix: 0 for prefix in self.camera_prefixes}
        self.lidar_rates = {prefix: 0 for prefix in self.lidar_prefixes}
        self.camera_boxes = {}
        self.lidar_boxes = {}
        self.gps_boxes = {}
        self.subscribe_to_topics()

    def init_ui(self):
        self.setWindowTitle("ROS Subscriber with PySide")
        self.resize(400, 300)

        self.camera_label = QtWidgets.QLabel("Camera Topics:")
        self.camera_layout = QtWidgets.QVBoxLayout()

        self.lidar_label = QtWidgets.QLabel("Lidar Topics:")
        self.lidar_layout = QtWidgets.QVBoxLayout()

        self.gps_label = QtWidgets.QLabel("GPS Topics:")
        self.gps_layout = QtWidgets.QVBoxLayout()

        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(self.camera_label)
        main_layout.addLayout(self.camera_layout)
        main_layout.addWidget(self.lidar_label)
        main_layout.addLayout(self.lidar_layout)
        main_layout.addWidget(self.gps_label)
        main_layout.addLayout(self.gps_layout)

        self.setLayout(main_layout)

        self.setStyleSheet(
            "QFrame { background-color: #f0f0f0; border-radius: 10px; padding: 5px; }"
            "QLabel { font-size: 16px; }"
        )

    def subscribe_to_topics(self):
        for topic in self.camera_prefixes:
                self.camera_boxes[topic] = self.create_rate_box()
                self.camera_layout.addWidget(self.camera_boxes[topic])
                rospy.Subscriber(topic, Image, self.camera_rate_callback, callback_args=topic)
        for topic in self.lidar_prefixes:
            self.lidar_boxes[topic] = self.create_rate_box()
            self.lidar_layout.addWidget(self.lidar_boxes[topic])
            rospy.Subscriber(topic, PointCloud2, self.lidar_rate_callback, callback_args=topic)

        rospy.Subscriber('/odometry/filtered_odom', Odometry, self.gps_callback)
        self.gps_rates = {'/odometry/filtered_odom' : 0}
        self.gps_boxes['/odometry/filtered_odom'] = self.create_rate_box()
        self.gps_layout.addWidget(self.gps_boxes['/odometry/filtered_odom'])
        self.gps_boxes['GPS_coord'] = self.create_box("GPS XY Good")
        self.gps_layout.addWidget(self.gps_boxes['GPS_coord'])
        self.gps_xy = [0,0]



    def create_rate_box(self):
        box = QtWidgets.QFrame()
        box.setFixedSize(200, 50)
        box.setAutoFillBackground(True)
        box.setStyleSheet("background-color: green;")
        box.label = QtWidgets.QLabel("0 Hz")
        layout = QtWidgets.QVBoxLayout(box)
        layout.addWidget(box.label)
        return box

    def create_box(self, init_label):
        box = QtWidgets.QFrame()
        box.setFixedSize(200, 50)
        box.setAutoFillBackground(True)
        box.setStyleSheet("background-color: green;")
        box.label = QtWidgets.QLabel(init_label)
        layout = QtWidgets.QVBoxLayout(box)
        layout.addWidget(box.label)
        return box

    @QtCore.Slot(Image)
    def camera_rate_callback(self, msg, topic):
        # topic = rospy.get_caller_id()
        # print(topic)
        self.camera_rates[topic] += 1

    @QtCore.Slot(PointCloud2)
    def lidar_rate_callback(self, msg, topic):
        # topic = rospy.get_caller_id()
        self.lidar_rates[topic] += 1

    @QtCore.Slot(Odometry)
    def gps_callback(self, msg):
        # topic = rospy.get_caller_id()
        self.gps_rates['/odometry/filtered_odom'] += 1
        self.gps_xy = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def update_all(self):
        self.update_rates()
        self.update_other()

    def update_rates(self):
        for topic, box in self.camera_boxes.items():
            # print(topic)
            rate = self.camera_rates[topic] / self.update_rate
            hz = rate/self.update_rate
            box.label.setText(topic + ": " + f"{hz:.2f} Hz")
            self.update_box_color(box, rate)
            self.camera_rates[topic] = 0
            box.setFixedSize(box.sizeHint())

        for topic, box in self.lidar_boxes.items():
            rate = self.lidar_rates[topic] / self.update_rate
            hz = rate/self.update_rate
            box.label.setText(topic + ": " + f"{hz:.2f} Hz")
            self.update_box_color(box, rate)
            self.lidar_rates[topic] = 0
            box.setFixedSize(box.sizeHint())

        for topic, box in self.gps_boxes.items():
            if topic[0] == '/':
                rate = self.gps_rates[topic] / self.update_rate
                hz = rate/self.update_rate
                box.label.setText(topic + ": " + f"{hz:.2f} Hz")
                self.update_box_color(box, rate)
                self.gps_rates[topic] = 0
                box.setFixedSize(box.sizeHint())

    def update_other(self):
        if self.gps_xy[0] < 4000000 or self.gps_xy[1] > -502700:
            self.gps_boxes['GPS_coord'].label.setText("XY Vals Bad")
            self.gps_boxes['GPS_coord'].setStyleSheet("background-color: red;")
            self.gps_boxes['GPS_coord'].setFixedSize(self.gps_boxes['GPS_coord'].sizeHint())
        else:
            self.gps_boxes['GPS_coord'].label.setText("XY Vals Good")
            self.gps_boxes['GPS_coord'].setStyleSheet("background-color: green;")
            self.gps_boxes['GPS_coord'].setFixedSize(self.gps_boxes['GPS_coord'].sizeHint())
    def update_box_color(self, box, rate):
        if rate < 1:
            box.setStyleSheet("background-color: red;")
        else:
            box.setStyleSheet("background-color: green;")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("plastique")
    subscriber = ROSSubscriber()
    subscriber.show()
    sys.exit(app.exec_())
