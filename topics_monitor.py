import sys
import rospy
from std_msgs.msg import String
from PySide2 import QtWidgets, QtGui, QtCore
from sensor_msgs.msg import Image, PointCloud2, Joy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
# from rostopic import ROSTopicHz
import rostopic
import numpy as np
import yaml


class ROSSubscriber(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_ros()

        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self.update_all)
        self.update_rate = 1.0
        self.update_timer.start(self.update_rate * 1000)  # Update rates every 1 second

    def init_ros(self):
        rospy.init_node("pyside_ros_subscriber")
        # self.camera_prefixes = ['/camera_topic_1']

        self.camera_prefixes = ['/multisense/left/image_rect_color']

        self.lidar_prefixes = ['/velodyne_1/velodyne_points', '/velodyne_2/velodyne_points', '/livox/lidar']
        # self.lidar_prefixes = ['/lidar_topic_1', '/lidar_topic_2']

        # self.other_list = {'/multisense/left/image_rect_color':8,'/velodyne_1/velodyne_points':8,'/livox/lidar':8}
        with open('topic_rates.yaml', 'r') as yaml_file:
            self.other_list = yaml.safe_load(yaml_file)

        self.gsubscribers = {}
        self.other_msg_stats = {}
        # self.bad_list = np.zeros(len(self.other_list))
        # self.inv_dict = {}
        self.bad_list = []
        self.bad_ts_list = []

        for i,topic in enumerate(self.other_list):
            # message_type = rospy.get_param(topic + '/type')
            # topic += '/header'
            # print(topic)
            TopicType, topic_str, _ = rostopic.get_topic_class(topic)
            self.gsubscribers[topic] = rospy.Subscriber(topic, TopicType, self.generic_callback, callback_args=topic)
            self.other_msg_stats[topic] = np.zeros(10)


        self.camera_rates = {prefix: 0 for prefix in self.camera_prefixes}
        self.lidar_rates = {prefix: 0 for prefix in self.lidar_prefixes}
        self.camera_boxes = {}
        self.lidar_boxes = {}
        self.gps_boxes = {}

        self.misc_boxes = {}

        self.subscribe_to_topics()

    def init_ui(self):
        self.setWindowTitle("ROS Subscriber with PySide")
        self.resize(400, 300)

        self.auto_label = QtWidgets.QLabel("Autonomy Status:")
        self.auto_layout = QtWidgets.QVBoxLayout()
        self.auto_status = ""

        self.camera_label = QtWidgets.QLabel("Camera Topics:")
        self.camera_layout = QtWidgets.QVBoxLayout()

        self.lidar_label = QtWidgets.QLabel("Lidar Topics:")
        self.lidar_layout = QtWidgets.QVBoxLayout()

        self.gps_label = QtWidgets.QLabel("GPS Topics:")
        self.gps_layout = QtWidgets.QVBoxLayout()


        self.other_label = QtWidgets.QLabel("Miscellaneous:")
        self.other_layout = QtWidgets.QVBoxLayout()

        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(self.auto_label)
        main_layout.addLayout(self.auto_layout)
        main_layout.addWidget(self.camera_label)
        main_layout.addLayout(self.camera_layout)
        main_layout.addWidget(self.lidar_label)
        main_layout.addLayout(self.lidar_layout)
        main_layout.addWidget(self.gps_label)
        main_layout.addLayout(self.gps_layout)


        main_layout.addWidget(self.other_label)
        main_layout.addLayout(self.other_layout)

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

        rospy.Subscriber('/mux/joy', Joy, self.auto_callback)

        self.gps_rates = {'/odometry/filtered_odom' : 0}
        self.gps_boxes['/odometry/filtered_odom'] = self.create_rate_box()
        self.gps_layout.addWidget(self.gps_boxes['/odometry/filtered_odom'])
        self.gps_boxes['GPS_coord'] = self.create_box("GPS XY Good")
        self.gps_layout.addWidget(self.gps_boxes['GPS_coord'])
        self.gps_boxes['Speed'] = self.create_box("Speed")
        self.gps_layout.addWidget(self.gps_boxes['Speed'])
        self.gps_xy = [0,0]
        self.speed = 0


        self.misc_boxes['rates'] = self.create_box("")
        self.other_layout.addWidget(self.misc_boxes['rates'])

        self.misc_boxes['ts'] = self.create_box("")
        self.other_layout.addWidget(self.misc_boxes['ts'])

        self.auto_box = self.create_box("")
        self.auto_layout.addWidget(self.auto_box)


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

        secs = msg.header.stamp.to_sec()
        if np.abs(secs - rospy.Time.now().to_sec()) > 2:
            # print(secs, rospy.Time.now().to_sec())
            if topic not in self.bad_ts_list:
                self.bad_ts_list.append(topic)

    @QtCore.Slot(PointCloud2)
    def lidar_rate_callback(self, msg, topic):
        # topic = rospy.get_caller_id()
        self.lidar_rates[topic] += 1


        secs = msg.header.stamp.to_sec()
        if np.abs(secs - rospy.Time.now().to_sec()) > 2:
            # print(secs, rospy.Time.now().to_sec())
            if topic not in self.bad_ts_list:
                self.bad_ts_list.append(topic)

    @QtCore.Slot(PointCloud2)
    def generic_callback(self, msg, topic):
        # topic = rospy.get_caller_id()
        # self.lidar_rates[topic] += 1
        # print(topic)
        # print(msg.header.stamp)
        secs = msg.header.stamp.to_sec()
        buff = self.other_msg_stats[topic]
        buff[1:] = buff[:-1]
        buff[0] = secs
        dt = 1.0/((buff[:-1]-buff[1:]).mean())
        # print(dt, topic)
        if dt < self.other_list[topic]:
            if topic not in self.bad_list:
                self.bad_list.append(topic)

        if np.abs(secs - rospy.Time.now().to_sec()) > 2:
            # print(secs, rospy.Time.now().to_sec())
            if topic not in self.bad_ts_list:
                self.bad_ts_list.append(topic)


    @QtCore.Slot(Odometry)
    def gps_callback(self, msg):
        # topic = rospy.get_caller_id()
        self.gps_rates['/odometry/filtered_odom'] += 1
        self.gps_xy = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.speed = float(np.linalg.norm([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]))


        secs = msg.header.stamp.to_sec()
        if np.abs(secs - rospy.Time.now().to_sec()) > 2:
            # print(secs, rospy.Time.now().to_sec())
            if '/odometry/filtered_odom' not in self.bad_ts_list:
                self.bad_ts_list.append('/odometry/filtered_odom')

    @QtCore.Slot(Joy)
    def auto_callback(self, msg):
        # topic = rospy.get_caller_id()
        self.auto_status = msg.header.frame_id

    def update_all(self):
        self.update_rates()
        self.update_other()
        self.update_other_rates()

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

            # hz = rate/self.update_rate
            hz = rate

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

        self.gps_boxes['Speed'].label.setText(str(self.speed) + " m/s")
        self.gps_boxes['Speed'].setStyleSheet("background-color: green;")
        self.gps_boxes['Speed'].setFixedSize(self.gps_boxes['Speed'].sizeHint())

        self.auto_box.label.setText(self.auto_status)
        if self.auto_status == 'teleop':
            self.auto_box.setStyleSheet("background-color: red;")
        else:
            self.auto_box.setStyleSheet("background-color: green;")
        self.auto_box.setFixedSize(self.auto_box.sizeHint())


    def update_other_rates(self):
        if len(self.bad_list) == 0:
            self.misc_boxes['rates'].label.setText('Good')
            self.misc_boxes['rates'].setStyleSheet("background-color: green;")
            self.misc_boxes['rates'].setFixedSize(self.misc_boxes['rates'].sizeHint())
        else:
            badstr = ''
            for bad in self.bad_list:
                badstr += bad + '\n'
            self.bad_list = []
            self.misc_boxes['rates'].label.setText(badstr)
            self.misc_boxes['rates'].setStyleSheet("background-color: red;")
            self.misc_boxes['rates'].setFixedSize(self.misc_boxes['rates'].sizeHint())

        if len(self.bad_ts_list) == 0:
            self.misc_boxes['rates'].label.setText('Good')
            self.misc_boxes['rates'].setStyleSheet("background-color: green;")
            self.misc_boxes['rates'].setFixedSize(self.misc_boxes['rates'].sizeHint())
        else:
            badstr = ''
            for bad in self.bad_ts_list:
                badstr += bad + '\n'
            self.bad_ts_list = []
            self.misc_boxes['ts'].label.setText(badstr)
            self.misc_boxes['ts'].setStyleSheet("background-color: red;")
            self.misc_boxes['ts'].setFixedSize(self.misc_boxes['ts'].sizeHint())

    def update_box_color(self, box, rate, desired=8):
        if rate < desired:

            box.setStyleSheet("background-color: red;")
        else:
            box.setStyleSheet("background-color: green;")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("plastique")
    subscriber = ROSSubscriber()
    subscriber.show()
    sys.exit(app.exec_())
