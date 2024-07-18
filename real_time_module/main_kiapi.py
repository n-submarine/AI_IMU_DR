import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import torch
import numpy as np
from utils_torch_filter import TORCHIEKF
from utils_numpy_filter import NUMPYIEKF as IEKF
from main_kitti import CustomParameters
from utils import prepare_data
import tf

class RealTimeIMU:
    def __init__(self):
        self.iekf = IEKF()
        self.torch_iekf = TORCHIEKF()
        self.filter_parameters = CustomParameters()
        
        self.iekf.filter_parameters = self.filter_parameters
        self.iekf.set_param_attr()  # Class에서 정의된 모든 params에 필터가 접근할 수 있게 함
        self.torch_iekf.filter_parameters = self.filter_parameters
        self.torch_iekf.set_param_attr()

        self.u = []
        self.t = []
        self.initialized = False
        self.start_time = None

        rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)
        # rospy.Subscriber('/novatel/oem7/inspva', Odometry, self.gt_callback)

        self.pub = rospy.Publisher('/ai_imu_dr/odometry', Odometry, queue_size=10)

    def imu_callback(self, msg):
        if not self.initialized:
            self.start_time = rospy.Time.now()
            self.initialized = True

        timestamp = msg.header.stamp
        current_time = timestamp.secs + timestamp.nsecs * 1e-9
        current_time = (rospy.Time.now() - self.start_time).to_sec()

        self.u.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                       msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.t.append(current_time)

        if len(self.u) >= 50:  # Process frequency
            self.process_data()

    # def gt_callback(self, msg):
    #     pass

    def process_data(self):
        u_np = np.array(self.u)
        t_np = np.array(self.t)

        t_torch = torch.tensor(t_np, dtype=torch.float32)
        u_torch = torch.tensor(u_np, dtype=torch.float32)

        measurements_covs = self.torch_iekf.forward_nets(u_torch)
        measurements_covs = measurements_covs.detach().numpy()

        Rot, v, p, b_omega, b_acc, Rot_c_i, t_c_i = self.iekf.run(t_np, u_np, measurements_covs, None, None, None)

        self.publish_odometry(p[-1], v[-1], Rot[-1])

        # Clear the buffer
        self.u = []
        self.t = []

    def publish_odometry(self, position, velocity, orientation_matrix):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]

        # Convert orientation matrix to quaternion
        quaternion = tf.transformations.quaternion_from_matrix(orientation_matrix)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # Set the velocity
        odom.twist.twist.linear.x = velocity[0]
        odom.twist.twist.linear.y = velocity[1]
        odom.twist.twist.linear.z = velocity[2]

        self.pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('ai_imu_dr')
    real_time_imu = RealTimeIMU()
    rospy.spin()
