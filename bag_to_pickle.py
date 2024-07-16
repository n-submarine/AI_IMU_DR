import rosbag
from sensor_msgs.msg import Imu
import torch
import pickle
import numpy as np
import os

def read_imu_data(bag_file, topic):
    bag = rosbag.Bag(bag_file)
    times = []
    ang_gt = []
    p_gt = []  # Placeholder for position ground truth if available
    v_gt = []  # Placeholder for velocity ground truth if available
    u = []

    for topic, msg, _ in bag.read_messages(topics=[topic]):
        # 메시지 타입을 문자열로 확인
        if msg._type == 'sensor_msgs/Imu':
            timestamp = msg.header.stamp
            times.append(timestamp.secs + timestamp.nsecs * 1e-9)
            ang_gt.append([0, 0, 0])
            u.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    bag.close()

    if len(times) == 0:
        raise ValueError("No IMU messages found in the provided topic.")

    times = np.array(times) - times[0]
    t0 = times[0]
    ang_gt = torch.tensor(ang_gt, dtype=torch.float32)
    t = torch.tensor(times, dtype=torch.float32)
    u = torch.tensor(u, dtype=torch.float32)
    p_gt = torch.zeros_like(ang_gt)  # Placeholder if position ground truth is not available
    v_gt = torch.zeros_like(ang_gt)  # Placeholder if velocity ground truth is not available

    return {
        'name': os.path.basename(bag_file),
        'ang_gt': ang_gt,
        'p_gt': p_gt,
        't0': t0,
        't': t,
        'v_gt': v_gt,
        'u': u
    }

def save_to_pickle(data, output_file):
    with open(output_file, 'wb') as f:
        pickle.dump(data, f)

if __name__ == "__main__":
    bag_file = '/home/hsy/Bagfiles/underground_output.bag'  
    topic = '/gps/imu'  
    output_file = './data_mobinha/underground_output.p' 

    data = read_imu_data(bag_file, topic)
    save_to_pickle(data, output_file)

    print("save complete")
