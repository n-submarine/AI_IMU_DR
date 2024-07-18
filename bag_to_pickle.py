import rosbag
from sensor_msgs.msg import Imu
from novatel_oem7_msgs.msg import INSPVA
import torch
import pickle
import numpy as np
import os
import pymap3d as pm

def lla_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    e, n, u = pm.geodetic2enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
    return e, n, u

def convert_enu_to_xyz(p_gt_enu, v_gt_enu):
    # ENU to XYZ: [North, East, Up] -> [X, -Y, Z]
    p_gt_xyz = np.array([p_gt_enu[:, 1], -p_gt_enu[:, 0], p_gt_enu[:, 2]]).T
    v_gt_xyz = np.array([v_gt_enu[:, 1], -v_gt_enu[:, 0], v_gt_enu[:, 2]]).T
    return p_gt_xyz, v_gt_xyz

def read_imu_data(bag_file, imu_topic, gt_topic):
    bag = rosbag.Bag(bag_file)
    times = []
    ang_gt = []
    p_gt = []
    v_gt = []
    u = []

    data_rate = 125
    one_g = 9.80665
    gsf = np.deg2rad((0.008/65536)/data_rate)
    asf = (0.2/65536)*(one_g/1000)/data_rate

    ref_lat, ref_lon, ref_alt = None, None, None

    for topic, msg, t in bag.read_messages(topics=[imu_topic, gt_topic]):
        timestamp = msg.header.stamp
        current_time = timestamp.secs + timestamp.nsecs * 1e-9
        
        if topic == imu_topic:
            times.append(current_time)
            ax, ay, az = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            lx, ly, lz = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z 
            # ax, ay, az = gsf*ax, gsf*ay, gsf*az
            # lx, ly, lz = asf*lx, asf*ly, asf*lz
            u.append([ax, ay, az, lx, ly, lz])
        elif topic == gt_topic:
            if not ref_lat:
                ref_lat, ref_lon, ref_alt = msg.latitude, msg.longitude, msg.height
            p_e, p_n, p_u = lla_to_enu(msg.latitude, msg.longitude, msg.height, ref_lat, ref_lon, ref_alt)
            ang_gt.append([msg.roll, msg.pitch, np.deg2rad(msg.azimuth)])  # Assuming azimuth is in degrees
            p_gt.append([p_e, p_n, p_u])
            v_gt.append([msg.north_velocity, msg.east_velocity, msg.up_velocity])

    bag.close()

    if len(times) == 0:
        raise ValueError("No IMU messages found in the provided topic.")

    t0 = times[0]
    times = np.array(times) - times[0]
    t = torch.tensor(times, dtype=torch.float32)
    u = torch.tensor(u, dtype=torch.float32)

    ang_gt = torch.tensor(ang_gt, dtype=torch.float32)
    # p_gt = np.array(p_gt, dtype=np.float32) 
    # v_gt = np.array(v_gt, dtype=np.float32)

    # p_gt, v_gt = convert_enu_to_xyz(p_gt, v_gt)
    p_gt = torch.tensor(p_gt, dtype=torch.float32)
    v_gt = torch.tensor(v_gt, dtype=torch.float32)

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

def bags_to_pickles(input_path, output_path, imu_topic, gt_topic):
    os.makedirs(output_path, exist_ok=True)
    for file_name in os.listdir(input_path):
        if file_name.endswith('.bag'):
            file_path = os.path.join(input_path, file_name)
            data = read_imu_data(file_path, imu_topic, gt_topic)
            output_file = os.path.join(output_path, file_name.replace('.bag', '.p'))
            save_to_pickle(data, output_file)

if __name__ == "__main__":
    input_path = '/home/hsy/Bagfiles/ai-imu-dr'
    output_path = './data_mobinha'
    imu_topic = '/imu/data_raw'
    gt_topic = '/novatel/oem7/inspva'

    bags_to_pickles(input_path, output_path, imu_topic, gt_topic)

    print("All bag files processed and saved as pickle files.")
