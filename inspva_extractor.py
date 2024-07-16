import rosbag
import pymap3d as pm
import torch
import pickle
import numpy as np

# Variables to store reference latitude, longitude, and altitude
init_flag = True
ref_lat = 0
ref_lon = 0
ref_alt = 0

def lla_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    e, n, u = pm.geodetic2enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
    return e, n, u

def extract_inspva_data(bag_file, topic):
    global init_flag, ref_lat, ref_lon, ref_alt
    lla_data = []
    v_data = []
    ang_data = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic]):
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.height
            north_vel = msg.north_velocity
            east_vel = msg.east_velocity
            up_vel = msg.up_velocity
            roll = msg.roll
            pitch = msg.pitch
            yaw = np.deg2rad(msg.azimuth)

            if init_flag:
                ref_lat = lat
                ref_lon = lon
                ref_alt = alt
                init_flag = False

            enu = lla_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
            lla_data.append(enu)
            v_data.append([north_vel, east_vel, up_vel])
            ang_data.append([roll, pitch, yaw])

    return lla_data, v_data, ang_data

bag_file = '/home/hsy/Bagfiles/underground_output.bag'
topic = '/novatel/oem7/inspva'
pickle_file = '/home/hsy/ai-imu-dr/data_mobinha/underground_output.p'

p_gt_data, v_gt_data, ang_gt_data = extract_inspva_data(bag_file, topic)

p_gt = torch.tensor(p_gt_data, dtype=torch.float32)
v_gt = torch.tensor(v_gt_data, dtype=torch.float32)
ang_gt = torch.tensor(ang_gt_data, dtype=torch.float32)

with open(pickle_file, 'rb') as f:
    data = pickle.load(f)

data['p_gt'] = p_gt
data['v_gt'] = v_gt
data['ang_gt'] = ang_gt

with open(pickle_file, 'wb') as f:
    pickle.dump(data, f)

print(f"Updated p_gt, v_gt, and ang_gt in {pickle_file}")
