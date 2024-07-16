import rosbag
import pymap3d as pm
import matplotlib.pyplot as plt

'''
base lla : first data
'''

init_flag = True
ref_lat = 0
ref_lon = 0
ref_alt = 0

def lla_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    e, n, u = pm.geodetic2enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
    return e, n, u

def lla_to_ned(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    e, n, u = lla_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
    return n, e, -u

def extract_lla_from_bag(bag_file, topic):
    global init_flag, ref_lat, ref_lon, ref_alt
    lla_data = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic]):
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.height
            if init_flag:
                ref_lat = lat
                ref_lon = lon
                ref_alt = alt
                init_flag = False
            lla_data.append((lat, lon, alt))

    return lla_data

def plot_3d_data(data, title, _3d):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x = [t[0] for t in data]
    y = [t[1] for t in data]
    if _3d:
        z = [t[2] for t in data]

    if _3d:
        ax.scatter(x, y, z)
    else:
        ax.scatter(x, y)


    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    if _3d:
        ax.set_zlabel('Z Label')
    
    plt.title(title)
    plt.grid()
    plt.axis('auto')
    plt.show()

def main():
    bag_file = '/home/hsy/Bagfiles/songdo_230923.bag'
    topic = '/novatel/oem7/inspva'

    lla_data = extract_lla_from_bag(bag_file, topic)

    enu_coords = []
    ned_coords = []

    for lat, lon, alt in lla_data:
        enu = lla_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt)
        ned = lla_to_ned(lat, lon, alt, ref_lat, ref_lon, ref_alt)
        enu_coords.append(enu)
        ned_coords.append(ned)

    print("ENU :", enu_coords)
    # print("NED :", ned_coords)

    plot_3d_data(enu_coords, title="ENU", _3d=False)
    # plot_3d_data(ned_coords, title="NED", _3d=True)

if __name__ == "__main__":
    main()
