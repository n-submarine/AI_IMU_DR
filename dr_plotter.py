import can
import cantools
import threading
import time
import matplotlib.pyplot as plt
import time
from novatel_oem7_msgs.msg import INSPVA
import rospy
import pymap3d
import signal
import sys
import math

class IONIQ:
    def __init__(self):
        rospy.init_node("DR_test")

        self.bus = can.ThreadSafeBus(
            interface='socketcan', channel='can0', bitrate=500000)
        self.db = cantools.database.load_file('/home/inha/Documents/catkin_ws/src/mobinha/selfdrive/car/dbc/ioniq/can.dbc')
        
        # sensor data subscribe
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.novatel_cb)
        self.x, self.y, self.z = 0, 0, 0

        # dr data subscribe
        rospy.Subscriber('notDefined', notDefined, self.deadreckoning_cb)
        self.dr_x, self.dr_y, self.dr_z = 0, 0, 0

        # Plot 관련 초기화
        self.plot_lock = threading.Lock()
        self.x_prev, self.y_prev, self.z_prev = -999, -999, -999
        self.dr_x_prev, self.dr_y_prev, self.dr_z_prev = -999, -999, -999
        self.true_position = []
        self.true_position_history = []
        self.dr_position = []
        self.dr_position_history = []


    def update_values(self):
        with self.plot_lock:
            if math.sqrt((self.x-self.x_prev)**2 + (self.y-self.y_prev)**2 + (self.z-self.z_prev)**2) > 0.1:
                self.true_position_history.append((self.x, self.y, self.z))
                self.x_prev, self.y_prev, self.z_prev = self.x, self.y, self.z

            if math.sqrt((self.dr_x-self.dr_x_prev)**2 + (self.dr_y-self.dr_y_prev)**2 + (self.dr_z-self.dr_z_prev)**2) > 0.1:
                self.dr_position_history.append((self.dr_x, self.dr_y, self.dr_z))
                self.dr_x_prev, self.dr_y_prev, self.dr_z_prev = self.dr_x, self.dr_y, self.dr_z
            
            # for arr in [self.true_position, self.dr_position]:
            #     if len(arr) > 100:
            #         arr.pop(0)


    def plot_true_position(self):
        _3d = False
        if _3d:
            self.plot_data(self.true_position_history, 'r', _3d=True)
        else:
            self.plot_data(self.true_position_history, 'r', _3d=False)
            return
            plt.ion()
            fig, ax = plt.subplots()
            position_point, = ax.plot([], [], 'bo', label="True Position")
            position_line, = ax.plot([], [], 'b-', label="True Path")

            plt.legend(loc='upper left')
            plt.grid(True)
            plt.axis('equal')

            while not rospy.is_shutdown():
                with self.plot_lock:
                    position_point.set_xdata(self.x)
                    position_point.set_ydata(self.y)
                    position_line.set_xdata([pos[0] for pos in self.true_position_history])
                    position_line.set_ydata([pos[1] for pos in self.true_position_history])
                    
                    ax.relim()
                    ax.autoscale_view()

                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()

                plt.pause(0.01)
            
            return
        
    def plot_data(self, data, color, _3d):
        plt.ion()
        fig = plt.figure()
        if _3d:
            ax = fig.add_subplot(111, projection='3d')
        else:
            ax = fig.add_subplot(111)


        while not rospy.is_shutdown():
            with self.plot_lock:
                x = [t[0] for t in data]
                y = [t[1] for t in data]
                if _3d:
                    z = [t[2] for t in data]

                ax.clear()  # Clear previous data
                
                if _3d:
                    ax.scatter(x, y, z, c=color)
                    ax.scatter(x[-1], y[-1], z[-1], s=20, c=color)
                else:
                    ax.scatter(x, y, c=color)
                    ax.scatter(x[-1], y[-1], s=20, c=color)

                

                ax.set_xlabel('X Label')
                ax.set_ylabel('Y Label')
                if _3d:
                    ax.set_zlabel('Z Label')
                
                plt.grid()

                fig.canvas.draw_idle()
                fig.canvas.flush_events()

                plt.pause(0.01)


    def novatel_cb(self, msg):
        if base_flag:
            self.base_lat, self.base_lon, self.base_hgt = msg.latitude, msg.longitude, msg.height
            base_flag = True

        self.x, self.y, self.z = pymap3d.geodetic2enu(
            msg.latitude, msg.longitude, msg.height, self.base_lat, self.base_lon, self.base_hgt)
        
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = 90 - msg.azimuth + 360 if (-270 <= 90 - msg.azimuth <= -180) else 90 - msg.azimuth

    def deadreckoning_cb(self, msg):
        notDefined = msg.data
        '''
        self.dr_x~z update
        '''

    
    def plot_true_position(self):
        _3d = False
        if _3d:
            self.plot_data(self.dr_position_history, 'b', _3d=True)
        else:
            self.plot_data(self.dr_position_history, 'b', _3d=False)
            return
            plt.ion()
            fig, ax = plt.subplots()
            position_point, = ax.plot([], [], 'bo', label="True Position")
            position_line, = ax.plot([], [], 'b-', label="True Path")

            plt.legend(loc='upper left')
            plt.grid(True)
            plt.axis('equal')

            while not rospy.is_shutdown():
                with self.plot_lock:
                    position_point.set_xdata(self.x)
                    position_point.set_ydata(self.y)
                    position_line.set_xdata([pos[0] for pos in self.true_position_history])
                    position_line.set_ydata([pos[1] for pos in self.true_position_history])
                    
                    ax.relim()
                    ax.autoscale_view()

                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()

                plt.pause(0.01)
            
            return
    

    

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Exiting gracefully...')
    rospy.signal_shutdown('Exiting')
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    IONIQ = IONIQ()
    t1 = threading.Thread(target=IONIQ.update_values)
    t2 = threading.Thread(target=IONIQ.plot_true_position)
    t3 = threading.Thread(target=IONIQ.plot_dr_position)

    for el in [t1, t2, t3]:
        el.start()

    t1.join()
