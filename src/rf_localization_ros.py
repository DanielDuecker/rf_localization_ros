import numpy as np
import rospy
import time as t
from gantry_control_ros.msg import gantry


class RF_ROS():
    def __init__(self):
        self.__oRf = []
        self.__measdata_filename = 'meas.txt'
        self.__pos_meas_counter = 0
        self.__data_list = []
        self.__gantry_pos_m = np.array([0,0,0])
        self.__gantry_pos_reached = False

        rospy.init_node('RF_Loc', anonymous=True)
        self.__pos_sub = rospy.Subscriber("/gantry/current_position", gantry, self.get_update_pos_m)

    def start_RfEar(self, center_freq=434.2e6, freqspan=1e5):
        import rf
        self.__oRf = rf.RfEar(sdr_type='NooElec',center_freq=center_freq, freqspan=freqspan)

        freq6tx = [434.00e6, 434.1e6, 434.30e6, 434.45e6, 434.65e6, 433.90e6]
        """
        tx_6pos = [[700, 440],
           [1560,450],
           [2440, 460],
           [2440, 1240],
           [1560, 1235],
           [700, 1230]]
        """

        tx_6pos = np.array([[520, 430,0],
                   [1540, 430, 0],
                   [2570, 430, 0],
                   [2570, 1230, 0],
                   [1540, 1230, 0],
                   [530, 1230, 0]])

        self.__oRf.set_txparams(freq6tx, tx_6pos)

        return True

    def init_meas_file(self, meas_filename='meas_file.txt'):
        self.__measdata_filename = meas_filename
        with open(self.__measdata_filename, 'w') as measfile:
            measfile.write('Measurement file for trajectory following\n')
            measfile.write('Measurement was taken on ' + t.ctime() + '\n')
            measfile.write('### begin grid settings\n')
            measfile.write('sample size = ' + str(self.__oRf.get_samplesize()) + ' [*1024]\n')
            # measfile.write('avg. meas frequency = ' + str(meas_freq) + ' Hz\n')
            # measfile.write('start_point =' + str(start_wp) + '\n')
            # measfile.write('wp_list =' + str(wp_list) + '\n')
            measfile.write('data format = [meas_counter, time_elapsed, pos_x_mm, pos_y_mm, pos_z_mm], pxx_den_max\n')
            measfile.write('### begin data log\n')

    def take_measurements_at_pos(self, current_pos, b_meas):

        # position m --> mm
        pos_x_mm = current_pos[0]*1000
        pos_y_mm = current_pos[1]*1000
        pos_z_mm = current_pos[2]*1000

        # taking measurements
        self.__pos_meas_counter =+ 1  # counts number of measurements at this position

        time_elapsed = 0

        freq_den_max, pxx_den_max = self.__oRf.get_rss_peaks()

        data_row = np.append([self.__pos_meas_counter, time_elapsed, pos_x_mm, pos_y_mm, pos_z_mm], pxx_den_max)
        self.__data_list.append(data_row)

        return True

    def write_all_data_to_file(self):

        with open(self.__measdata_filename, 'w') as measfile:
            data_mat = np.asarray(self.__data_list)
            for row in data_mat:
                row_string = ''
                for i in range(len(row)):
                    row_string += str(row[i]) + ','
                row_string += '\n'
                measfile.write(row_string)

            measfile.close()

        return True

    def get_update_pos_m(self, gantry_pos_m):
        self.__gantry_pos_m = np.array([gantry_pos_m.pos_gantry.x,
                                        gantry_pos_m.pos_gantry.y,
                                        gantry_pos_m.pos_gantry.z])
        self.__gantry_pos_reached = gantry_pos_m.reached

    def get_gantry_pos_m(self):
        return self.__gantry_pos_m

    def get_gantry_pos_reached_status(self):
        return self.__gantry_pos_reached


def record_measurements():
    while not rospy.is_shutdown():
        oRf.take_measurements_at_pos(oRf.get_gantry_pos_m(), oRf.get_gantry_pos_reached_status())


if __name__ == '__main__':
    print("ROS_RF_LOC started")
    oRf = RF_ROS()
    oRf.start_RfEar()
    oRf.init_meas_file('measfile.txt')



    oRf.write_all_data_to_file()