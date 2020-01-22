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
        self.__gantry_pos_m = np.array([0, 0, 0])
        self.__gantry_pos_reached = False
        self.__stop_recording = False
        self.__stop_recording_timer = 0

        rospy.init_node('RF_Loc', anonymous=True)
        self.__pos_sub = rospy.Subscriber("/gantry/current_position", gantry, self.get_update_pos_m)

    def start_RfEar(self, center_freq=434.2e6, freqspan=1e5):
        import rf
        self.__oRf = rf.RfEar(sdr_type='NooElec', center_freq=center_freq, freqspan=freqspan)

        freq6tx = [434.82e6, 434.17e6, 434.02e6, 434.62e6, 434.32e6]
        """
        tx_6pos = [[700, 440],
           [1560,450],
           [2440, 460],
           [2440, 1240],
           [1560, 1235],
           [700, 1230]]
        """

        tx_6pos = np.array([[2359, 1349, 641],
                            [2110, 1349, 641],
                            [1861, 1349, 641],
                            [1612, 1349, 641],
                            [1363, 1349, 641]])

        self.__oRf.set_txparams(freq6tx, tx_6pos)

        # self.__oRf.plot_txrss_live()

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

    def take_measurements_at_pos(self, current_pos):

        # position m --> mm
        pos_x_mm = current_pos[0] * 1000
        pos_y_mm = current_pos[1] * 1000
        pos_z_mm = current_pos[2] * 1000

        # taking measurements
        self.__pos_meas_counter += 1  # counts number of measurements at this position

        time_elapsed = 0

        freq_den_max, pxx_den_max = self.__oRf.get_rss_peaks()

        data_row = np.append(
            [self.__pos_meas_counter, time_elapsed, round(pos_x_mm, 0), round(pos_y_mm, 0), round(pos_z_mm, 0)],
            pxx_den_max)
        self.__data_list.append(data_row)

        return True

    def write_all_data_to_file(self):

        with open(self.__measdata_filename, 'a') as measfile:
            data_mat = np.asarray(self.__data_list)
            for row in data_mat:
                row_string = ''
                for i in range(len(row)):
                    row_string += str(row[i]) + ','
                row_string += '\n'
                measfile.write(row_string)

            measfile.close()
        self.__data_list = []  # reset
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

    def get_meas_counter(self):
        return self.__pos_meas_counter

    def record_measurements(self, max_waiting_time):
        timer_started = False
        is_not_written = False
        while not rospy.is_shutdown():

            if oRf.get_gantry_pos_reached_status() and not timer_started:
                start_timer = t.time()
                timer_started = True

            elif not oRf.get_gantry_pos_reached_status():
                timer_started = False
                start_timer = t.time()

            if oRf.get_gantry_pos_reached_status():
                oRf.take_measurements_at_pos(oRf.get_gantry_pos_m())
                is_not_written = True

            elif is_not_written and not oRf.get_gantry_pos_reached_status():
                oRf.write_all_data_to_file()
                is_not_written = False
                print("wrote to file")

            if (t.time() - start_timer) > max_waiting_time:
                print("end of recording")
                print("max waiting time of " + str(max_waiting_time) + "s  reached")
                print("Number of recorded measurments: " + str(self.get_meas_counter()))
                break


if __name__ == '__main__':
    print("ROS_RF_LOC started")
    oRf = RF_ROS()
    oRf.start_RfEar()
    oRf.init_meas_file('measfile.txt')
    oRf.record_measurements(max_waiting_time=3)
