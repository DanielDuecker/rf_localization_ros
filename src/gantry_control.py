import numpy as np
import matplotlib.pyplot as plt
import time as t
#import serial_control as sc
#import hippocampus_toolbox as hc_tools
import rf_tools
#import estimator


class GantryControl(object):
    def __init__(self, gantry_dimensions=[0, 3000, 0, 1580], use_gui=False):  # [x0 ,x1, y0, y1]
        self.__dimensions = gantry_dimensions
        self.__gantry_pos = [0, 0]  # initial position after start
        self.__target_wp_mm = []
        self.__oRf = []
        self.__starttime = []
    def follow_wp_and_take_measurements(self, start_wp=[1000, 1000], sample_size=32):

        self.start_RfEar()
        self.__oRf.set_samplesize(sample_size)
        sample_size = self.__oRf.get_samplesize()

        wplist_filename = hc_tools.select_file()
        print(wplist_filename)
        wp_append_list = []
        with open(wplist_filename, 'r') as wpfile:
            for i, line in enumerate(wpfile):
                print('line = ' + line)
                print(line.split(','))
                temp_list = line.split(',')
                wp_append_list.append(map(float, temp_list[0:-1]))

        print(str(np.asarray(wp_append_list)))
        wp_list = np.asarray(wp_append_list)

        measdata_filename = hc_tools.save_as_dialog()
        print(measdata_filename)

        num_wp = len(wp_list)
        print('Number of way points: ' + str(num_wp))
        start_time = t.time()

        self.set_target_wp(start_wp)
        self.start_moving_gantry_to_target()
        print('Moving to start position = ' + str(start_wp))
        while not self.confirm_arrived_at_target_wp():
            t.sleep(0.2)
        print('Arrived at start point')

        t.sleep(0.5)
        print('Start following way point sequence')

        data_list = []
        meas_counter = 0
        time_elapsed = 0.0
        for wp in wp_list:
            # go to wp
            self.set_target_wp(wp)
            self.start_moving_gantry_to_target()
            not_arrived_at_wp = True
            print('Moving to wp = ' + str(wp))

            # taking measurements
            while not_arrived_at_wp:
                meas_counter += 1
                time_elapsed = t.time() - start_time
                pos_x_mm, pos_y_mm = self.get_gantry_pos_xy_mm()
                freq_den_max, pxx_den_max = self.__oRf.get_rss_peaks()

                data_row = np.append([meas_counter, time_elapsed, pos_x_mm, pos_y_mm], pxx_den_max)
                data_list.append(data_row)

                if self.confirm_arrived_at_target_wp():
                    not_arrived_at_wp = False

            meas_freq = meas_counter / time_elapsed
            print('Logging with avg. ' + str(meas_freq) + ' Hz')

        with open(measdata_filename, 'w') as measfile:
            measfile.write('Measurement file for trajectory following\n')
            measfile.write('Measurement was taken on ' + t.ctime() + '\n')
            measfile.write('### begin grid settings\n')
            measfile.write('sample size = ' + str(sample_size) + ' [*1024]\n')
            measfile.write('avg. meas frequency = ' + str(meas_freq) + ' Hz\n')
            measfile.write('start_point =' + str(start_wp) + '\n')
            measfile.write('wp_list =' + str(wp_list) + '\n')
            measfile.write('data format = [meas_counter, time_elapsed, pos_x_mm, pos_y_mm], pxx_den_max\n')
            measfile.write('### begin data log\n')
            data_mat = np.asarray(data_list)
            for row in data_mat:
                row_string = ''
                for i in range(len(row)):
                    row_string += str(row[i]) + ','
                row_string += '\n'
                measfile.write(row_string)

            measfile.close()

        return True

    def follow_wp_path_opt_take_measurements(self, num_plot_points=250, model_type='log', b_take_meas=False, b_log_data=False ,tol=10 ,start_wp=[1000, 1000], sample_size=32):
        """
        :param b_take_meas:
        :param start_wp:
        :param sample_size:
        :return:
        """
        if b_take_meas is True:  # dont start SDR if not RSS measurements will be taken
            self.start_RfEar()
            self.__oRf.set_samplesize(sample_size)
            sample_size = self.__oRf.get_samplesize()

        wplist_filename = hc_tools.select_file()
        print(wplist_filename)
        wp_append_list = []
        with open(wplist_filename, 'r') as wpfile:
            for i, line in enumerate(wpfile):
                #print('line = ' + line)
                #print(line.split(','))
                temp_list = line.split(',')
                wp_append_list.append(map(float, temp_list[0:-1]))

#        print(str(np.asarray(wp_append_list)))
        wp_list = np.asarray(wp_append_list)
        if b_log_data:
            measdata_filename = hc_tools.save_as_dialog()
 #       print(measdata_filename)

        num_wp = len(wp_list)
        print('Number of way points: ' + str(num_wp))
        start_time = t.time()

        data_list = []
        meas_counter = 0
        time_elapsed = 0.0
        tolx_mm = tol  # mm
        toly_mm = tol  # mm

        b_ekf = True
        if b_ekf is True:
            # init EKF
            EKF = estimator.ExtendedKalmanFilter(model_type)
            import estimator_plot_tools
            EKF_plotter = estimator_plot_tools.EKF_Plot(EKF.get_tx_pos(), model_type)

        # follow wp sequence
        for wp in wp_list:
            print('wp in list = ' + str(wp))
            # go to wp
            self.set_target_wp(wp)
            self.start_moving_gantry_to_target()
            not_arrived_at_wp = True
            print('Moving to wp = ' + str(wp))

            # following sequence
            while not_arrived_at_wp:
                meas_counter += 1
                time_elapsed = t.time() - start_time
                pos_x_mm, pos_y_mm = self.get_gantry_pos_xy_mm()

                if b_ekf is True:
                    EKF.ekf_prediction()
                    EKF.ekf_update(-85)
                    # EKF.check_valid_position_estimate()
                    # print(EKF.get_x_est())
                    EKF_plotter.add_x_est_to_plot(EKF.get_x_est())
                    EKF_plotter.update_meas_circles(EKF.get_z_meas(), EKF.get_tx_alpha(), EKF.get_tx_gamma(), True, EKF.get_y_est())
                    EKF_plotter.plot_gantry_pos([pos_x_mm, pos_y_mm])
                    EKF_plotter.plot_ekf_pos_live(True, num_plot_points)
                    # EKF_plotter.add_p_cov_to_plot(EKF.get_p_mat())
                    #EKF_plotter.plot_p_cov(num_plot_points)


                if b_take_meas is True:
                    # taking measurements
                    freq_den_max, pxx_den_max = self.__oRf.get_rss_peaks()
                    data_row = np.append([meas_counter, time_elapsed, pos_x_mm, pos_y_mm], pxx_den_max)

                else:
                    data_row = [meas_counter, time_elapsed, pos_x_mm, pos_y_mm]

                # add new data to list
                data_list.append(data_row)

                # arrived at wp? -> go to next
                if self.confirm_arrived_at_target_wp(tolx_mm, toly_mm):
                    not_arrived_at_wp = False

            meas_freq = meas_counter / time_elapsed
            print('Logging with avg. ' + str(meas_freq) + ' Hz')

        if b_log_data:
            with open(measdata_filename, 'w') as measfile:
                measfile.write('Measurement file for trajectory following\n')
                measfile.write('Measurement was taken on ' + t.ctime() + '\n')
                measfile.write('### begin grid settings\n')
                if b_take_meas is True:
                    measfile.write('sample size = ' + str(sample_size) + ' [*1024]\n')
                    measfile.write('avg. meas frequency = ' + str(meas_freq) + ' Hz\n')
                measfile.write('start_point =' + str(start_wp) + '\n')
                measfile.write('wp_list =' + str(wp_list) + '\n')
                if b_take_meas is True:
                    measfile.write('data format = [meas_counter, time_elapsed, pos_x_mm, pos_y_mm], pxx_den_max\n')
                else:
                    measfile.write('data format = [meas_counter, time_elapsed, pos_x_mm, pos_y_mm]\n')

                measfile.write('### begin data log\n')

                data_mat = np.asarray(data_list)
                for row in data_mat:
                    row_string = ''
                    for i in range(len(row)):
                        row_string += str(row[i]) + ','
                    row_string += '\n'
                    measfile.write(row_string)

                measfile.close()

        return True

    def position_hold_measurements(self, xy_pos_mm, meas_time, filename, set_sample_size=256):
        """

        :param xy_pos_mm:
        :param meas_time:
        :param filename:
        :param set_sample_size:
        :return:
        """
        self.set_target_wp(xy_pos_mm)
        self.start_moving_gantry_to_target()
        while not self.confirm_arrived_at_target_wp():
            print('Moving to position = ' + str(xy_pos_mm))
            t.sleep(0.2)

        self.__oCal.set_size(set_sample_size)
        sample_size = self.__oCal.get_size()

        print('Sampling with sample size ' + str(sample_size) + ' [*1024]\n')

        start_time = t.time()
        print('measuring for ' + str(meas_time) + 's ...\n')
        time_elapsed = 0.0
        meas_counter = 0.0
        data_list = []

        # taking measurements
        while time_elapsed < meas_time:
            pos_x_mm, pos_y_mm = self.get_gantry_pos_xy_mm()
            freq_den_max, pxx_den_max = self.__oCal.get_rss_peaks_at_freqtx()
            time_elapsed = t.time() - start_time
            meas_counter += 1.0

            data_row = np.append([meas_counter, time_elapsed, pos_x_mm, pos_y_mm], pxx_den_max)
            data_list.append(data_row)

        meas_freq = meas_counter / time_elapsed
        print('Logging with avg. ' + str(meas_freq) + ' Hz')

        # save data to file
        with open(filename, 'w') as measfile:
            measfile.write('Measurement file for trajectory following\n')
            measfile.write('Measurement was taken on ' + t.ctime() + '\n')
            measfile.write('### begin grid settings\n')
            measfile.write('measurements at position = ' + str(xy_pos_mm) + '\n')
            measfile.write('Meas_time = ' + str(meas_time) + '\n')
            measfile.write('sample size = ' + str(sample_size) + ' [*1024]\n')
            measfile.write('avg. meas frequency = ' + str(meas_freq) + ' Hz\n')
            measfile.write('data format = [meas_counter, time_elapsed, pos_x_mm, pos_y_mm], pxx_den_max\n')
            measfile.write('### begin data log\n')
            data_mat = np.asarray(data_list)
            for row in data_mat:
                row_string = ''
                for i in range(len(row)):
                    row_string += str(row[i]) + ','
                row_string += '\n'
                measfile.write(row_string)

            measfile.close()

        return True

    def start_field_measurement_file_select(self):
        #read data from waypoint file

        wplist_filename = hc_tools.select_file()
        print(wplist_filename)

        measdata_filename = hc_tools.save_as_dialog()
        print(measdata_filename)

        self.start_RfEar()
        freqtx, numtx, tx_abs_pos = self.__oRf.get_txparams()
        print(freqtx)
        print(numtx)
        print(tx_abs_pos)

        self.process_measurement_sequence(wplist_filename, measdata_filename, numtx, tx_abs_pos, freqtx)

    def process_measurement_sequence(self, wplist_filename, measdata_filename, numtx, tx_abs_pos, freqtx):
        """
        :return:
        """
        print('Process Measurement Sequence started')
        # read data from waypoint file
        #wplist_filename = hc_tools.select_file()

        """
        with open(wplist_filename, 'r') as wpfile:

            for i, line in enumerate(wplist_filename):
                print('i= ' + str(i) + ' line:' + line)
                if line == '###':
                if i >= 3:  # ignore header (first 3 lines)

            wp_data_list = [map(float, line.split(',')) for line in wpfile]
            wp_data_mat = np.asarray(wp_data_list)
            wpfile.close()
        """
        #wp_data_mat, x0, xn, grid_dxdy, timemeas = rf_tools.read_data_from_wp_list_file(wplist_filename)
        with open(wplist_filename, 'r') as wpfile:
            load_description = True
            load_grid_settings = False
            load_wplist = False
            wp_append_list = []
            for i, line in enumerate(wpfile):

                if line == '### begin grid settings\n':
                    print('griddata found')
                    load_description = False
                    load_grid_settings = True
                    load_wplist = False
                    continue
                elif line == '### begin wp_list\n':
                    load_description = False
                    load_grid_settings = False
                    load_wplist = True
                    print('### found')
                    continue
                if load_description:
                    print('file description')
                    print(line)

                if load_grid_settings and not load_wplist:
                    grid_settings = map(float, line.split(','))
                    x0 = [grid_settings[0], grid_settings[1]]
                    xn = [grid_settings[2], grid_settings[3]]
                    grid_dxdy = [grid_settings[4], grid_settings[5]]
                    timemeas = grid_settings[6]

                    data_shape = [xn[0] / grid_dxdy[0] + 1, xn[1] / grid_dxdy[1] + 1]

                if load_wplist and not load_grid_settings:
                    # print('read wplist')
                    wp_append_list.append(map(float, line.split(',')))

            print(str(np.asarray(wp_append_list)))
            wp_data_mat = np.asarray(wp_append_list)

            wpfile.close()

        #measdata_filename = hc_tools.save_as_dialog('Save measurement data as...')
        with open(measdata_filename, 'w') as measfile:

            # write header to measurement file
            file_description = 'Measurement file\n' + 'Measurement was taken on ' + t.ctime() + '\n'

            txdata = str(numtx) + ', '
            for itx in range(numtx):
                txpos = tx_abs_pos[itx]
                txdata += str(txpos[0]) + ', ' + str(txpos[1]) + ', '
            for itx in range(numtx):
                txdata += str(freqtx[itx]) + ', '

            print('txdata = ' + txdata)

            measfile.write(file_description)
            measfile.write('### begin grid settings\n')
            measfile.write(str(x0[0]) + ', ' + str(x0[1]) + ', ' +
                           str(xn[0]) + ', ' + str(xn[1]) + ', ' +
                           str(grid_dxdy[0]) + ', ' + str(grid_dxdy[1]) + ', ' +
                           str(timemeas) + ', ' + txdata +
                           '\n')
            measfile.write('### begin measurement data\n')

            # setup plot
            plt.ion()
            plt.plot(wp_data_mat[:, 1], wp_data_mat[:, 2], 'b.-')
            plt.xlabel('Distance in mm (belt-drive)')
            plt.ylabel('Distance in mm (spindle-drive)')
            plt.xlim(-100, 3100)
            plt.ylim(-100, 1800)
            #plt.xlim(x0[0]-10, xn[0]+100)
            #plt.ylim(x0[1]-10, xn[1]+100)
            plt.grid()
            for i in range(len(tx_abs_pos)):
                txpos_single = tx_abs_pos[i]
                plt.plot(txpos_single[0], txpos_single[1], 'ro')
            plt.show()

            totnumofwp = np.shape(wp_data_mat)

            totnumofwp = totnumofwp[0]
            print ('Number of waypoints = ' + str(totnumofwp) + '\n')

            # loop over all way-points
            for row in wp_data_mat:

                numwp = int(row[0])
                new_target_wpx = row[1]
                new_target_wpy = row[2]
                new_target_wp = [new_target_wpx, new_target_wpy]  # find a solution for this uggly workaround...
                meastime = row[3]

                # estimate time left for plot title
                if numwp == 0:
                    starttime = float(t.time())
                    t_left_h = 0
                    t_left_m = 0
                    t_left_s = 0
                else:
                    time_per_point = (float(t.time()) - starttime) / (numwp + 1)  # as numwp starts at 0
                    time_left_sec = time_per_point * (totnumofwp-numwp+1)
                    m, t_left_s = divmod(time_left_sec, 60)
                    t_left_h, t_left_m = divmod(m, 60)

                if self.transmit_wp_to_gantry(new_target_wp):
                    if self.move_gantry_to_target():
                        if self.confirm_arrived_at_target_wp():
                            t.sleep(.25)  # wait to damp motion/oscillation of antenna etc

                            print('START Measurement for ' + str(meastime) + 's')
                            print('Measuring at Way-Point #' + str(numwp) + ' of ' + str(totnumofwp) + ' way-points')
                            plt.plot(new_target_wp[0], new_target_wp[1], 'go')
                            plt.title('Way-Point #' + str(numwp) + ' of ' + str(totnumofwp) + ' way-points ' +
                                      '- Time left: %d:%02d:%02d' % (t_left_h, t_left_m, t_left_s))
                            #dataseq = self.__oCal.take_measurement(meastime)
                            dataseq = self.__oRf.take_measurement(meastime)


                            [nummeas, numtx] = np.shape(dataseq)

                            # way point data - structure 'wp_x, wp_y, num_wp, num_tx, num_meas'
                            str_base_data = str(new_target_wp[0]) + ', ' + str(new_target_wp[1]) + ', ' +\
                                            str(numwp) + ', ' + str(numtx) + ', ' + str(nummeas) + ', '
                            # freq data
                            str_freqs = ', '.join(map(str, freqtx)) + ', '

                            # rss data - str_rss structure: 'ftx1.1, ftx1.2, [..] ,ftx1.n, ftx2.1, ftx2.2, [..], ftx2.n
                            #print('data ' + str(dataseq))
                            str_rss = ''
                            for i in range(numtx):
                                str_rss = str_rss + ', '.join(map(str, dataseq[:, i])) + ', '

                            measfile.write(str_base_data + str_freqs + str_rss + '\n')
                            # print(str_base_data + str_freqs + str_rss)

                    else:
                        print ('Error: Failed to move gantry to new way-point!')
                        print ('Way-point #' + str(numwp) + ' @ position x= ' +
                               str(new_target_wp[0]) + ', y = ' + str(new_target_wp[1]))

                else:
                    print ('Error: Failed to transmit new way-point to gantry!')
                    print ('point# ' + str(numwp) + ' @ position x= ' +
                           str(new_target_wp[0]) + ', y = ' + str(new_target_wp[1]))
                plt.pause(0.001)
                print
            measfile.close()

            self.__oScX.close_port()
            self.__oScY.close_port()

        return True


    def start_RfEar(self, center_freq=434.2e6, freqspan=1e5):
        import rf
        self.__oRf = rf.RfEar(center_freq, freqspan)
        # freqtx = [433.9e6, 434.15e6, 434.40e6, 434.65e6]
        # tx_pos = [[790, 440],
        #          [2530, 460],
        #          [2530, 1240],
        #          [790, 1230]]
        # self.__oRf.set_txparams(freqtx, tx_pos)
        freq6tx = [434.00e6,  434.1e6, 434.30e6, 434.45e6, 434.65e6, 433.90e6]
        """
        tx_6pos = [[700, 440],
           [1560,450],
           [2440, 460],
           [2440, 1240],
           [1560, 1235],
           [700, 1230]]
           """

        tx_6pos = [[520, 430],
                   [1540, 430],
                   [2570, 430],
                   [2570, 1230],
                   [1540, 1230],
                   [530, 1230]]
        self.__oRf.set_txparams(freq6tx, tx_6pos)
        return True


