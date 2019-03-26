"""
XEM6010 Phase-lock box GUI, displays diagnostics on the raw signal, phase noise measurements, and loop filters tuning
by JD Deschenes, October 2013

"""
from __future__ import print_function
import time
from PyQt5 import QtGui, Qt
#import PyQt5.Qwt5 as Qwt
import numpy as np
import math
from scipy.signal import lfilter
from scipy.signal import decimate
from scipy.signal import detrend

import os
import errno

#from SuperLaserLand_JD2 import SuperLaserLand_JD2
from LoopFiltersUI import LoopFiltersUI
from DisplayVNAWindow import DisplayVNAWindow
#from LoopFiltersUI_DAC1_and_DAC2 import LoopFiltersUI_DAC1_and_DAC2
from DisplayDitherSettingsWindow import DisplayDitherSettingsWindow
#from DisplayCrashMonitorWindow import DisplayCrashMonitorWindow
#from ILX_laser_control import ILX_laser_control
#from PyDAQmx_single_1 import NIDAQ_USB
#from NIUSB_DAQ import Instrument
from user_friendly_QLineEdit import user_friendly_QLineEdit

import SuperLaserLand_JD_RP
#import matplotlib.pyplot as plt

import traceback


# stuff for Python 3 port
import pyqtgraph as pg
from ThermometerWidget import ThermometerWidget # to replace Qwt's thermometer widget

def smooth(x,window_len=11,window='hanning'):
    """smooth the data using a window with requested size.

    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.

    input:
        x: the input signal
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.

    output:
        the smoothed signal

    example:

    t=linspace(-2,2,0.1)
    x=sin(t)+randn(len(t))*0.1
    y=smooth(x)

    see also:

    numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
    scipy.signal.lfilter

    TODO: the window parameter could be the window itself if an array instead of a string
    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

    if x.ndim != 1:
        raise ValueError("smooth only accepts 1 dimension arrays.")

    if x.size < window_len:
        raise ValueError("Input vector needs to be bigger than window size.")


    if window_len<3:
        return x


    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError("Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'")


    s=np.r_[x[window_len-1:0:-1],x,x[-1:-window_len:-1]]
    #print(len(s))
    if window == 'flat': #moving average
        w=np.ones(window_len,'d')
    else:
        w=eval('np.'+window+'(window_len)')

    y=np.convolve(w/w.sum(),s,mode='valid')
    return y


class XEM_GUI_MainWindow(QtGui.QWidget):

    display_phase = 0 # used to refresh the phase noise plot only once every N refresh cycles
    VCO_detected_gain_in_Hz_per_Volts = [1, 1, 1]

#    def __init__(self):
#        super(XEM_GUI_MainWindow, self).__init__()
#
#        self.bDisplayTiming = False # Activate to turn a lot of timing print()s
#        self.output_controls = (True, True, True)
#        self.initUI()

    def __init__(self, sl, strTitle, selected_ADC, output_controls, sp, custom_style_sheet, strFGPASerialNumber):
        assert isinstance(sl, SuperLaserLand_JD_RP.SuperLaserLand_JD_RP)

        super(XEM_GUI_MainWindow, self).__init__()
        self.strTitle = strTitle
        self.sl = sl
        self.sp = sp    # Holds the system parameters (configuration values)
        self.bDisplayTiming = False # Activate to turn a lot of timing print()s
        self.selected_ADC = selected_ADC
        self.output_controls = output_controls
        self.setObjectName('MainWindow')
        self.setStyleSheet(custom_style_sheet)
        self.strFGPASerialNumber = strFGPASerialNumber



        # For the crash monitor
        self.crash_number = 0
        self.crash_windows = []
        self.crash_windows_opening_times = []

        self.filtered_baseband_snr = 0.
        self.bAveragePhaseNoise = True
        self.bAveragePhaseNoiseLast = False
        self.N_spc_average = 10.

        self.initUI()


#    def reject(self):
#        print('User closed the window.')
##        super(XEM_GUI_MainWindow, self).reject()


#    def setDACOffset_event(self, e):

    def getValues(self):
        self.getDACoffset()
        self.getVCOFreq()

        self.qloop_filters[self.selected_ADC].getValues() # We should get qloop_filters.kc here
        self.getVCOGain()
        self.setLock()

        self.timerIDDither = Qt.QTimer(self)
        self.timerIDDither.timeout.connect(self.timerDitherEvent)
        self.startTimers()
        self.displayDAC()   # This populates the current DAC values with the actual value
        self.refreshChk_event()

    def retrieveValues(self):
        outdict = {}
        dac_offset = round(float(self.q_dac_offset[self.selected_ADC].value() / (2.**15.-1)), 4)
        vco_gain = self.qedit_vco_gain[self.selected_ADC].text()
        limit_low = round(float(self.sl.DACs_limit_low[self.selected_ADC] / (2.**15.-1)), 4)      
        limit_high = round(float(self.sl.DACs_limit_high[self.selected_ADC] / (2.**15.-1)), 4)
        ref_freq = self.qedit_ref_freq.text()
        dac_limited = int(self.qchk_dac_limited[self.selected_ADC].isChecked())

        kp = self.qloop_filters[self.selected_ADC].qedit_kp.text()
        fi = self.qloop_filters[self.selected_ADC].qedit_fi.text()
        fii = self.qloop_filters[self.selected_ADC].qedit_fii.text()
        fd = self.qloop_filters[self.selected_ADC].qedit_fd.text()
        fdf = self.qloop_filters[self.selected_ADC].qedit_fdf.text()
        chkKp = str(self.qloop_filters[self.selected_ADC].qchk_kp.isChecked())
        chkKd = str(self.qloop_filters[self.selected_ADC].qchk_kd.isChecked())
        chkLock = str(self.qloop_filters[self.selected_ADC].qchk_lock.isChecked())
        chkKpCrossing = str(self.qloop_filters[self.selected_ADC].qchk_bKpCrossing.isChecked())

#        kp = []
#        fi = []
#        fii = []
#        fd = []
#        fdf = []
#        chkKp = []
#        chkKd = []
#        chkLock = []
#        chkKpCrossing = []
#        
#        kp[i] = self.qloop_filters[i].qedit_kp.text()
#        fi[i] = self.qloop_filters[i].qedit_fi.text()
#        fii[i] = self.qloop_filters[i].qedit_fii.text()
#        fd[i] = self.qloop_filters[i].qedit_fd.text()
#        fdf[i] = self.qloop_filters[i].qedit_fdf.text()
#        chkKp[i] = int(self.qchk_kp.isChecked())
#        chkKd[i] = int(self.qchk_kd.isChecked())
#        chkLock[i] = int(self.qchk_lock.isChecked())
#        chkKpCrossing[i] = int(self.qchk_bKpCrossing.isChecked())
        
        outdict['dac_offset'] = dac_offset
        outdict['vco_gain'] = vco_gain
        outdict['limit_low'] = limit_low
        outdict['limit_high'] = limit_high
        outdict['ref_freq'] = ref_freq
        outdict['dac_limited'] = dac_limited
        outdict['kp'] = kp
        outdict['fi'] = fi
        outdict['fii'] = fii
        outdict['fd'] = fd
        outdict['fdf'] = fdf
        outdict['chkKp'] = chkKp
        outdict['chkKd'] = chkKd
        outdict['chkLock'] = chkLock
        outdict['chkKpCrossing'] = chkKpCrossing
#        print('DAC offsets: ', dac_offset)
#        print('vco_gain: ', vco_gain)
#        print('limit_lows: ', limit_low)
#        print('limit_highs: ', limit_high)
#        print('reference freq: ', ref_freq)
#        print('DAC limited? ', dac_limited)
        return outdict

    def pushActualValues(self):
        print("Push actual values of MainWindow")


    def pushDefaultValues(self):
        print("Push default values of MainWindow")
        #For now, equivalent to call initSL()

        self.loadParameters()


        # Send values to FPGA
        self.setVCOFreq_event()
        self.setVCOGain_event()
        # self.setDACOffset_event()  # not needed because setVCOGain_event calls it anyway
        self.chkLockClickedEvent()
        if self.output_controls[0] == True:
            self.setPWM0_event()

        self.timerIDDither = Qt.QTimer(self)
        self.timerIDDither.timeout.connect(self.timerDitherEvent)
        self.startTimers()

        self.displayDAC()   # This populates the current DAC values with the actual value
        self.refreshChk_event()

    def killTimers(self):
        print("%s kill timers" % self.strTitle)
        self.timerIDDither.stop()

    def startTimers(self):
        print("%s start timers" % self.strTitle)
        # Need to init timerID
        self.timerID = 0

        # Start the timer which reads the dither:
        self.timerIDDither.start(100)   # 100 ms readout delay, increased to 1000 ms for debugging

    def updateDAClimit_event(self): # limits or unlimits DAC outputs
        for k in range(3):
            if self.output_controls[k] == True:
                if self.qchk_dac_limited[k].isChecked(): # limit DAC
                    print('limit DAC')
                    self.sl.set_dac_limits(k, 0, 2**15-1)
                else: # unlimit DAC
                    print('unlimit DAC')
                    self.sl.set_dac_limits(k, -2**15, 2**15-1)

    def updateDACOffset_event(self): # Transfers slider value to qedit and calls set_DACOffset_event
        for k in range(3):
            if self.output_controls[k] == True:
				# Convert from slider units to volts (via counts)
                try:
                    q_dac_offset_in_volts = float(self.qedit_dac_offset[k].text())
                except ValueError:
                    q_dac_offset_in_volts = self.q_dac_offset[k].value()
                    pass
                q_dac_offset_in_counts = int(float(q_dac_offset_in_volts) * (2.**15.-1))
#                q_dac_offset_in_sliders = int(float(q_dac_offset_in_counts - self.sl.DACs_limit_low[k])/float(self.sl.DACs_limit_high[k] - self.sl.DACs_limit_low[k])*1e6)
#				print('%f volts to %d counts to %d slides' % (q_dac_offset_in_volts, q_dac_offset_in_counts, q_dac_offset_in_sliders))
                
                # Update q_dac_offset
                self.q_dac_offset[k].blockSignals(True)
                self.q_dac_offset[k].setValue(q_dac_offset_in_counts)
                self.q_dac_offset[k].blockSignals(False)
                self.setDACOffset_event() # Update RP output

    def setDACOffset_event(self):
        for k in range(3):
            if self.output_controls[k] == True:
                # Slider units are the same as DAC counts
                counts_offset = int(self.q_dac_offset[k].value())
                self.sl.set_dac_offset(k, counts_offset)
                try:
                    VCO_gain_in_Hz_per_Volts = float(self.qedit_vco_gain[k].text())
                except:
                    VCO_gain_in_Hz_per_Volts = 1e9

                # Update the display:
                current_output_in_volts = self.sl.dev.DAC_V_INT * counts_offset
                current_output_in_hz = current_output_in_volts * VCO_gain_in_Hz_per_Volts
                self.qlabel_dac_offset_value[k].setText('{:.4f} V\n{:.0f} MHz'.format(current_output_in_volts, current_output_in_hz/1e6))
                self.qedit_dac_offset[k].setText(str(np.round(current_output_in_volts,3)))


    def getDACoffset(self):
        for k in range(3):
            if self.output_controls[k] == True:
                self.sl.get_dac_limits(k)
                counts_offset = self.sl.get_dac_offset(k)
                # Slider units are the same as DAC counts
                q_dac_offset = counts_offset

                self.q_dac_offset[k].blockSignals(True)
                self.q_dac_offset[k].setValue(q_dac_offset)
                self.q_dac_offset[k].blockSignals(False)

                try:
                    VCO_gain_in_Hz_per_Volts = float(self.qedit_vco_gain[k].text())
                except:
                    VCO_gain_in_Hz_per_Volts = 1e9

                # Update the display:
                current_output_in_volts = self.sl.dev.DAC_V_INT * counts_offset
                current_output_in_hz = current_output_in_volts * VCO_gain_in_Hz_per_Volts
                self.qlabel_dac_offset_value[k].setText('{:.4f} V\n{:.0f} MHz'.format(current_output_in_volts, current_output_in_hz/1e6))
                self.qedit_dac_offset[k].setText(str(np.round(current_output_in_volts,3)))

                # Update DAC limited checkbox
                if int(self.sl.DACs_limit_low[k]) == 0: # is this a safe comparator type-wise?
                    self.qchk_dac_limited[k].setChecked(True)
                    print('DAC %d already limited' % k)
                else:
                    self.qchk_dac_limited[k].setChecked(False)

    def setVCOGain_event(self):
#        print('setVCOGain_event(): Entering')
#        self.setFLL0_event()

        # Update the loop filters gain settings based on the new VCO gains:

        # Also set the scale on the manual output sliders (and the steps)
        # We want the user to be able to easily control the beat frequency with the mousewheel.
        # (mousewheel scroll: 3 small steps or arrow keys: 1 small step)
        # We want each mousewheel step to be about 0.5 MHz,
        # large steps (clicking in the open area of the scrollbar) to be about 5 MHz
        for k in range(3):
            if self.output_controls[k]:
                try:
                    VCO_gain_in_Hz_per_Volts = float(self.qedit_vco_gain[k].text())
                except:
                    VCO_gain_in_Hz_per_Volts = 1e9

                # getFreqDiscriminatorGain is in DDC Counts/Hz
                # getDACGainInVoltsPerCounts is in V/(DAC Counts)
                VCO_gain_in_counts_per_counts = VCO_gain_in_Hz_per_Volts * self.sl.pll[k].getFreqDiscriminatorGain() * self.sl.dev.DAC_V_INT

                # print('VCO_gain_in_counts_per_counts = %f' % VCO_gain_in_counts_per_counts)
                if k == 0 or k == 1:
                    self.qloop_filters[k].kc = VCO_gain_in_counts_per_counts
                    self.qloop_filters[k].checkFirmwareLimits()
                    self.qloop_filters[k].updateFilterSettings()
                    self.qloop_filters[k].updateGraph()
                elif k == 2:
                    # DAC 2 loop settings are controlled by the same widget as DAC1
                    self.qloop_filters[1].kc_dac2 = VCO_gain_in_counts_per_counts
                    self.qloop_filters[1].checkFirmwareLimits()
                    self.qloop_filters[1].updateFilterSettings()
                    self.qloop_filters[1].updateGraph()

                self.setSliderStep(k, VCO_gain_in_Hz_per_Volts)

        # This function needs the VCO gain to compute the control effort so we have to update it if we have changed.
        self.setDACOffset_event()

    def getVCOGain(self):
        for k in range(3):
            if self.output_controls[k]:
                VCO_gain_in_counts_per_counts = self.qloop_filters[self.selected_ADC].kc # must be called after
                VCO_gain_in_Hz_per_Volts = VCO_gain_in_counts_per_counts / (self.sl.pll[k].getFreqDiscriminatorGain() * self.sl.dev.DAC_V_INT)

                self.qedit_vco_gain[k].blockSignals(True)
                self.qedit_vco_gain[k].setText('{:.1e}'.format(VCO_gain_in_Hz_per_Volts))
                self.qedit_vco_gain[k].blockSignals(False)

                if k == 0 or k == 1:
                    self.qloop_filters[k].kc = VCO_gain_in_counts_per_counts
                    self.qloop_filters[k].checkFirmwareLimits()
                    self.qloop_filters[k].updateGraph()
                elif k == 2:
                    # DAC 2 loop settings are controlled by the same widget as DAC1
                    self.qloop_filters[1].kc_dac2 = VCO_gain_in_counts_per_counts
                    self.qloop_filters[1].checkFirmwareLimits()
                    self.qloop_filters[1].updateGraph()

                self.setSliderStep(k, VCO_gain_in_Hz_per_Volts)

    def setSliderStep(self, k, VCO_gain_in_Hz_per_Volts):
        # Units for the slider are millionth of fullscale (goes from 0 to 1e6), which maps to [DAC_limit_low, DAC_limit_high]
        # The times three is because the scroll wheel actually does 3 small_steps (this is settings in Windows and can change from one computer to the next..)
        small_step = int(1/3. * 50e3 / VCO_gain_in_Hz_per_Volts / self.sl.dev.DAC_V_INT)
        large_step = int(500e3 / VCO_gain_in_Hz_per_Volts / self.sl.dev.DAC_V_INT)

        if small_step < 1:
            small_step = 1
        if small_step > self.sl.dev.DAC_INT_HR/(3*100):
            small_step = self.sl.dev.DAC_INT_HR/(3*100)
        if large_step < self.sl.dev.DAC_INT_HR/(100):
            large_step = self.sl.dev.DAC_INT_HR/(100)
        if large_step > self.sl.dev.DAC_INT_HR/(10):
            large_step = self.sl.dev.DAC_INT_HR/(10)

        self.q_dac_offset[k].setSingleStep(small_step)
        self.q_dac_offset[k].setPageStep(large_step)
    ##
    ## HB, 4/27/2015, Added PWM support on DOUT0
    ##
    def setPWM0_event(self):
        # Get slider units
        slider_units = float(self.q_pwm0_value.value())
        # Convert to volts
        value_in_volts = slider_units*1e-6*(self.PWM0_max-self.PWM0_min)+self.PWM0_min
        # Convert to counts
        value_in_counts = self.sl.convertPWMVoltsToCounts(self.PWM0_standard, self.PWM0_levels, value_in_volts)
        # Send to FPGA
        self.sl.set_pwm_settings(self.PWM0_levels, value_in_counts)
        # Update label
        self.qlabel_pwm0_value.setText('%.2f V' % (value_in_volts))

    def setVCOFreq_event(self):
        try:
            frequency_in_hz = float(self.qedit_ref_freq.text())
        except:
            frequency_in_hz = 5e6

        # If the VCO has positive sign, we need to put a negative reference frequency to make the
        # total loop sign be negative so that it's stable when we close the loop
        if self.qsign_positive.isChecked():
            frequency_in_hz = -frequency_in_hz
#        print('frequency_in_hz = %e' % frequency_in_hz)
        if self.selected_ADC == 0:
            self.sl.set_ddc0_ref_freq(frequency_in_hz)
        elif self.selected_ADC == 1:
            self.sl.set_ddc1_ref_freq(frequency_in_hz)

    def getVCOFreq(self):
        if self.selected_ADC == 0:
            frequency_in_hz = self.sl.get_ddc0_ref_freq_from_RAM()
        elif self.selected_ADC == 1:
            frequency_in_hz = self.sl.get_ddc1_ref_freq_from_RAM()

        # If the VCO has positive sign, we need to put a negative reference frequency to make the
        # total loop sign be negative so that it's stable when we close the loop
        if frequency_in_hz < 0:
            self.qsign_positive.setChecked(True)
        else:
            self.qsign_negative.setChecked(True)



        self.qedit_ref_freq.blockSignals(True)
        self.qedit_ref_freq.setText('%.2e' % abs(frequency_in_hz))
        self.qedit_ref_freq.blockSignals(False)


    def refreshChk_event(self):
#        print('refreshChk_event()')
        # We are doing a not running->running transition
        try:
#            if True:
#                print('self.qedit_timerdelay.text() = %s' % self.qedit_timerdelay.text())
            timer_delay = float(self.qedit_timerdelay.text())

        except:
#            else:
            timer_delay = 1000
#            print('Timer delay = %d ms' % timer_delay)
        if self.timerID != 0:
            self.killTimer(self.timerID)
        self.timerID = self.startTimer(int(round(timer_delay)))
        self.timerEvent(0)  # run the event handler once right away, makes the checkbox feel more responsive
#            print('Starting timer')

    def exportData(self):
        # First need to create a unique file name template (with good probability)
        # We simply use the system date and time, and hope that this function doesn't get called twice in a second
        strNameTemplate = time.strftime("data_export\%m_%d_%Y_%H_%M_%S_")
#        Data to write:
#        self.inst_freq
#        self.freq_noise_psd
#        self.freq_noise_axis
#        self.raw_adc_samples

        # Create the subdirectory if it doesn't exist:
        os.makedirs('data_export', exist_ok=True)

        # Open files for output, write raw data
#        if True:
        try:
            strCurrentName = strNameTemplate + 'raw_adc_samples.bin'
            f = open(strCurrentName, 'wb')
            f.write(self.raw_adc_samples)
            f.close()
        except:
            pass
        try:
            strCurrentName = strNameTemplate + 'inst_freq.bin'
            f = open(strCurrentName, 'wb')
            f.write(self.inst_freq)
            f.close()
        except:
            pass
        try:
            strCurrentName = strNameTemplate + 'freq_noise_psd.bin'
            f = open(strCurrentName, 'wb')
            f.write(self.freq_noise_psd)
            f.close()
        except:
            pass
        try:
            strCurrentName = strNameTemplate + 'freq_noise_axis.bin'
            f = open(strCurrentName, 'wb')
            f.write(self.freq_noise_axis)
            f.close()
        except:
            pass


    def showVNA(self):
        self.vna = DisplayVNAWindow(self.sl)



    def grabAndExportData(self):

        start_time = time.clock()
        print('Grabbing and exporting data')
        # Check if another function is currently using the DDR2 logger:
        if self.sl.bDDR2InUse:
            print('grabAndExportData(): DDR2 logger in use, cannot get data from adc')
            return

        # Ask which input to use:
        currentSelector, ok = QtGui.QInputDialog.getItem(self, 'Raw data export',
            'Select the data source:', ('ADC0', 'ADC1', 'DAC0', 'DAC1', 'DAC2'))
        if not ok:
            return
        currentSelector = str(currentSelector)

        # Ask how many points:
        N_points_str, ok = QtGui.QInputDialog.getText(self, 'Raw data export',
            'Enter the number of points desired [1, 32768]:', Qt.QLineEdit.Normal, '32768')
        if not ok:
            return

        # Block access to the DDR2 Logger to any other function until we are done:
        self.sl.bDDR2InUse = True

#        try:
#            N_points = int(float(self.qedit_rawdata_length.text()))
#        except:
#            N_points = 4e3
        try:
            N_points = int(float(N_points_str))
        except:
            N_points = 4e3

        # if N_points < 4e3:
        #     N_points = 4e3

        if N_points < 64:
            N_points = 64

#        currentSelector = self.qcombo_adc_plot.currentIndex()
        # Python doesn't have switch-case statements
        # Apparently the best way is to use a dictionary instead:
        setup_func_dict = {'ADC0': self.sl.setup_ADC0_write,
                           'ADC1': self.sl.setup_ADC1_write,
                           'DAC0': self.sl.setup_DAC0_write,
                           'DAC1': self.sl.setup_DAC1_write,
                           'DAC2': self.sl.setup_DAC2_write}


        try:
            # Read from selected source
            setup_func_dict[currentSelector](N_points)

            ##################################################
            # Synchronize trigger as best as possible to the next multiple of time_quantum seconds:
            time_quantum = 0.01
            time_now = time.time()
            time_target = np.ceil(time_now/time_quantum) * time_quantum
            print('time_now = %f, time_target = %f' % (time_now, time_target))

            while time_target > time_now:
                time.sleep(1e-3)
                time_now = time.time()



            self.sl.trigger_write()
            print('time_now = %f, time_target = %f' % (time_now, time_target))
            self.sl.wait_for_write()
            (samples_out, ref_exp0) = self.sl.read_adc_samples_from_DDR2()
            samples_out = samples_out.astype(dtype=np.float) * self.sl.dev.ADC_V_INT
        except:
            # ADC read failed.
            print('Unhandled exception in ADC read')
#            del self.sl
#            raise

        # Signal to other functions that they can use the DDR2 logger
        self.sl.bDDR2InUse = False

        print('Elapsed time (Comm) = %f' % (time.clock()-start_time))
        start_time = time.clock()

        # Write the data to disk:
        strNameTemplate = time.strftime("data_export\%m_%d_%Y_%H_%M_%S_")

        os.makedirs('data_export', exist_ok=True)
        # Open files for output, write raw data
        try:
            strCurrentName = strNameTemplate + self.strFGPASerialNumber +  '_raw_adc_samples.bin'
            f = open(strCurrentName, 'wb')
            f.write(samples_out)
            f.close()
        except:
            pass

        print('Elapsed time (write to disk) = %f' % (time.clock()-start_time))
        start_time = time.clock()

    def reset_front_end(self):
        self.sl.reset_front_end()


    def openDitherControls(self):
        self.dither_settings = DisplayDitherSettingsWindow(self.sl)

    def setLock(self):
        bLock = self.qloop_filters[self.selected_ADC].qchk_lock.isChecked()
        self.qchk_lock.setChecked(bLock)
        if bLock:
            #We are reconnecting to a RP which has a locked loop filter
            self.qchk_lock.setStyleSheet('font-size: 18pt; color: white; background-color: green')
        else:
            self.qchk_lock.setStyleSheet('font-size: 18pt; color: white; background-color: red')

    def chkLockClickedEvent(self):
        bLock = self.qchk_lock.isChecked()
        if bLock:
            # we are doing an unlocked->locked transition.

            # We first check if the detected VCO gain seems right:
            if self.sl.dither_enable[self.selected_ADC]:
                # check if gain is OK
                try:
                    VCO_gain_in_Hz_per_Volts = float(self.qedit_vco_gain[self.selected_ADC].text())
                except:
                    VCO_gain_in_Hz_per_Volts = 1e9
                # First check if sign is right:
                if np.sign(self.VCO_detected_gain_in_Hz_per_Volts[self.selected_ADC]) != np.sign(VCO_gain_in_Hz_per_Volts):
                    # Display warning message.
                    reply = QtGui.QMessageBox.question(self, 'Warning',
                                "The detected VCO gain is negative.  This will most likely make the loop unstable.  This is either caused by trying to lock to an incorrect sideband, or an incorrect setting of the VCO sign in the UI.  Do you want to turn on the lock anyway?",
                                QtGui.QMessageBox.Yes | QtGui.QMessageBox.No, QtGui.QMessageBox.No)

                    if reply == QtGui.QMessageBox.No:
                        # Exit early
                        self.qchk_lock.setChecked(False)
                        return
                    print('Warning about the loop sign ignored.')
                else:
                    print('Gain sign OK')
                # Now we check if the magnitude of the entered VCO gain and the detected gain agree within some tolerance:
                if self.VCO_detected_gain_in_Hz_per_Volts[self.selected_ADC]/VCO_gain_in_Hz_per_Volts > 1.5 or self.VCO_detected_gain_in_Hz_per_Volts[self.selected_ADC]/VCO_gain_in_Hz_per_Volts < 1/1.5:
                    # Display warning message.
                    reply = QtGui.QMessageBox.question(self, 'Warning',
                                "The detected VCO gain (%.2e Hz/V) has a significantly different magnitude than the entered value used for designing the controller (%.2e Hz/V).  This may make the loop unstable.  Do you want to turn on the lock anyway?" % (self.VCO_detected_gain_in_Hz_per_Volts[self.selected_ADC], VCO_gain_in_Hz_per_Volts),
                                QtGui.QMessageBox.Yes | QtGui.QMessageBox.No, QtGui.QMessageBox.No)

                    if reply == QtGui.QMessageBox.No:
                        # Exit early
                        self.qchk_lock.setChecked(False)
                        return
                    print('Warning about the loop gain ignored.')
                else:
                    print('Gain magnitude OK')

            # If we get here that means either that all the parameters have passed the checks, or the dither was off.
            # Turn the dither off if the dither mode is automatic:
            if self.selected_ADC == 0:
                if self.sl.dither_mode_auto[0] == 1:
                    # automatic mode
                    self.sl.setDitherLockInState(0, False)
            else:
                # Optical lock: we have two dithers to take care of:
                if self.sl.dither_mode_auto[1] == 1:
                    # automatic mode
                    self.sl.setDitherLockInState(1, False)
                # if self.sl.dither_mode_auto[2] == 1:
                #     # automatic mode
                #     self.sl.setDitherLockInState(2, False)


            self.qchk_lock.setStyleSheet('font-size: 18pt; color: white; background-color: green')
            # Turn the lock on
            if self.selected_ADC == 0:
                self.qloop_filters[0].qchk_lock.setChecked(True)
                self.qloop_filters[0].updateFilterSettings()

            elif self.selected_ADC == 1:
                self.qloop_filters[1].qchk_lock.setChecked(True)
                self.qloop_filters[1].updateFilterSettings()

                # LEGACY procedure for the NIST lockbox with two DACs on the 2nd PLL channel:
                # # There is a different procedure for turning the lock on on the optical loop:
                # # first we grab the beat using the DAC2 frequency-locked loop. then we set this integrator to hold
                # # and switch to the DAC1 PLL + DAC2 second integrator.
                # self.qloop_filters[1].qradio_mode_off.setChecked(False)
                # self.qloop_filters[1].qradio_mode_slow.setChecked(True)
                # self.qloop_filters[1].qradio_mode_fast.setChecked(False)
                # self.qloop_filters[1].qradio_mode_both.setChecked(False)
                # self.qloop_filters[1].updateSettings()

                # # Wait for the integrator to grab on to the beat
                # time.sleep(0.2)

                # # Turn on the full-blown PLL
                # self.qloop_filters[1].qradio_mode_off.setChecked(False)
                # self.qloop_filters[1].qradio_mode_slow.setChecked(False)
                # self.qloop_filters[1].qradio_mode_fast.setChecked(False)
                # self.qloop_filters[1].qradio_mode_both.setChecked(True)
                # self.qloop_filters[1].updateSettings()


        else:   # bLock = False
            if not self.sl.output_vco[self.selected_ADC]:
                # We are doing a locked->unlocked transition
                # Turn the lock off
                if self.selected_ADC == 0:
                    self.qloop_filters[0].qchk_lock.setChecked(False)
                    self.qloop_filters[0].updateFilterSettings()
    #                self
    #                self.updateGraph
                elif self.selected_ADC == 1:
                    self.qloop_filters[1].qchk_lock.setChecked(False)
                    self.qloop_filters[1].updateFilterSettings()

                # Update the manual dac offsets to where the lock has decided to sit:
                # This is to prevent any violent step on the actuator when we turn off the lock:
                # It also prevents mode changes (the laser should stay fairly close to when it was locked.
                if self.selected_ADC == 0:
                    # Go and measure the current DAC DC value:
                    N_points = 10e3
                    self.sl.setup_DAC0_write(N_points)
                    self.sl.trigger_write()
                    self.sl.wait_for_write()
                    samples_out = self.sl.read_dac_samples_from_DDR2()
#                    print(np.mean(samples_out))
                    current_dac_offset_in_counts = np.mean(samples_out)
                    kDAC = 0


                elif self.selected_ADC == 1:
                    N_points = 10e3
                    self.sl.setup_DAC1_write(N_points)
                    self.sl.trigger_write()
                    self.sl.wait_for_write()
                    samples_out = self.sl.read_dac_samples_from_DDR2()
#                    print(np.mean(samples_out))
                    current_dac_offset_in_counts = np.mean(samples_out)
                    kDAC = 1

                # Read the current manual offset value:
#                current_manual_offset_in_slider_units = float(self.q_dac_offset[kDAC].value())
                # Convert the DAC DC offset to the slider units:
                current_dac_offset_in_slider_units = current_dac_offset_in_counts # same as slider units

                self.q_dac_offset[kDAC].setValue(current_dac_offset_in_slider_units)
                #self.setDACOffset_event()

#                # Set up a ramp with 20 steps:
#                desired_ramp = np.linspace(current_manual_offset_in_slider_units, current_dac_offset_in_slider_units, 20)
#                print('ramping from %d to %d in slider units' % (current_manual_offset_in_slider_units, current_dac_offset_in_slider_units))
#
#                Total_ramp_time = 0.1
#                for k2 in range(len(desired_ramp)):
##                    print('set slider to %d' % desired_ramp[k2])
#                    self.q_dac_offset[kDAC].setValue(desired_ramp[k2])
#                    self.setDACOffset_event()
#                    time.sleep(float(Total_ramp_time)/len(desired_ramp))
            else:
                # if the VCO is activated, we don't want to try to estimate the output offset, we just turn off the lock directly
                # 2. turn the lock off
                if self.selected_ADC == 0:
                    self.qloop_filters[0].qchk_lock.setChecked(False)
                    self.qloop_filters[0].updateFilterSettings()
                elif self.selected_ADC == 1:
                    self.qloop_filters[1].qchk_lock.setChecked(False)
                    self.qloop_filters[1].updateFilterSettings()

            # 3. Turn the dither on if the dither mode is automatic:
            if self.selected_ADC == 0:
                if self.sl.dither_mode_auto[0] == 1:
                    # automatic mode
                    self.sl.setDitherLockInState(0, True)
            else:
                # Optical lock: we have two dithers to take care of:
                if self.sl.dither_mode_auto[1] == 1:
                    # automatic mode
                    self.sl.setDitherLockInState(1, True)
                # if self.sl.dither_mode_auto[2] == 1:
                #     # automatic mode
                #     self.sl.setDitherLockInState(2, True)

            self.qchk_lock.setStyleSheet('font-size: 18pt; color: white; background-color: red')

    def initUI(self):
#        second_half_offset = 50
        # Change the background color of the main form so that each controls group stand out better
        PalNormal = Qt.QPalette()

        # Assign the palette to the main form to read off the 'normal' background color:
        self.setPalette(PalNormal)
        normalBackgroundRGB = PalNormal.color(Qt.QPalette.Background).getRgb()
#        print(normalBackground.getRgb())

        # Darken the background of the dialog slightly
        darker_factor = 0.5
        PalDarkerBackground = Qt.QPalette()
        PalDarkerBackground.setColor(Qt.QPalette.Background, Qt.QColor(normalBackgroundRGB[0]*darker_factor, normalBackgroundRGB[1]*darker_factor, normalBackgroundRGB[2]*darker_factor))
#        PalDarkerBackground.setColor(Qt.QPalette.Background, Qt.QColor(255, 255, 255))
        self.setPalette(PalDarkerBackground)
        self.setAutoFillBackground(True)


        # PalNormal's color has been changed when we assigned PalDarkerBackground to self - this statement seems very circular but somehow it works
        PalNormal.setColor(Qt.QPalette.Background, PalNormal.color(Qt.QPalette.Background))

        ######################################################################
        # Settings
        ######################################################################
        self.qgroupbox_settings = Qt.QGroupBox('Settings', self)

        # Button which exports the data to the disk
        self.qbtn = QtGui.QPushButton('Export PSD data')
        self.qbtn.clicked.connect(self.exportData)

        # Button which grabs a single acquisition from the DDR memory and exports the data to the disk
        self.qbtn_grab = QtGui.QPushButton('Export ADC data')
        self.qbtn_grab.clicked.connect(self.grabAndExportData)

        # Button which opens the VNA window:
        self.qbtn_VNA = QtGui.QPushButton('Transfer function')
        self.qbtn_VNA.clicked.connect(self.showVNA)

        # VCO modulation gain:
        self.qedit_vco_gain = {}

        self.qlabel_detected_vco_gain = {}
        if self.selected_ADC == 0:
            # CEO Lock: only one output (DAC0)
            self.qlabel_vco_gain = Qt.QLabel('VCO Gain (DAC0) [Hz/V]:')
            self.qlabel_detected_vco_gain_label = Qt.QLabel('Detected VCO Gain [Hz/V]:')

            self.qedit_vco_gain[0] = user_friendly_QLineEdit('1e6')
            self.qedit_vco_gain[0].returnPressed.connect(self.setVCOGain_event)
            self.qedit_vco_gain[0].setMaximumWidth(60)

            self.qlabel_detected_vco_gain[0] = Qt.QLabel('0 Hz/V')
            self.qlabel_detected_vco_gain[0].setAlignment(Qt.Qt.AlignHCenter)

        else:
            # Optical lock
            # self.qlabel_vco_gain = Qt.QLabel('VCO Gains (DAC1, DAC2HV) [Hz/V]:')
            self.qlabel_vco_gain = Qt.QLabel('VCO Gain (DAC1) [Hz/V]:')

            self.qlabel_detected_vco_gain_label = Qt.QLabel('Detected VCO Gain [Hz/V]:')

            self.qedit_vco_gain[1] = user_friendly_QLineEdit('1e6')
            self.qedit_vco_gain[1].returnPressed.connect(self.setVCOGain_event)
            self.qedit_vco_gain[1].setMaximumWidth(60)

            # self.qedit_vco_gain[2] = user_friendly_QLineEdit('1e6')
            # self.qedit_vco_gain[2].returnPressed.connect(self.setVCOGain_event)
            # self.qedit_vco_gain[2].setMaximumWidth(60)

            self.qlabel_detected_vco_gain[1] = Qt.QLabel('0 Hz/V')
            self.qlabel_detected_vco_gain[1].setAlignment(Qt.Qt.AlignHCenter)

            # self.qlabel_detected_vco_gain[2] = Qt.QLabel('0 Hz/V')
            # self.qlabel_detected_vco_gain[2].setAlignment(Qt.Qt.AlignHCenter)


        # DDC reference frequency:
        self.qlabel_ref_freq = Qt.QLabel('Reference freq [Hz]:')
        self.qedit_ref_freq = user_friendly_QLineEdit('5e6')
        self.qedit_ref_freq.returnPressed.connect(self.setVCOFreq_event)
        self.qedit_ref_freq.setMaximumWidth(60)


        # Main button for turning the locks on/off:
        self.qchk_lock = Qt.QCheckBox('Lock')
        self.qchk_lock.setStyleSheet('')
        self.qchk_lock.setStyleSheet('font-size: 18pt; color: white; background-color: red')
#        self.qchk_lock.setStyleSheet('font-size: 18pt; color: white; background-color: green')
        self.qchk_lock.clicked.connect(self.chkLockClickedEvent)
        self.qchk_lock.setChecked(False)


        # Button which opens the dither controls:
        self.qbutton_dither_controls = Qt.QPushButton('')
        self.qbutton_dither_controls.clicked.connect(self.openDitherControls)

        # VCO sign:
        self.qsign_positive = Qt.QRadioButton('VCO sign +')
        self.qsign_negative = Qt.QRadioButton('VCO sign -')
        self.qsign_group = Qt.QButtonGroup(self)
        self.qsign_group.addButton(self.qsign_positive)
        self.qsign_group.addButton(self.qsign_negative)

        self.qsign_positive.setChecked(True)
        self.qsign_negative.setChecked(False)
        self.qsign_positive.clicked.connect(self.setVCOFreq_event)
        self.qsign_negative.clicked.connect(self.setVCOFreq_event)


        # Create widgets to indicate performance
        self.last_refresh = time.clock()
        self.qlabel_refreshrate_display = Qt.QLabel('Actual delay:')
        self.qlabel_refreshrate = Qt.QLabel('1000 ms')
#        self.qlabel_refreshrate.resize(self.qlabel_refreshrate.sizeHint())


        self.qlabel_timerdelay = Qt.QLabel('Refresh delay [ms]:')
        self.qedit_timerdelay = user_friendly_QLineEdit('200')
        self.qedit_timerdelay.returnPressed.connect(self.refreshChk_event)
        self.qedit_timerdelay.setMaximumWidth(60)

        # write new settings file button
        if self.selected_ADC == 0: # only have button for first window
            self.qbutton_export_json = QtGui.QPushButton('Save PID')
            self.qbutton_export_json.clicked.connect(self.sl.controller.retrieveValues)
            self.qbutton_export_json.setMaximumWidth(60)
            
            self.qedit_export_json = user_friendly_QLineEdit('locking_settings.json')
        

        # Status reporting:
        if self.selected_ADC == 0:
            self.qlbl_status1 = Qt.QLabel('Status: Idle')
        elif self.selected_ADC == 1:
            self.qlbl_status1 = Qt.QLabel('Status: Idle')
            self.qlbl_status2 = Qt.QLabel('Status: Idle')

#        self.qbtn_reset = QtGui.QPushButton('Reset frontend')
#        self.qbtn_reset.clicked.connect(self.reset_front_end)


        # Put all the widgets into a grid layout
        grid = QtGui.QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(1)

        # 3 rows, XX columns
        grid.addWidget(self.qbtn,                       0, 0)
        grid.addWidget(self.qbtn_VNA,                   1, 0)
        grid.addWidget(self.qbtn_grab,                  2, 0)

        grid.addWidget(self.qlabel_timerdelay,          1, 1)
        grid.addWidget(self.qedit_timerdelay,           1, 2)

        grid.addWidget(self.qlabel_refreshrate_display, 2, 1)
        grid.addWidget(self.qlabel_refreshrate,         2, 2)



#        grid.addWidget(self.qlabel_bytes_skip,          0, 3)
#        grid.addWidget(self.qedit_bytes_skip,           0, 4)
        grid.addWidget(self.qchk_lock,                  0, 3, 1, 2)
        grid.addWidget(self.qlabel_ref_freq,            1, 3)
        grid.addWidget(self.qedit_ref_freq,             1, 4)

        # both PLLs need to receive a threshold for the residuals.
        # See tooltip for info
#        grid.addWidget(self.qlabel_crash_threshold,     2, 3)
#        grid.addWidget(self.qedit_crash_threshold,      2, 4)




        # only the first PLL has a crash monitor module in the current firmware:
        if self.selected_ADC == 0:
            pass
            #FEATURE
            #grid.addWidget(self.qlabel_crash_threshold_freq,     2, 5)
            #grid.addWidget(self.qedit_crash_threshold_freq,      2, 6)
            #grid.addWidget(self.qchk_crash_monitor,         2, 7)



        # We put a sub-grid in the grid
        # we put the VCO controls in the sub-grid, this way the outer grid stays the same size regardless of the number of elements
        grid2 = Qt.QGridLayout()
        grid2.setHorizontalSpacing(10)
        grid2.setVerticalSpacing(10)

        if self.selected_ADC == 0:
            # CEO Lock: only one output (DAC0)
            grid2.addWidget(self.qlabel_vco_gain,                           0, 0)
            grid2.addWidget(self.qlabel_detected_vco_gain_label,            1, 0)

            grid2.addWidget(self.qedit_vco_gain[0],                         0, 1)
            grid2.addWidget(self.qlabel_detected_vco_gain[0],               1, 1)


        else:
            # Optical lock: two outputs (DAC1 and DAC2)
            grid2.addWidget(self.qlabel_vco_gain,                           0, 0)
            grid2.addWidget(self.qlabel_detected_vco_gain_label,            1, 0)

            grid2.addWidget(self.qedit_vco_gain[1],                         0, 1)
            grid2.addWidget(self.qlabel_detected_vco_gain[1],               1, 1)

            # grid2.addWidget(self.qedit_vco_gain[2],                         0, 2)
            # grid2.addWidget(self.qlabel_detected_vco_gain[2],               1, 2)

        grid.addLayout(grid2, 0, 5, 2, 2)
        grid.addWidget(self.qsign_positive,             0, 7)
        grid.addWidget(self.qsign_negative,             1, 7)
        if self.selected_ADC == 0:
            grid.addWidget(self.qbutton_export_json,        0, 8)
            grid.addWidget(self.qedit_export_json,          1, 8)



#         # Status reporting:
#         if self.selected_ADC == 0:

#             #FEATURE
#             #grid.addWidget(self.qchk_residuals_streaming,   1, 8, 1, 2)
#             grid.addWidget(self.qlbl_status1,               2, 5, 1, 2)
#             #FEATURE
#             #grid.addWidget(self.qlbl_status1,               2, 8, 1, 2)
# #            grid.addWidget(self.qtxt_adcphase,               2, 8, 1, 1)
# #            grid.addWidget(self.qbtn_send_adcphase,               2, 9, 1, 1)

#         elif self.selected_ADC == 1:
#             grid.addWidget(self.qlbl_status1,           2, 5, 1, 1)
#             grid.addWidget(self.qlbl_status2,           2, 6, 1, 1)

#        grid.addWidget(self.qbtn_reset,                 1, 7)
        grid.addWidget(Qt.QLabel(),                     0, 9, 1, 1)
        grid.setColumnStretch(9, 1)

        self.qgroupbox_settings.setLayout(grid)
        self.qgroupbox_settings.setPalette(PalNormal)
        self.qgroupbox_settings.setAutoFillBackground(True)


        ######################################################################
        # Spectrum analyzer/Diagnostics
        ######################################################################
        self.qgroupbox_diagnostics = Qt.QGroupBox('Spectrum analyzer/diagnostics (all computed from raw ADC input)', self)

        # Create the scale which indicates the ADC fill ratio:
        self.qlabel_adc_fill = Qt.QLabel('ADC fill\n[bits]')
        self.qlabel_adc_fill.setAlignment(Qt.Qt.AlignHCenter)

        self.qadc0_scale = ThermometerWidget()#Qwt.QwtThermo()
        #self.qadc0_scale.setOrientation(Qt.Qt.Vertical, Qwt.QwtThermo.LeftScale)
        self.qadc0_scale.setRange(0, 16)
        #self.qadc0_scale.setScale(0, 16)
        self.qadc0_scale.setValue(0)
        self.qadc0_scale.setFillColor(Qt.Qt.blue)
        ticksListMajor = [0, 5, 10, 15]
        ticksListMinor = [2.5, 7.5, 12.5]
        ticksLabelMajor = list(map(str, ticksListMajor))
        self.qadc0_scale.setTicks(ticksListMajor, ticksListMinor, ticksLabelMajor)

        self.qlabel_adc_fill_value = Qt.QLabel('10 bits')
        self.qlabel_adc_fill_value.setAlignment(Qt.Qt.AlignHCenter)

        # Create the scale which indicates the baseband SNR:
        self.qlabel_baseband_snr = Qt.QLabel('SNR\n[dB]')
        self.qlabel_baseband_snr.setAlignment(Qt.Qt.AlignHCenter)

        self.qthermo_baseband_snr = ThermometerWidget()#Qwt.QwtThermo()
        #self.qthermo_baseband_snr.setOrientation(Qt.Qt.Vertical, Qwt.QwtThermo.LeftScale)
        self.qthermo_baseband_snr.setRange(0, 50)
        self.qthermo_baseband_snr.setScale(0, 50)
        self.qthermo_baseband_snr.setValue(0)
        self.qthermo_baseband_snr.setFillColor(Qt.Qt.blue)
        ticksListMajor = [0, 10, 20, 30, 40, 50]
        ticksListMinor = [5, 15, 25, 35, 45]
        ticksLabelMajor = list(map(str, ticksListMajor))
        self.qthermo_baseband_snr.setTicks(ticksListMajor, ticksListMinor, ticksLabelMajor)


        self.qlabel_baseband_snr_value = Qt.QLabel('20 dB')
        self.qlabel_baseband_snr_value.setAlignment(Qt.Qt.AlignHCenter)

       # # Create the scale which indicates the average frequency error:
       # self.qlabel_ddc0_error = Qt.QLabel('Freq error\n[MHz]')
       # self.qlabel_ddc0_error.setAlignment(Qt.Qt.AlignHCenter)

       # self.qddc0_error_scale = ThermometerWidget()#Qwt.QwtThermo()
       # self.qddc0_error_scale.setOrientation(Qt.Qt.Vertical, Qwt.QwtThermo.LeftScale)
       # self.qddc0_error_scale.setRange(-5, 5)
       # self.qddc0_error_scale.setScale(-5, 5)
       # self.qddc0_error_scale.setValue(0)
       # self.qddc0_error_scale.setFillColor(Qt.Qt.blue)




        # Create the widgets which shows the current dac output and sets the output offset:
        self.qlabel_dac_current = {}
        self.qlabel_dac_current_value = {}
        self.qthermo_dac_current = {}
        self.qlabel_dac_offset = {}
        self.q_dac_offset = {}
        self.qlabel_dac_offset_value = {}
        self.qedit_dac_offset = {}
        self.qchk_dac_limited = {}

        if self.output_controls[0] == True:
            self.qlabel_pwm0 = Qt.QLabel('Output\nPWM 0 [V]')
            self.qlabel_pwm0.setAlignment(Qt.Qt.AlignHCenter)

            self.q_pwm0_value = Qt.QSlider()
            self.q_pwm0_value.valueChanged.connect(self.setPWM0_event)
            self.q_pwm0_value.setOrientation(Qt.Qt.Vertical)
            self.q_pwm0_value.setMinimum(0)
            self.q_pwm0_value.setMaximum(1e6)
            self.q_pwm0_value.setSliderPosition(0)

            self.qlabel_pwm0_value = Qt.QLabel('Value\nPWM 0 [V]')
            self.qlabel_pwm0_value.setAlignment(Qt.Qt.AlignHCenter)

        for k in range(3):
            if self.output_controls[k] == True:
                self.qlabel_dac_current[k] = Qt.QLabel('Output\nDAC %d [V]' % k)
                self.qlabel_dac_current[k].setAlignment(Qt.Qt.AlignHCenter)

                self.qthermo_dac_current[k] = ThermometerWidget()#Qwt.QwtThermo()
                #self.qthermo_dac_current[k].setOrientation(Qt.Qt.Vertical, Qwt.QwtThermo.LeftScale)
                self.qthermo_dac_current[k].setRange(self.sl.dev.DAC_V_INT * self.sl.DACs_limit_low[k], self.sl.dev.DAC_V_INT * self.sl.DACs_limit_high[k])
                self.qthermo_dac_current[k].setScale(self.sl.dev.DAC_V_INT * self.sl.DACs_limit_low[k], self.sl.dev.DAC_V_INT * self.sl.DACs_limit_high[k])
                self.qthermo_dac_current[k].setValue(0)
                #self.qthermo_dac_current[k].setFillBrush(Qt.QBrush(Qt.QColor(0, 186, 52)))
                self.qthermo_dac_current[k].setFillColor(Qt.QColor(0, 186, 52))
                ticksListMajor = [-1, -0.5, 0, 0.5, 1]
                ticksListMinor = [-0.75, -0.25, 0.25, 0.75]
                ticksLabelMajor = list(map(str, ticksListMajor))
                self.qthermo_dac_current[k].setTicks(ticksListMajor, ticksListMinor, ticksLabelMajor)


                self.qlabel_dac_offset[k] = Qt.QLabel('Offset\nDAC %d [V]' % k)
                self.qlabel_dac_offset[k].setAlignment(Qt.Qt.AlignHCenter)

                self.q_dac_offset[k] = Qt.QSlider()
                self.q_dac_offset[k].valueChanged.connect(self.setDACOffset_event)
                self.q_dac_offset[k].setSliderPosition(0)
                self.q_dac_offset[k].setOrientation(Qt.Qt.Vertical)

                # Units are the same as the full integer range of DAC values
                self.q_dac_offset[k].setMinimum(-self.sl.dev.DAC_INT_HR)
                self.q_dac_offset[k].setMaximum(self.sl.dev.DAC_INT_HR)

                self.qedit_dac_offset[k] = Qt.QLineEdit('') #JShaw
                self.qedit_dac_offset[k].setMaximumWidth(60)
                self.qedit_dac_offset[k].returnPressed.connect(self.updateDACOffset_event)

				# Limit DAC boxes. Check existing DAC limit during initialization # Doesn't work order-wise
                self.qchk_dac_limited[k] = Qt.QCheckBox('Limit DAC')
#                print('DAC %d lower limit %d' % (k, self.sl.DACs_limit_low[k])) # Useless because not yet connected to RP
#                if int(self.sl.DACs_limit_low[k]) == 0: # is this a safe comparator type-wise?
#                    self.qchk_dac_limited[k].setChecked(True)
#                    print('DAC %d already limited')
#                else:
#                    self.qchk_dac_limited[k].setChecked(False)
                self.qchk_dac_limited[k].clicked.connect(self.updateDAClimit_event)

                self.qlabel_dac_current_value[k] = Qt.QLabel('0 V')
                self.qlabel_dac_current_value[k].setAlignment(Qt.Qt.AlignHCenter)

                self.qlabel_dac_offset_value[k] = Qt.QLabel('0 V')
                self.qlabel_dac_offset_value[k].setAlignment(Qt.Qt.AlignHCenter)


        # Create widgets to set the number of points for the graphs:
        self.qlabel_rawdata_rbw = Qt.QLabel('RBW: 100 kHz')
        self.qlabel_rawdata_pnts = Qt.QLabel('Points:')
        self.qedit_rawdata_length = Qt.QLineEdit('1.73e3')
        self.qedit_rawdata_length.setMaximumWidth(60)

        # Plot type select
        self.qlabel_adc_plot_type = Qt.QLabel('Plot type:')
        self.qcombo_adc_plottype = Qt.QComboBox()
        self.qcombo_adc_plottype.addItems(['Spectrum', 'Time: raw input', 'Time: Phase', 'Time: IQ', 'Time: IQ, synced'])



        # Input select
        self.qlabel_adc_plot_input = Qt.QLabel('Input:')
        self.qcombo_adc_plot = Qt.QComboBox()
        self.qcombo_adc_plot.addItems(['ADC 0', 'ADC 1', 'DAC 0', 'DAC 1', 'DAC 2'])
        self.qcombo_adc_plot.setCurrentIndex(self.selected_ADC)

        # Set a few global PyQtGraph settings before creating plots:
        pg.setConfigOption('leftButtonPan', False)
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        pg.setConfigOption('antialias', True)


        # Create the baseband IQ plot:
        #self.qplt_IQ = Qwt.QwtPlot()
        self.qplt_IQ = pg.PlotWidget()
        #self.qplt_IQ.move(50, 50)
        #self.qplt_IQ.resize(200, 200)
        self.qplt_IQ.setMinimumSize(50, 50)
        self.qplt_IQ.setMaximumSize(200, 200)
        self.qplt_IQ.setFixedSize(100, 100)
#        self.qplt_IQ.setsetHeightForWidth(True)
        # qPolicy = Qt.QSizePolicy(Qt.QSizePolicy.Preferred, Qt.QSizePolicy.Preferred)
        qPolicy = Qt.QSizePolicy(Qt.QSizePolicy.Fixed, Qt.QSizePolicy.Fixed)
        # qPolicy.setHeightForWidth(True)
        self.qplt_IQ.setSizePolicy(qPolicy)

       # self.qplt_IQ.setSizePolicy(Qt.QSizePolicy.setHeightForWidth(True))

#        print(self.qplt_IQ.sizeHint())
        #self.qplt_IQ.setTitle('', fill=None)
        # self.qplt_IQ.setTitle('Baseband IQ')

        #self.qplt_IQ.setCanvasBackground(Qt.Qt.white)
        #self.qplt_IQ.enableAxis(Qwt.QwtPlot.yLeft, False)
        #self.qplt_IQ.enableAxis(Qwt.QwtPlot.xBottom, False)
        self.qplt_IQ.hideAxis('left')
        self.qplt_IQ.hideAxis('bottom')

        self.lblplt_IQ_title = Qt.QLabel('Baseband IQ:')

        # Create the curves in the plot
        self.curve_IQ = self.qplt_IQ.getPlotItem().plot(pen = None, symbol = 'o', symbolPen=None, symbolSize=.75, symbolBrush='b')
        # self.curve_IQ = Qwt.QwtPlotCurve('Spectrum')
        # self.curve_IQ.attach(self.qplt_IQ)
        # self.curve_IQ.setPen(Qt.QPen(Qt.Qt.NoPen))
        # self.curve_IQ.setSymbol(Qwt.QwtSymbol(Qwt.QwtSymbol.Ellipse,
        #                             Qt.QBrush(Qt.Qt.blue),
        #                             Qt.QPen(Qt.Qt.blue),
        #                             Qt.QSize(3, 3)))
        # self.curve_IQ.setRenderHint(Qwt.QwtPlotItem.RenderAntialiased);

        # Create the frequency domain plot:
        # self.qplt_spc = Qwt.QwtPlot()
        # self.qplt_spc.setTitle('Spectrum')
        # self.qplt_spc.setCanvasBackground(Qt.Qt.white)
        self.plt_spc = pg.PlotWidget()


        # Create the curves in the plot
        # self.curve_spc = Qwt.QwtPlotCurve('Spectrum')
        # self.curve_spc.attach(self.qplt_spc)
        # self.curve_spc.setPen(Qt.QPen(Qt.Qt.blue))
        # self.curve_filter = Qwt.QwtPlotCurve('Spectrum')
        # self.curve_filter.attach(self.qplt_spc)
        # self.curve_filter.setPen(Qt.QPen(Qt.Qt.red))
        self.curve_spc = self.plt_spc.getPlotItem().plot(title='Spectrum', pen='b')
        self.curve_filter = self.plt_spc.getPlotItem().plot(pen='r')

        # Put all the widgets into a grid layout
        grid = QtGui.QGridLayout()
        grid.setSpacing(5)
        grid.addWidget(self.qlabel_adc_fill,        0, 0)
        grid.addWidget(self.qadc0_scale,            1, 0, 4, 1)
        grid.addWidget(self.qlabel_adc_fill_value,  5, 0, 1, 1)
        grid.addWidget(self.qlabel_baseband_snr,    0, 1)
        grid.addWidget(self.qthermo_baseband_snr,   1, 1, 4, 1)
        grid.addWidget(self.qlabel_baseband_snr_value,  5, 1, 1, 1)
#        grid.addWidget(self.qlabel_ddc0_error,      0, 2)
#        grid.addWidget(self.qddc0_error_scale,      1, 2, 4, 1)



        N_dac_controls = 0
        for k in range(3):
            if self.output_controls[k] == True:
                grid.addWidget(self.qlabel_dac_current[k],     0, 2+N_dac_controls)
                grid.addWidget(self.qthermo_dac_current[k],    1, 2+N_dac_controls, 4, 1)
                grid.addWidget(self.qlabel_dac_current_value[k],5, 2+N_dac_controls, 1, 1)
                grid.addWidget(self.qlabel_dac_offset[k],      0, 3+N_dac_controls)
                grid.addWidget(self.q_dac_offset[k],           1, 3+N_dac_controls, 4, 1)
#                grid.addWidget(self.qlabel_dac_offset_value[k],5, 3+N_dac_controls, 1, 1)
                grid.addWidget(self.qedit_dac_offset[k],5, 3+N_dac_controls, 1, 1)


                N_dac_controls = N_dac_controls + 2

        #FEATURE
        #if self.output_controls[0] == True:
            #grid.addWidget(self.qlabel_pwm0,       0, 3+N_dac_controls, 1, 1)
            #grid.addWidget(self.q_pwm0_value,      1, 3+N_dac_controls, 3, 1)
            #grid.addWidget(self.qlabel_pwm0_value, 4, 3+N_dac_controls, 1, 1)
            #N_dac_controls = N_dac_controls + 2


        grid.setRowStretch(1, 1)

        # The plots:
        qhoriz = Qt.QHBoxLayout()
        qhoriz.addWidget(self.lblplt_IQ_title)
        qhoriz.addWidget(self.qplt_IQ)
        qhoriz.addStretch(1)

        grid.addLayout(qhoriz,                      0, 2+N_dac_controls, 2, 2)

        #grid.addWidget(self.qplt_IQ,                0, 2+N_dac_controls, 2, 2)
        grid.addWidget(self.plt_spc,                0, 4+N_dac_controls, 6, 1)
        grid.setColumnStretch(4+N_dac_controls, 1)
#
        # The controls below the IQ plot:
        grid.addWidget(self.qlabel_adc_plot_input,  2, 2+N_dac_controls)
        grid.addWidget(self.qlabel_adc_plot_type,   3, 2+N_dac_controls)
        grid.addWidget(self.qlabel_rawdata_pnts,    4, 2+N_dac_controls)
        grid.addWidget(self.qlabel_rawdata_rbw,     5, 3+N_dac_controls)

        grid.addWidget(self.qcombo_adc_plot,        2, 3+N_dac_controls)
        grid.addWidget(self.qcombo_adc_plottype,    3, 3+N_dac_controls)
        grid.addWidget(self.qedit_rawdata_length,   4, 3+N_dac_controls)

#        grid.addItem(spacerItem, 9, 0, 1, 2)

        for k in range(3):
            if self.output_controls[k] == True:
                grid.addWidget(self.qchk_dac_limited[k],0, 2+N_dac_controls)

        self.qgroupbox_diagnostics.setLayout(grid)
        self.qgroupbox_diagnostics.setPalette(PalNormal)
        self.qgroupbox_diagnostics.setAutoFillBackground(True)


        ######################################################################
        # Create the controls for the loop filters
        ######################################################################
        self.qgroupbox_loop_filters = Qt.QGroupBox('Loop filters', self)

        hbox = Qt.QHBoxLayout()
        self.qloop_filters = {}

        for k in range(3):
            if self.output_controls[k] == True:
                if k == 0 or k == 1:
    #                print('XEM_GUI_MainWindow(): About to call LoopFiltersUI()')
                    self.qloop_filters[k] = LoopFiltersUI(self.sl, k, bDisplayLockChkBox=False)


                    hbox.addWidget(self.qloop_filters[k])
                    #self.qloop_filters[k].show()

                # elif k == 1:
                #     self.qloop_filters[k] = LoopFiltersUI_DAC1_and_DAC2(self.sl, k, self.sl.pll[k])
                #     hbox.addWidget(self.qloop_filters[k])
                #     self.qloop_filters[k].show()



        self.qgroupbox_loop_filters.setLayout(hbox)

#        self.qgroupbox_loop_filters.setLayout(grid)
        self.qgroupbox_loop_filters.setPalette(PalNormal)
        self.qgroupbox_loop_filters.setAutoFillBackground(True)

        ######################################################################
        # Phase noise analysis
        ######################################################################
        self.qgroupbox_phasenoise = Qt.QGroupBox('Phase noise (all computed from DDC output)', self)

        # Selector for the plot type (phase or freq noise)
#        self.qlabel_ddc_plot_select = Qt.QLabel('Plot type:')
        self.qcombo_ddc_plot = Qt.QComboBox()
        self.qcombo_ddc_plot.addItem('Freq')
        self.qcombo_ddc_plot.addItem('Phase')
        self.qcombo_ddc_plot.addItem('Freq: time domain')
        self.qcombo_ddc_plot.addItem('Phase: time domain')
        self.qcombo_ddc_plot.setCurrentIndex(1)

        # Create widgets to set the number of points for the DDC graphs:
        self.qlabel_ddc_rbw = Qt.QLabel('RBW: 100 kHz; Points:')
        self.qedit_ddc_length = Qt.QLineEdit('32.768e3') # this used to be 3e5 in the Dave Leibrant box version, but was changed to 16e3 due to RedPitaya memory limitations
        self.qedit_ddc_length.setMaximumWidth(60)

        # Create widgets to set the higher frequency of the integration:
        self.qlabel_cumul_integral = Qt.QLabel('Integration\nlimit [Hz]:')
        self.qedit_cumul_integral = Qt.QLineEdit('5e6')
        self.qedit_cumul_integral.setMaximumWidth(60)

        # Display mean frequency error:
        self.qlbl_mean_freq_error = Qt.QLabel('Mean freq error = 0 MHz')

        # Checkbox to enable faster updates of the phase noise plot:
        self.qchk_phase_noise_fast_updates = Qt.QCheckBox('Faster updates')
        self.qchk_phase_noise_fast_updates.setChecked(False)

        # X and Y limits for the plot:
        self.qlbl_xlims = Qt.QLabel('Xmin, Xmax')
        self.qedit_xlims = Qt.QLineEdit('3e3, 5e6')
        self.qedit_xlims.setMaximumWidth(60)
        self.qlbl_ylims = Qt.QLabel('Ymin, Ymax')
        self.qedit_ylims = Qt.QLineEdit('-100, -30')
        self.qedit_ylims.setMaximumWidth(60)

        # Averaging controls: # Averages (1=off)
        self.qlbl_spc_averaging = Qt.QLabel('# Averages\n(1=off)')
        self.qedit_spc_averaging = Qt.QLineEdit('1')
        self.qedit_spc_averaging.setMaximumWidth(60)

        # Create the frequency domain plot for the DDC0
        self.qplt_DDC0_spc = pg.PlotWidget()
        self.qplt_DDC0_spc.setTitle('Freq noise PSD')
        #self.qplt_DDC0_spc.setCanvasBackground(Qt.Qt.white)
        #self.qplt_DDC0_spc.setAxisScaleEngine(Qwt.QwtPlot.xBottom, Qwt.QwtLog10ScaleEngine())
        self.qplt_DDC0_spc.getPlotItem().setLogMode(x=True)
        self.qplt_DDC0_spc.setYRange(-60, 60)

        #self.qplt_DDC0_spc.enableAxis(Qwt.QwtPlot.yRight)
        self.qplt_DDC0_spc.setLabel('bottom', 'Frequency [Hz]')
        self.qplt_DDC0_spc.setLabel('left', 'PSD [dB Hz^2/Hz]')
        #self.qplt_DDC0_spc.setLabel('right', 'Phase [rad]')

        # create the right-side axis:
        p1 = self.qplt_DDC0_spc.getPlotItem()
        self.qplt_DDC0_spc_right_viewbox = pg.ViewBox()
        #self.qplt_DDC0_spc_right_viewbox.setLogMode(x=True)
        p1.showAxis('right')
        p1.scene().addItem(self.qplt_DDC0_spc_right_viewbox)
        p1.getAxis('right').linkToView(self.qplt_DDC0_spc_right_viewbox)
        self.qplt_DDC0_spc_right_viewbox.setXLink(p1)
        p1.getAxis('right').setLabel('Phase [rad]')

        self.updatePhaseNoiseViews()
        p1.vb.sigResized.connect(self.updatePhaseNoiseViews)


        #self.qplt_DDC0_spc.showGrid(x=True, y=True)
        #plot_grid = Qwt.QwtPlotGrid()
        #plot_grid.setMajPen(Qt.QPen(Qt.Qt.black, 0, Qt.Qt.DotLine))
        #plot_grid.attach(self.qplt_DDC0_spc)

        # Create the curve in the plot
        self.curve_DDC0_spc = self.qplt_DDC0_spc.getPlotItem().plot(title='Phase noise PSD', pen='b')
        #self.curve_DDC0_spc.attach(self.qplt_DDC0_spc)
        #self.curve_DDC0_spc.setPen(Qt.QPen(Qt.Qt.blue))

        self.curve_DDC0_spc_amplitude_noise = self.qplt_DDC0_spc.getPlotItem().plot(pen='r')
        #self.curve_DDC0_spc_amplitude_noise.attach(self.qplt_DDC0_spc)
        #self.curve_DDC0_spc_amplitude_noise.setPen(Qt.QPen(Qt.Qt.red))


        #self.curve_DDC0_cumul_phase = pg.PlotCurveItem(pen='g')
        self.curve_DDC0_cumul_phase = pg.PlotDataItem(pen='k')
        self.curve_DDC0_cumul_phase.setLogMode(True, False)
        self.qplt_DDC0_spc_right_viewbox.addItem(self.curve_DDC0_cumul_phase)

        #self.curve_DDC0_cumul_phase = self.qplt_DDC0_spc_right_viewbox.getPlotItem().plot(pen='k')
        #self.curve_DDC0_cumul_phase.attach(self.qplt_DDC0_spc)
        #self.curve_DDC0_cumul_phase.setPen(Qt.QPen(Qt.Qt.black))
        # self.curve_DDC0_cumul_phase.setYAxis(Qwt.QwtPlot.yRight)

        self.curve_DDC0_spc_avg = self.qplt_DDC0_spc.getPlotItem().plot(pen='g')
        #self.curve_DDC0_spc_avg.attach(self.qplt_DDC0_spc)
        #self.curve_DDC0_spc_avg.setPen(Qt.QPen(Qt.Qt.darkGreen))



        # Put all the widgets into a grid layout
        grid = Qt.QGridLayout()
#        grid.addWidget(self.qlabel_ddc_plot_select, 0, 0)
        grid.addWidget(self.qcombo_ddc_plot, 0, 0, 1, 2)
        grid.addWidget(self.qlabel_ddc_rbw, 1, 0)
        grid.addWidget(self.qedit_ddc_length, 1, 1)
        grid.addWidget(self.qlabel_cumul_integral, 2, 0)
        grid.addWidget(self.qedit_cumul_integral, 2, 1)

        grid.addWidget(self.qlbl_xlims, 3, 0, 1, 1)
        grid.addWidget(self.qedit_xlims, 3, 1, 1, 1)
        grid.addWidget(self.qlbl_ylims, 4, 0, 1, 1)
        grid.addWidget(self.qedit_ylims, 4, 1, 1, 1)

        grid.addWidget(self.qlbl_spc_averaging, 5, 0, 1, 1)
        grid.addWidget(self.qedit_spc_averaging, 5, 1, 1, 1)



        grid.addWidget(self.qchk_phase_noise_fast_updates, 6, 0, 1, 2)
        grid.addWidget(self.qlbl_mean_freq_error, 7, 0, 1, 2)

        grid.addWidget(Qt.QLabel(''), 8, 0)


        grid.addWidget(self.qplt_DDC0_spc, 0, 2, 9, 1)
        grid.setRowStretch(7, 1)
        grid.setColumnStretch(2, 1)


        self.qgroupbox_phasenoise.setLayout(grid)
        self.qgroupbox_phasenoise.setPalette(PalNormal)
        self.qgroupbox_phasenoise.setAutoFillBackground(True)

        ######################################################################
        # Layout for the whole form:
        ######################################################################
        grid = Qt.QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(self.qgroupbox_settings,          0, 0, 1, 0)
        grid.addWidget(self.qgroupbox_diagnostics,       1, 0, 1, 0)
        grid.addWidget(self.qgroupbox_phasenoise,        2, 0, 1, 1)
        grid.addWidget(self.qgroupbox_loop_filters,      3, 0, 1, 1)
#        grid.setRowStretch(2, 1)

        self.setLayout(grid)


        # Adjust the size and position of the window
#        self.resize(940, 1080-100+30)
#        self.center()
        #self.setGeometry(18, 40, 950, 1010)
        #self.setGeometry(0, 0, 750, 1000)
        self.setWindowTitle(self.strTitle)
        #self.show()

#    def resizeEvent(self, event):
#        print('resizeEvent')
#        print(self.geometry())

    ## Handle view resizing for the phase noise plot (since we need to manual link the left and right side axes)
    def updatePhaseNoiseViews(self):
        ## view has resized; update auxiliary views to match
        p1 = self.qplt_DDC0_spc.getPlotItem()

        self.qplt_DDC0_spc_right_viewbox.setGeometry(p1.vb.sceneBoundingRect())

        ## need to re-update linked axes since this was called
        ## incorrectly while views had different shapes.
        ## (probably this should be handled in ViewBox.resizeEvent)
        self.qplt_DDC0_spc_right_viewbox.linkedViewChanged(p1.vb, self.qplt_DDC0_spc_right_viewbox.XAxis)



    def loadParameters(self):

        # Update the values in the UI to reflect the internal values:

        # Get values from xml file
        for k in range(3):
            if self.output_controls[k] == True:
#                print('XEM_GUI_MainWindow(): About to call loadParameters()')
                if k < 2: # For qllop_filter 0 and 1
                    #print('before calling self.qloop_filters[k].loadParameters(self.sp)')
                    self.qloop_filters[k].loadParameters(self.sp)                               # Get values from xml file for loop_filters
                    #print('after calling self.qloop_filters[k].loadParameters(self.sp)')
                    self.qchk_lock.setChecked(self.qloop_filters[k].qchk_lock.isChecked()) # update the qchk_lock in this widget with the value loaded from sp

                # Get dac gain from the system parameters object and set it in the UI:
                strDAC = 'DAC{:01d}'.format(k)
                str_VCO_gain = (self.sp.getValue('VCO_gain', strDAC))
                # print('before calling self.qedit_vco_gain[k].setText(str_VCO_gain)')
                self.qedit_vco_gain[k].blockSignals(True)
                self.qedit_vco_gain[k].setText(str_VCO_gain)
                self.qedit_vco_gain[k].blockSignals(False)
                # print('after calling self.qedit_vco_gain[k].setText(str_VCO_gain)')

                # Output offsets values:
                output_offset_in_volts = float(self.sp.getValue('Output_offset_in_volts', strDAC))

                # Scale this to the correct units for the output offset slider:
                slider_units = output_offset_in_volts / self.sl.dev.DAC_V_INT
                #print('calling dac offset slider setValue()')
                self.q_dac_offset[k].blockSignals(True)
                self.q_dac_offset[k].setValue(slider_units)
                self.q_dac_offset[k].blockSignals(False)
                #print('done calling dac offset slider setValue()')

        if self.output_controls[0] == True:

            self.PWM0_standard = float(self.sp.getValue('PWM0_settings', 'standard'));
            self.PWM0_levels   = int(self.sp.getValue('PWM0_settings', 'levels'));
            self.PWM0_default  = float(self.sp.getValue('PWM0_settings', 'default'));
            self.PWM0_min      = float(self.sp.getValue('PWM0_settings', 'minval'));
            self.PWM0_max      = float(self.sp.getValue('PWM0_settings', 'maxval'));

            slider_units = (self.PWM0_default-self.PWM0_min)/(self.PWM0_max-self.PWM0_min) * 1e6
            self.q_pwm0_value.blockSignals(True)
            self.q_pwm0_value.setValue(slider_units)
            self.q_pwm0_value.blockSignals(False)


        # Get ddc reference frequency from the system parameters object and set it in the UI:
        strDDC = 'DDC{:01d}'.format(self.selected_ADC)
        str_ref_freq = (self.sp.getValue('Reference_frequency', strDDC))
        self.qedit_ref_freq.setText(str_ref_freq)
        self.qedit_ref_freq.reset_my_color()


        #print('done loadParameters()')
        return

    def center(self):

        qr = self.frameGeometry()
        cp = QtGui.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
#        self.move(qr.topLeft())
        self.move(QtGui.QDesktopWidget().availableGeometry().topLeft() + Qt.QPoint(10, 10))



    def initSL(self):
        # Old function to start the GUI communication
#        self.sl = SuperLaserLand_JD2()
#        self.sl.open()
        print("initSL()")

        self.loadParameters()

        bUpdateFPGA = True
        # Send values to FPGA
        if bUpdateFPGA == True:
            self.setVCOFreq_event()
            self.setVCOGain_event()
            # self.setDACOffset_event()  # not needed because setVCOGain_event calls it anyway
            self.chkLockClickedEvent()
            if self.output_controls[0] == True:
                self.setPWM0_event()

#        self.setFLL0_event()
#        self.setPLL0_event()


#        self.timerID = self.startTimer(500 * 1/1)
        self.timerID = 0
        self.refreshChk_event()

        # Start the timer which reads the dither:
#        self.timerIDDither = Qt
        self.timerIDDither = Qt.QTimer(self)
        self.timerIDDither.timeout.connect(self.timerDitherEvent)
        self.timerIDDither.start(100)   # 100 ms readout delay, increased to 1000 ms for debugging
        # print "Warning! Increased self.timerIDDither.start(100) to 3000 for debugging."
#        self.timerDitherEvent()


        self.displayDAC()   # This populates the current DAC values with the actual value
#        self.timerEvent(0)  # run the event handler once right away, to populate the rest of the window

        print("initSL() done")

    def closeEvent(self, event):
        # from http://stackoverflow.com/questions/1414781/prompt-on-exit-in-pyqt-application
#        quit_msg = "Are you sure you want to exit the program?"
#        reply = QtGui.QMessageBox.question(self, 'Message',
#                         quit_msg, QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
#
#        print(event)
#
#        if reply == QtGui.QMessageBox.Yes:
#            event.accept()
#            app = Qt.QApplication.instance()
#            app.closeAllWindows()
##            app.
#        else:
#            event.ignore()
        return

    def timerDitherEvent(self):
        if self.isVisible():
            # print('timerDitherEvent')

            #print("timerDitherEvent: self.qthermo_baseband_snr.sizeHint() = ", self.qthermo_baseband_snr.sizeHint())

            # Check if the sl object exists: otherwise this timer will keep throwing exceptions, filling up the console messages
            # and preventing us form seeing the real cause
            if not hasattr(self, 'sl'):
                print('SL object does not exist anymore. disabling timer in timerDitherEvent')
                self.timerIDDither.stop()
                return

            start_time = time.clock()
            for k in range(3):
                if self.output_controls[k]:
                    if self.sl.dither_enable[k] == False:
                        self.qlabel_detected_vco_gain[k].setText('off')
                        self.qlabel_detected_vco_gain[k].setStyleSheet("color: white; background-color: black")

                    else:
                        samples = self.sl.ditherRead(2, k)
                        # There is an implicit (-) sign because the DDC has to shift the frequency to 0.
                        # This means that the detected gain will be positive when the VCO sign checkbox is correctly tuned
                        samples = -samples
                        samples = np.mean(samples)
    #                    print('Dither output: %d' % np.real(samples))
                        # np.mean() returns a numpy.float, but the conversions functions expect an ndarray
        #                print(type(samples))
                        samples = np.ndarray((1,), dtype=np.float, buffer=samples)
        #                print(type(samples))

        #                rep1 = self.sl.scaleDitherResultsToHz(np.real(samples), k)
        #                rep2 = self.sl.scaleDitherResultsToHzPerVolts(np.imag(samples), k)
        #                print('Real part = %f Hz' % self.sl.scaleDitherResultsToHz(np.real(samples[0]), k))
        #                print('Real part = %f Hz/V' % self.sl.scaleDitherResultsToHzPerVolts(np.real(samples[0]), k))

                        # TODO: fancier things with the real and imaginary part, to try to detect invalid readings?  Is this necessary?
                        # TODO: Compare many different readings to try to sort out any incorrect ones?
                        VCO_detected_gain_in_Hz_per_Volts = self.sl.scaleDitherResultsToHzPerVolts(samples, k)
                        self.VCO_detected_gain_in_Hz_per_Volts[k] = VCO_detected_gain_in_Hz_per_Volts
                        self.qlabel_detected_vco_gain[k].setText('%.1e' % VCO_detected_gain_in_Hz_per_Volts)
                        elapsed_time = time.clock() - start_time
        #                print('Elapsed time (timerDitherEvent) = %f ms' % (1000*elapsed_time))

                        # If the detected gain is negative, the loop will be unstable when closed, so we switch to a red background so that the user can flip the sign
                        if VCO_detected_gain_in_Hz_per_Volts > 0:
    #                        self.qedit_fi.setStyleSheet("background-color: %s" % Qt.QColor(Qt.Qt.white).name())
                            self.qlabel_detected_vco_gain[k].setStyleSheet("color: white; background-color: green")
                        else:
                            # red background
    #                        self.qlabel_detected_vco_gain[k].setStyleSheet("color: white; background-color: %s" % Qt.QColor(Qt.Qt.red).name())
                            self.qlabel_detected_vco_gain[k].setStyleSheet("color: white; background-color: red")

    def timerEvent(self, e):
        if self.isVisible():
            # Handle the LEDs display
            (LED_G0, LED_R0, LED_G1, LED_R1, LED_G2, LED_R2) = self.sl.readLEDs()
            #print ('%d, %d, %d, %d, %d, %d' % (LED_G0, LED_R0, LED_G1, LED_R1, LED_G2, LED_R2))
            self.displayAnalyzer()
            self.displayDAC()

            if self.display_phase == 0 or self.qchk_phase_noise_fast_updates.isChecked():
                self.displayDDC()

            self.display_phase = self.display_phase + 1
            if self.display_phase > 5:
                self.display_phase = 0

            self.qlabel_refreshrate.setText('%.0f ms' % (1000*(time.clock() - self.last_refresh)))
            self.last_refresh = time.clock()

    # timerEvent()
    def displayDAC(self):

        # Check if another function is currently using the DDR2 logger:
        if self.sl.bDDR2InUse:
            print('displayDAC(): DDR2 logger in use, cannot get data from dac')
            return
        # Block access to the DDR2 Logger to any other function until we are done:
        self.sl.bDDR2InUse = True

        # For now: we grab the smallest chunk of points from the output (so as to not use too much time to refresh)
        # and display the current average:
        for k in range(3):
            if self.output_controls[k]:
                # Read from DAC #k
                start_time = time.clock()

                # For the USB bug, changed this from 64 to 256
                N_points = 256   # I think that this is the smallest chunk we can read at a time with the current design of the DDR2 controller
                if k == 0:
                    self.sl.setup_DAC0_write(N_points)
                elif k == 1:
                    self.sl.setup_DAC1_write(N_points)
                elif k == 2:
                    self.sl.setup_DAC2_write(N_points)

                self.sl.trigger_write()
                self.sl.wait_for_write()
                samples_out = self.sl.read_dac_samples_from_DDR2()


                if k == 0 or k == 1:
                    samples_out = samples_out.astype(dtype=np.float)
                elif k == 2:
                    samples_out = samples_out.astype(dtype=np.float)*2**4  # The DAC is actually 20 bits, but only the 16 MSBs are sent to the DDR2 logger, which amounts to dividing the DAC counts by 2**4


                try:
                    VCO_gain_in_Hz_per_Volts = float(self.qedit_vco_gain[k].text())
                except:
                    VCO_gain_in_Hz_per_Volts = 1e9

                # Update the display:
                # For the USB bug, compute the mean from the last points
                current_output_in_volts = self.sl.dev.DAC_V_INT * np.mean(samples_out[128:256])
                current_output_in_hz = current_output_in_volts * VCO_gain_in_Hz_per_Volts
                self.qthermo_dac_current[k].setValue(current_output_in_volts)
                self.qlabel_dac_current_value[k].setText('{:.4f} V\n{:.2f} MHz'.format(current_output_in_volts, current_output_in_hz/1e6))

                elapsed_time = time.clock() - start_time
                if self.bDisplayTiming == True:
                    print('Elapsed time (displayDAC) = %f ms' % (1000*elapsed_time))

        # Signal to other functions that they can use the DDR2 logger
        self.sl.bDDR2InUse = False

    def displayDDC(self):

        # Check if another function is currently using the DDR2 logger:
        if self.sl.bDDR2InUse:
            print('displayDDC(): DDR2 logger in use, cannot get data from adc')
            return
        # Block access to the DDR2 Logger to any other function until we are done:
        self.sl.bDDR2InUse = True

        # Read from DDC0
        try:
            N_points = int(float(self.qedit_ddc_length.text()))
        except:
            N_points = 100e3
        if N_points < 64:
            N_points = 64

        start_time = time.clock()
        if self.selected_ADC == 0:
            self.sl.setup_DDC0_write(N_points)
        elif self.selected_ADC == 1:
            self.sl.setup_DDC1_write(N_points)
        self.sl.trigger_write()
        self.sl.wait_for_write()
        if self.bDisplayTiming == True:
            print('Elapsed time (setup write) = %f' % (time.clock()-start_time))
        start_time = time.clock()
        inst_freq = self.sl.read_ddc_samples_from_DDR2()
        self.inst_freq = inst_freq

        if self.bDisplayTiming == True:
            print('Elapsed time (communication) = %f' % (time.clock()-start_time))


        # Signal to other functions that they can use the DDR2 logger
        self.sl.bDDR2InUse = False



#            self.qddc0_error_scale.setValue(np.mean(inst_freq)/1e6)
#            print('mean freq error = %f MHz, raw code = %f' % (np.mean(inst_freq)/1e6, np.mean(inst_freq)*2**10 / self.sl.dev.ADC_CLK_Hz*4))
        self.qlbl_mean_freq_error.setText('Freq error: %.2f MHz' % (np.mean(inst_freq)/1e6))

        # Compute the spectrum:
        # We first perform decimation on the data since we don't have useful information above the cut-off frequency anyway:
        start_time = time.clock()
        N_decimation = 10
        fs_new = self.sl.dev.ADC_CLK_Hz/N_decimation
        inst_freq_decimated = decimate(inst_freq, N_decimation)
#            inst_freq_decimated = decimate(inst_freq, N_decimation, zero_phase=False)

#            inst_freq_decimated = inst_freq
#            fs_new = self.sl.dev.ADC_CLK_Hz

        # For debugging: we want to check
#            inst_freq_decimated = np.random.randn(100e3)
#            print('Data std dev = %f Hz' % np.std(inst_freq_decimated))
#            print('Data variance = %f Hz^2' % np.var(inst_freq_decimated))
        if self.bDisplayTiming == True:
            print('Elapsed time (decimation) = %f' % (time.clock()-start_time))
        start_time = time.clock()

        # Compute the spectrum of the decimated signal:
        start_time = time.clock()
        N_fft = 2**(int(np.ceil(np.log2(len(inst_freq_decimated)))))
        frequency_axis = np.linspace(0, (N_fft-1)/float(N_fft)*fs_new, N_fft)
        last_index_shown = int(np.round(len(frequency_axis)/2))
        window_function = np.blackman(len(inst_freq_decimated))
        window_NEB = np.sum((window_function/np.sum(window_function))**2) * fs_new;
#            print('window_NEB = %f Hz' % window_NEB)

        spc = np.fft.fft(detrend(inst_freq_decimated) * window_function, N_fft)
        spc = np.real(spc*np.conj(spc))/(sum(window_function)**2) # Spectrum is now scaled in power (Hz^2 per bin)
        # Scale the spectrum to be a single-sided power spectral density in Hz^2/Hz:
        spc[1:last_index_shown] = 2*spc[1:last_index_shown] / window_NEB

#            # Compute the running average:
        # Compute spectrum averaging with exponential smoothing (simple first-order IIR filter)
        try:
            n_spc_avg = int(round(float(self.qedit_spc_averaging.text())))
            if n_spc_avg > 1.:
                self.bAveragePhaseNoise = True
                self.N_spc_average = n_spc_avg
            else:
                self.bAveragePhaseNoise = False
                self.N_spc_average = 1.
        except:
            n_spc_avg = 1.
            self.bAveragePhaseNoise = False

        if self.bAveragePhaseNoise:
#                print('self.N_spc_average = %d' % self.N_spc_average)
            filter_alpha = np.exp(-1./self.N_spc_average)
            try:
                # if this is not the first time we are called with averaging enabled, we run the filter:
                if self.bAveragePhaseNoiseLast:
                    self.spc_running_sum = filter_alpha * self.spc_running_sum +  + (1-filter_alpha)*spc
                else:
                    # this is the first time that we are called with averaging enabled, so we reset the current state
                    self.spc_running_sum = spc
            except:
                # This is the first time that we are called:
                self.spc_running_sum = spc


        self.bAveragePhaseNoiseLast = self.bAveragePhaseNoise

#            print('Freq noise PSD: %e Hz^2/Hz' % (np.mean(spc[1:last_index_shown])))
        self.freq_noise_psd = spc[1:last_index_shown]
        self.freq_noise_axis = frequency_axis[1:last_index_shown]

#            spc = np.abs(spc)

        if self.bDisplayTiming == True:
            print('Elapsed time (FFT) = %f' % (time.clock()-start_time))


        try:
            f_limits = self.qedit_xlims.text()
            f_limits = f_limits.split(',')
            f_limits = (float(f_limits[0]), float(f_limits[1]))
        except:
            f_limits = (frequency_axis[1], frequency_axis[last_index_shown])

        try:
            y_limits = self.qedit_ylims.text()
            y_limits = y_limits.split(',')
            y_limits = (float(y_limits[0]), float(y_limits[1]))
        except:
            y_limits = (-140, 60)

        #----------------------------------------------------------------------
        #--- Plot Frequency Noise Spectrum
        #
        if self.qcombo_ddc_plot.currentIndex() == 0:
            spc = 10*np.log10(spc + 1e-20)
            self.curve_DDC0_spc.setData(frequency_axis[1:last_index_shown], spc[1:last_index_shown])
            if self.bAveragePhaseNoise:
                self.curve_DDC0_spc_avg.setData(frequency_axis[1:last_index_shown], 10*np.log10(self.spc_running_sum[1:last_index_shown] + 1e-20))
                self.curve_DDC0_spc_avg.setVisible(True)
            else:
                self.curve_DDC0_spc_avg.setVisible(False)
            self.qplt_DDC0_spc.setTitle('Freq noise PSD')
            self.qplt_DDC0_spc.setLabel('left', 'PSD [dB Hz^2/Hz]')
            self.qplt_DDC0_spc.enableAutoRange(y=False)
            self.qplt_DDC0_spc.setYRange(spc[1:last_index_shown].min()-10, spc[1:last_index_shown].max()+10)
            self.qplt_DDC0_spc.showGrid(y=False)
            #self.qplt_DDC0_spc.setAxisScale(Qwt.QwtPlot.xBottom, frequency_axis[1], frequency_axis[last_index_shown])
            self.qplt_DDC0_spc.getPlotItem().setLogMode(x=True)
            self.qplt_DDC0_spc.setXRange(np.log10(f_limits[0]), np.log10(f_limits[1]))
            #self.qplt_DDC0_spc.setAxisScaleEngine(Qwt.QwtPlot.xBottom, Qwt.QwtLog10ScaleEngine())
            self.qplt_DDC0_spc.setLabel('bottom', 'Frequency [Hz]')
            self.curve_DDC0_cumul_phase.setVisible(False)

        #----------------------------------------------------------------------
        #--- Plot Display Phase Noise Spectrum
        #
        elif self.qcombo_ddc_plot.currentIndex() == 1:
            # Compute the phase noise:
            phase_noise = 10*np.log10(spc + 1e-20) - 20*np.log10(frequency_axis)

            # Comput the time-domain standard deviation:
            inst_phase = np.cumsum(inst_freq*2*np.pi/self.sl.dev.ADC_CLK_Hz)
            phasenoise_stddev = np.std(inst_phase)

            # Display the cumulative integral of the phase noise:
            # Select desired frequency range:
            try:
                integration_higher_bound = float(self.qedit_cumul_integral.text())
            except:
                integration_higher_bound = 1e6
            if integration_higher_bound > fs_new/2:
                integration_higher_bound = fs_new/2
            if integration_higher_bound <= 2/len(spc)*fs_new:
                integration_higher_bound = 2/len(spc)*fs_new
            integration_higher_index = int(round(integration_higher_bound/fs_new*len(spc)))
#                print('integration up to %d out of %d' % (integration_higher_index, len(spc)))
            frequency_axis_integral = frequency_axis[1:integration_higher_index]

            # Integrate the phase noise PSD, from the highest frequency to the lowest
            if self.bAveragePhaseNoise:
                phase_psd = self.spc_running_sum[1:integration_higher_index] / frequency_axis_integral**2
                cumul_int = np.flipud(np.cumsum(np.flipud(phase_psd))) * np.mean(np.diff(frequency_axis_integral))
            else:
                phase_psd = spc[1:integration_higher_index] / frequency_axis_integral**2
                cumul_int = np.flipud(np.cumsum(np.flipud(phase_psd))) * np.mean(np.diff(frequency_axis_integral))

            integrated_noise = np.max(np.sqrt(cumul_int))

            # Show results
            self.curve_DDC0_cumul_phase.setData(frequency_axis_integral, np.sqrt(cumul_int))
            self.curve_DDC0_cumul_phase.setVisible(True)
            self.qplt_DDC0_spc_right_viewbox.setYRange(0, np.max(np.sqrt(cumul_int))*1.5)
            #self.qplt_DDC0_spc_right_viewbox.setXRange(0, 2*2*np.pi)

            # Display the phase noise (equal to 1/f^2 times the frequency noise PSD)
            self.curve_DDC0_spc.setData(frequency_axis[1:last_index_shown], phase_noise[1:last_index_shown])
            if self.bAveragePhaseNoise:
                phase_noise_avg = 10*np.log10(self.spc_running_sum + 1e-20) - 20*np.log10(frequency_axis)
                self.curve_DDC0_spc_avg.setData(frequency_axis[1:last_index_shown], phase_noise_avg[1:last_index_shown])
                self.curve_DDC0_spc_avg.setVisible(True)
            else:
                self.curve_DDC0_spc_avg.setVisible(False)
            self.qplt_DDC0_spc.setXRange(f_limits[0], f_limits[1])
            self.qplt_DDC0_spc.setTitle('Phase noise PSD, std dev = {:.2e} radrms; Integrated PSD = {:.2e} radrms'.format(phasenoise_stddev, integrated_noise))
            self.qplt_DDC0_spc.setLabel('left', 'PSD [dBc/Hz]')
            #self.qplt_DDC0_spc.setYRange(y_limits[0], y_limits[1])
            self.qplt_DDC0_spc.enableAutoRange(y=False)
            if self.bAveragePhaseNoise:
                self.qplt_DDC0_spc.setYRange(phase_noise_avg[1:last_index_shown].mean()-10, phase_noise_avg[1:last_index_shown].max()+10, padding=0)
            else:
                self.qplt_DDC0_spc.setYRange(phase_noise[1:last_index_shown].mean()-10, phase_noise[1:last_index_shown].max()+10, padding=0)
            self.qplt_DDC0_spc.showGrid(y=False)
            #self.qplt_DDC0_spc.setAxisScaleEngine(Qwt.QwtPlot.xBottom, Qwt.QwtLog10ScaleEngine())
            self.qplt_DDC0_spc.getPlotItem().setLogMode(x=True)

            self.qplt_DDC0_spc.setXRange(np.log10(f_limits[0]), np.log10(f_limits[1]*5./6.0))   # the scaling is because the widget doesn't seem to use the exact values that we pass...
            self.qplt_DDC0_spc.setLabel('bottom', 'Frequency [Hz]')


        #----------------------------------------------------------------------
        #--- Plot Frequency Error
        #
        elif self.qcombo_ddc_plot.currentIndex() == 2:
            # Display the raw, time-domain instantaneous frequency output by the DDC block, mostly for debugging:

            time_axis = np.arange(0, len(inst_freq))/self.sl.dev.ADC_CLK_Hz

            freq_error_stddev = np.std(inst_freq)
            freq_error_mean = np.mean(inst_freq)

            self.curve_DDC0_spc.setData(time_axis, inst_freq)
            self.curve_DDC0_spc_avg.setVisible(False)
            self.curve_DDC0_cumul_phase.setVisible(False)
            self.qplt_DDC0_spc.setTitle('Instantaneous frequency error = {:.3g} Hz; stddev = {:.3g} Hz'.format(freq_error_mean, freq_error_stddev))
            self.qplt_DDC0_spc.setLabel('left', 'Freq [Hz]')
            self.qplt_DDC0_spc.setLabel('bottom', 'Time [s]')
#                self.qplt_DDC0_spc.setAxisScale(Qwt.QwtPlot.yLeft, -self.sl.dev.ADC_CLK_Hz/2, self.sl.dev.ADC_CLK_Hz/2)
            #self.qplt_DDC0_spc.setYRange(np.min(inst_freq), np.max(inst_freq))
            self.qplt_DDC0_spc.enableAutoRange(y=True)
            self.qplt_DDC0_spc.showGrid(y=False)
            #self.qplt_DDC0_spc.setAxisScaleEngine(Qwt.QwtPlot.xBottom, Qwt.QwtLinearScaleEngine())
            self.qplt_DDC0_spc.getPlotItem().setLogMode(x=False)
            self.qplt_DDC0_spc.setXRange(time_axis[0], time_axis[-1])

        #----------------------------------------------------------------------
        #--- Plot Phase Error
        #
        elif self.qcombo_ddc_plot.currentIndex() == 3:
            # Display the time-domain instantaneous phase output by the DDC block (computed by integrating the frequency), mostly for debugging:

            time_axis = np.arange(0, len(inst_freq))/self.sl.dev.ADC_CLK_Hz
            inst_phase = np.cumsum(inst_freq*2*np.pi/self.sl.dev.ADC_CLK_Hz)

            # Compute the phase noise time-domain standard deviation:
            phasenoise_stddev = np.std(inst_phase)
            phasenoise_mean = np.mean(inst_phase)

            self.curve_DDC0_spc.setData(time_axis, inst_phase)
            self.curve_DDC0_spc_avg.setVisible(False)
            self.curve_DDC0_cumul_phase.setVisible(False)
            self.qplt_DDC0_spc.setTitle('Instantaneous phase error = {:.3g} rad; std dev = {:.3g} radrms'.format(phasenoise_mean, phasenoise_stddev))
            self.qplt_DDC0_spc.setLabel('left', 'Phase [rad]')
            self.qplt_DDC0_spc.setLabel('bottom', 'Time [s]')
#                self.qplt_DDC0_spc.setAxisScale(Qwt.QwtPlot.yLeft, -self.sl.dev.ADC_CLK_Hz/2, self.sl.dev.ADC_CLK_Hz/2)
            self.qplt_DDC0_spc.setYRange(np.min(inst_phase), np.max(inst_phase))
            #self.qplt_DDC0_spc.enableAutoRange(y=True)
            self.qplt_DDC0_spc.showGrid(y=False)
            #self.qplt_DDC0_spc.setAxisScaleEngine(Qwt.QwtPlot.xBottom, Qwt.QwtLinearScaleEngine())
            self.qplt_DDC0_spc.getPlotItem().setLogMode(x=False)
            self.qplt_DDC0_spc.setXRange(time_axis[0], time_axis[-1])
#                self.qplt_DDC0_spc.setAxisScale(Qwt.QwtPlot.yLeft, -3.14, 3.14)
#                print "debug warning: phase noise plot scaled to +/- pi"

#            # Display the un-decimated spectrum:
#            frequency_axis = np.linspace(0, (len(inst_freq)-1)/float(len(inst_freq))*(self.sl.dev.ADC_CLK_Hz), len(inst_freq))
#            last_index_shown = np.round(len(frequency_axis)/2)
#            window_function = np.blackman(len(inst_freq))
#            spc = np.abs(np.fft.fft((inst_freq-np.mean(inst_freq)) * window_function))/(sum(window_function)/2)
#            spc = 20*np.log10(np.abs(spc) + 1e-7)
#            self.curve_DDC0_spc.setData(frequency_axis[1:last_index_shown], spc[1:last_index_shown])

        # Refresh the display:

        self.qplt_DDC0_spc.replot()


        if window_NEB > 1e6:
            self.qlabel_ddc_rbw.setText('RBW: %.1f MHz; Points:' % (round(window_NEB*1e5)/1e5/1e6))
        elif window_NEB > 1e3:
            self.qlabel_ddc_rbw.setText('RBW: %.1f kHz; Points:' % (round(window_NEB*1e2)/1e2/1e3))
        else:
            self.qlabel_ddc_rbw.setText('RBW: %.0f Hz; Points:' % (round(window_NEB)))
#        pause(1/10.)

    def displayAnalyzer(self):

        start_time = time.clock()

        # Check if another function is currently using the DDR2 logger:
        if self.sl.bDDR2InUse:
            print('displayAnalyzer(): DDR2 logger in use, cannot get data from adc')
            return
        # Block access to the DDR2 Logger to any other function until we are done:
        self.sl.bDDR2InUse = True

        try:
            N_points = int(float(self.qedit_rawdata_length.text()))
        except:
            N_points = 4e3

        if N_points < 64:
            N_points = 64

        currentSelector = self.qcombo_adc_plot.currentIndex()
        # Python doesn't have switch-case statements
        # Apparently the best way is to use a dictionary instead:
        setup_func_dict = {0: self.sl.setup_ADC0_write,
                           1: self.sl.setup_ADC1_write,
                           2: self.sl.setup_DAC0_write,
                           3: self.sl.setup_DAC1_write,
                           4: self.sl.setup_DAC2_write}


        try:
            # Read from selected source
            setup_func_dict[currentSelector](N_points)
            self.sl.trigger_write()
            self.sl.wait_for_write()
            if currentSelector==0 or currentSelector==1:
                (samples_out_raw, ref_exp0) = self.sl.read_adc_samples_from_DDR2()
                self.ref_exp0 = ref_exp0
                self.raw_adc_samples = samples_out_raw
                # Scale samples to Full Scale (FS)
                samples_out = samples_out_raw.astype(dtype=np.float) / (self.sl.dev.ADC_INT_HR*2)
                # Scale samples to Volts
                samples_out_v = samples_out_raw.astype(dtype=np.float) * self.sl.dev.ADC_V_INT
            else:
                samples_out_raw = self.sl.read_dac_samples_from_DDR2()
                # Scale samples to Full Scale (FS)
                samples_out = samples_out_raw.astype(dtype=np.float) / (self.sl.dev.DAC_INT_HR*2)
                # Scale samples to Volts
                samples_out_v = samples_out_raw.astype(dtype=np.float) * self.sl.dev.DAC_V_INT

            max_abs = np.max(np.abs(samples_out_raw))

        except:
            # ADC read failed.
            print('Unhandled exception in ADC read')
            del self.sl
            raise

        # Signal to other functions that they can use the DDR2 logger
        self.sl.bDDR2InUse = False

        if self.bDisplayTiming == True:
            print('Elapsed time (Comm) = %f' % (time.clock()-start_time))
        start_time = time.clock()

        # Compute the window function used to display the spectrum:
        N_fft = 2**(int(np.ceil(np.log2(len(samples_out)))))
        frequency_axis = np.linspace(0, (N_fft-1)/float(N_fft)*self.sl.dev.ADC_CLK_Hz, N_fft)
        window_function = np.blackman(len(samples_out))
        last_index_shown = int(np.round(len(frequency_axis)/2))
        window_NEB = np.sum((window_function/np.sum(window_function))**2) * self.sl.dev.ADC_CLK_Hz

        # Show the RBW:
        if window_NEB > 1e6:
            self.qlabel_rawdata_rbw.setText('RBW: %.1f MHz' % (round(window_NEB*1e5)/1e5/1e6))
        elif window_NEB > 1e3:
            self.qlabel_rawdata_rbw.setText('RBW: %.1f kHz' % (round(window_NEB*1e2)/1e2/1e3))
        else:
            self.qlabel_rawdata_rbw.setText('RBW: %.0f Hz' % (round(window_NEB)))


        #----------------------------------------------------------------------
        #--- Setup Frequency Domain Plot
        #
        if self.qcombo_adc_plottype.currentIndex() == 0:    # Display Spectrum

            if self.bDisplayTiming == True:
                print('Elapsed time (pre-FFT) = %f' % (time.clock()-start_time))
            start_time = time.clock()

            # Apply window function to the data:
            samples_out_windowed = (samples_out-np.mean(samples_out)) * window_function

            if self.bDisplayTiming == True:
                print('Elapsed time (pre-FFT2) = %f' % (time.clock()-start_time))
            start_time = time.clock()

            # Compute the spectrum of the raw data:
            spc = np.fft.fft(samples_out_windowed, N_fft)

            if self.bDisplayTiming == True:
                print('Elapsed time (FFT) = %f' % (time.clock()-start_time))
            start_time = time.clock()

            spc = np.real(spc * np.conj(spc))/(np.sum(window_function)**2/4)
            spc = 10*np.log10(spc + 1e-13)


            if self.bDisplayTiming == True:
                print('Elapsed time (10log10 abs(FFT) = %f' % (time.clock()-start_time))
            start_time = time.clock()

            # Update the graph data:
            self.curve_spc.setData(frequency_axis[0:last_index_shown]/1e6, spc[0:last_index_shown])
            self.plt_spc.setXRange(frequency_axis[0]/1e6, frequency_axis[last_index_shown]/1e6)
            self.plt_spc.setYRange(-120, 0)
            self.plt_spc.showGrid(x=True, y=True)
            #self.qplt_spc.setAxisScale(Qwt.QwtPlot.xBottom, frequency_axis[0]/1e6, frequency_axis[last_index_shown]/1e6)
            #self.qplt_spc.setAxisScale(Qwt.QwtPlot.yLeft, -120, 0)
            self.plt_spc.setTitle('Spectrum')
            if not(currentSelector == 0 or currentSelector == 1):
                self.curve_filter.setVisible(False)

        #----------------------------------------------------------------------
        #--- Setup Time Domain Plot
        #
        elif self.qcombo_adc_plottype.currentIndex() == 1:
            # Display time-domain plot instead
            # Convert from Full Scale to Voltage
            time_axis = np.linspace(0, len(samples_out_v)-1, len(samples_out_v))/self.sl.dev.ADC_CLK_Hz

            self.curve_spc.setData(time_axis, samples_out_v)
#            self.qplt_spc.setAxisScale(Qwt.QwtPlot.yLeft, -120, 0)
#            self.qplt_spc.setAxisAutoScale(Qwt.QwtPlot.yLeft)
            self.curve_filter.setVisible(False)
            # Simply setting a curve to be invisible does not prevent it from being used to compute the axis, so we have to set the axis manually:
            valmin = np.min(samples_out_v)
            valmax = np.max(samples_out_v)

            # self.qplt_spc.setAxisScale(Qwt.QwtPlot.yLeft, (valmin+valmax)/2-1.1*(valmax-valmin)/2-1/65e3, (valmin+valmax)/2+1.1*(valmax-valmin)/2+1/65e3)
            # self.qplt_spc.setAxisScale(Qwt.QwtPlot.xBottom, time_axis[0], time_axis[-1])

            ####self.plt_spc.setYRange((valmin+valmax)/2-1.1*(valmax-valmin)/2-1/65e3, (valmin+valmax)/2+1.1*(valmax-valmin)/2+1/65e3)
            self.plt_spc.enableAutoRange(y=True)
            self.plt_spc.setXRange(time_axis[0], time_axis[-1])
            self.plt_spc.showGrid(x=True, y=True)

            self.plt_spc.setTitle('Time Domain signal = {:.2e} V; std = {:.1e} V'.format(np.mean(samples_out_v),np.std(samples_out_v)))



        #----------------------------------------------------------------------
        #--- ADC Data
        # If we are handling ADC0 or ADC1 data (as opposed to DAC data)
        #
        if currentSelector == 0 or currentSelector == 1:

            if np.real(ref_exp0) == 0 and np.imag(ref_exp0) == 0:
                print('displayAnalyzer(): Invalid complex exponential. Probably because of the USB bug.')
                return

            # The signal is from ADC0 or ADC1
            N_max_IQ = 10e3 # Max number of points to display in the IQ graph
            complex_baseband = self.sl.frontend_DDC_processing(self.raw_adc_samples, self.ref_exp0, self.selected_ADC)

            if self.bDisplayTiming == True:
                print('Elapsed time (Compute complex baseband) = %f' % (time.clock()-start_time))
            start_time = time.clock()

            # Update the scale which indicates the ADC fill ratio in numbers of bits:
            max_abs = np.max(np.abs(self.raw_adc_samples))
            if max_abs == 0:
                max_abs = 1 # to prevent passing a 0 value to the log function, which throws an exception
            max_abs_in_bits = np.log2(max_abs)

            self.qadc0_scale.setValue(max_abs_in_bits)
            self.qlabel_adc_fill_value.setText('{:.1f} bits'.format(max_abs_in_bits))

            # Compute the SNR on the amplitude of the baseband signal:
            amplitude = np.abs(complex_baseband)
            mean_amplitude = np.mean(amplitude)
            std_amplitude = np.std(amplitude)
            baseband_snr = 20*np.log10(np.mean(amplitude)/np.std(amplitude))
            # to get a more stable reading of the SNR without resorting to rounding:
            # we put a simple first-order IIR filter:
            filter_alpha = np.exp(-1./10.)
            temp_filtered_baseband_snr = filter_alpha * self.filtered_baseband_snr + (1-filter_alpha)*baseband_snr

            # Sometimes, the average of the amplitude is NaN, so we only accept the new SNR if it is not(NaN)
            if not(np.isnan(temp_filtered_baseband_snr)):
                self.filtered_baseband_snr = temp_filtered_baseband_snr
            else:
                print("Error 'nan' on filtered_baseband_snr")

            self.qthermo_baseband_snr.setValue(baseband_snr)
            self.qlabel_baseband_snr_value.setText('{:.2f} dB'.format(self.filtered_baseband_snr))

            #------------------------------------------------------------------
            #--- Plot ADC TD Phase Error
            #
            if self.qcombo_adc_plottype.currentIndex() == 2:
                # show phase error as a function of time

                # To mimick as much as possible the processing done in the FPGA, we quantize the complex baseband:
                complex_basebandr = np.round(complex_baseband)
                phi = np.unwrap(np.angle(complex_basebandr)) # returns radians
                phi_std = np.std(phi)
                # Set axis
                time_axis = np.linspace(0, len(complex_baseband)-1, len(complex_baseband))/self.sl.dev.ADC_CLK_Hz

                self.curve_spc.setData(time_axis, phi-phi[0])
    #            self.qplt_spc.setAxisScale(Qwt.QwtPlot.yLeft, -120, 0)
    #            self.qplt_spc.setAxisAutoScale(Qwt.QwtPlot.yLeft)
                self.curve_filter.setVisible(False)
                # Simply setting a curve to be invisible does not prevent it from being used to compute the axis, so we have to set the axis manually:
                valmin = np.min(phi)-phi[0]
                valmax = np.max(phi)-phi[0]

                # self.qplt_spc.setAxisScale(Qwt.QwtPlot.yLeft, valmin, valmax)
                # self.qplt_spc.setAxisScale(Qwt.QwtPlot.xBottom, time_axis[0], time_axis[-1])

                #self.plt_spc.setYRange(valmin, valmax)
                self.plt_spc.enableAutoRange(y=True)
                self.plt_spc.setXRange(time_axis[0], time_axis[-1])
                self.plt_spc.showGrid(x=True, y=True)

                self.plt_spc.setTitle('Time-domain phase, std = %.2f radrms' % phi_std)

            #------------------------------------------------------------------
            #--- Plot ADC TD IQ Signals
            #
            if self.qcombo_adc_plottype.currentIndex() == 3:

                # show time-domain I and Q signals
                # To mimick as much as possible the processing done in the FPGA, we quantize the complex baseband:
                complex_basebandr = np.round(complex_baseband)
                # Set axis
                time_axis = np.linspace(0, len(complex_basebandr)-1, len(complex_basebandr))/self.sl.dev.ADC_CLK_Hz


                self.curve_spc.setData(time_axis, np.real(complex_basebandr))
                self.curve_filter.setData(time_axis, np.imag(complex_basebandr))
    #            self.qplt_spc.setAxisScale(Qwt.QwtPlot.yLeft, -120, 0)
    #            self.qplt_spc.setAxisAutoScale(Qwt.QwtPlot.yLeft)
                self.curve_filter.setVisible(True)
                # Simply setting a curve to be invisible does not prevent it from being used to compute the axis, so we have to set the axis manually:
                valmax1 = np.mean(np.abs(complex_basebandr))

                self.plt_spc.setYRange(-1.5*valmax1, 1.5*valmax1)
                self.plt_spc.setXRange(time_axis[0], time_axis[-1])
                self.plt_spc.showGrid(x=True, y=True)

                self.plt_spc.setTitle('Time-domain IQ signals (I: blue, Q: red)')

            #------------------------------------------------------------------
            #--- Plot ADC TD Synced IQ Signals
            # Synced to this particular data trace, not to the internal FPGA LO
            #
            if self.qcombo_adc_plottype.currentIndex() == 4:

                # show time-domain I and Q signals
                # To mimick as much as possible the processing done in the FPGA, we quantize the complex baseband:
                complex_basebandr = np.round(complex_baseband)
                # Sync the phase to be equal to 0 at t=0:
                complex_basebandr = complex_basebandr * np.conj(complex_basebandr[0])/np.abs(complex_basebandr[0])
                # Set axis
                time_axis = np.linspace(0, len(complex_basebandr)-1, len(complex_basebandr))/self.sl.dev.ADC_CLK_Hz

                self.curve_spc.setData(time_axis, np.real(complex_basebandr))
                self.curve_filter.setData(time_axis, np.imag(complex_basebandr))
    #            self.qplt_spc.setAxisScale(Qwt.QwtPlot.yLeft, -120, 0)
    #            self.qplt_spc.setAxisAutoScale(Qwt.QwtPlot.yLeft)
                self.curve_filter.setVisible(True)
                # Simply setting a curve to be invisible does not prevent it from being used to compute the axis, so we have to set the axis manually:
                valmax1 = np.mean(np.abs(complex_basebandr))

                self.plt_spc.setYRange(-1.5*valmax1, 1.5*valmax1)
                self.plt_spc.setXRange(time_axis[0], time_axis[-1])
                self.plt_spc.showGrid(x=True, y=True)

                self.plt_spc.setTitle('Time-domain IQ signals, phase aligned at t=0')

            #------------------------------------------------------------------
            #--- Plot IQ Signals
            #
            #complex_baseband = complex_baseband[:int(np.min((len(complex_baseband), N_max_IQ)))]
            self.curve_IQ.setData(np.real(complex_baseband), np.imag(complex_baseband))

            self.qplt_IQ.setXRange(-(mean_amplitude+4*std_amplitude), (mean_amplitude+4*std_amplitude))
            self.qplt_IQ.setYRange(-(mean_amplitude+4*std_amplitude), (mean_amplitude+4*std_amplitude))
            # Refresh the display:
            # self.qplt_IQ.replot()

            if self.bDisplayTiming == True:
                print('Elapsed time (Display IQ) = %f' % (time.clock()-start_time))
            start_time = time.clock()

            #------------------------------------------------------------------
            #--- Plot Filter Response
            #

            if self.qcombo_adc_plottype.currentIndex() == 0:
                # Compute the spectrum of the filter:
                spc_filter = self.sl.get_frontend_filter_response(frequency_axis, self.selected_ADC)
#                spc_filter = np.sin(np.pi * (abs(frequency_axis-abs(f_reference))+10)*N_filter/self.sl.dev.ADC_CLK_Hz)/ (np.pi*(abs(frequency_axis-abs(f_reference))+10)*N_filter/self.sl.dev.ADC_CLK_Hz)
#                spc_filter = 20*np.log10(np.abs(spc_filter) + 1e-7)
                # Update the graph
                self.curve_filter.setData(frequency_axis[0:last_index_shown]/1e6, spc_filter[0:last_index_shown])
                self.curve_filter.setVisible(True)

            #------------------------------------------------------------------
            #--- Plot Amplitude Noise Spectrum
            #
            # If the phase noise spectrum is selected, we add the spectrum of the amplitude noise:
            if self.qcombo_ddc_plot.currentIndex() == 1 and True:
#                print('showing amplitude noise')
                # Normalize the amplitude to the carrier and remove DC:
                amplitude = amplitude / np.mean(amplitude) - 1

                # Decimate the signal since
                N_decimation = 10
                fs_new = self.sl.dev.ADC_CLK_Hz/N_decimation
                amplitude = decimate(amplitude, N_decimation)
#                amplitude = decimate(amplitude, N_decimation, zero_phase=False)

                N_fft = 2**(int(np.ceil(np.log2(len(amplitude)))))
                frequency_axis = np.linspace(0, (N_fft-1)/float(N_fft)*fs_new, N_fft)
                window_function = np.blackman(len(amplitude))
#                window_function = np.blackman(len(amplitude))
                window_NEB = np.sum((window_function/np.sum(window_function))**2) * fs_new
#                frequency_axis = np.linspace(0, (len(amplitude)-1)/float(len(amplitude))*(fs_new), len(amplitude))
                last_index_shown = int(np.round(len(frequency_axis)/2))

#                amplitude.resize(len(window_function))  # to match the window size
                spc = np.fft.fft(amplitude * window_function, N_fft)
                spc = np.real(spc*np.conj(spc))/(sum(window_function)**2) # Spectrum is now scaled in power (Hz^2 per bin)
                last_index_shown = int(np.round(len(frequency_axis)/2))
                # Scale the spectrum to be a single-sided power spectral density in Hz^2/Hz:
                spc[1:last_index_shown] = 2*spc[1:last_index_shown] / window_NEB
                spc = 10*np.log10(spc + 1e-20)

#                self.curve_DDC0_spc_amplitude_noise.setData(frequency_axis[1:last_index_shown], spc[1:last_index_shown])
                self.curve_DDC0_spc_amplitude_noise.setData(frequency_axis[1:last_index_shown], spc[1:last_index_shown])
                self.curve_DDC0_spc_amplitude_noise.setZValue(-1)
                self.curve_DDC0_spc_amplitude_noise.setVisible(True)

            else:
                # turn off amplitude noise curve:
                self.curve_DDC0_spc_amplitude_noise.setVisible(False)

            if self.bDisplayTiming == True:
                print('Elapsed time (Spectrum of amplitude noise) = %f' % (time.clock()-start_time))
            start_time = time.clock()




        else:
            # The signal is from DAC0 or DAC1:
            # Not sure what to put in the baseband IQ plot.  For now we put nothing
            #self.qthermo_baseband_snr.setValue(0)
#                empty_array = np.array(0)
#                self.curve_IQ.setData(empty_array, empty_array)
#                self.curve_DDC0_spc_amplitude_noise.setData(empty_array, empty_array)
#                self.curve_filter.setData(empty_array, empty_array)
            pass



        if self.bDisplayTiming == True:
            print('Elapsed time (self.plt_spc.replot()) = %f' % (time.clock()-start_time))
        start_time = time.clock()

#        self.bDisplayTiming = False
