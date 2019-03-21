# -*- coding: utf-8 -*-
"""
Created on Fri Dec 13 16:24:57 2013

@author: jnd
"""
from __future__ import print_function

# This class implements a thin wrapper around the ElementTree/Element classes, which does XML parsing/writing.
# This allows us change the implementation if we want, without having to rewrite the UI code.
# from xml.etree.ElementTree import ElementTree as ET, Element
import xml.etree.ElementTree as ET

import SuperLaserLand_JD_RP


class SLLSystemParameters():

    values_dict = {}

    def __init__(self, sl):
        assert isinstance(sl, SuperLaserLand_JD_RP.SuperLaserLand_JD_RP)
        self.sl = sl

        self.populateDefaults()
#        self.NewTree('hi_jshaw', "") # settings remain empty

        return

    def populateDefaults(self):
        # Create the tree structure:
        self.root = ET.Element('SuperLaserLandPLL_settings')
        self.tree = ET.ElementTree(self.root)

        # Default values for all the parameters:
        self.root.append(ET.Element('Reference_frequency', DDC0='31.25e6', DDC1='31.25e6'))
        self.root.append(ET.Element('VCO gain', DAC0='2e8', DAC1='0.5e6', DAC2='9e6'))
        self.root.append(ET.Element('Output_limits_low', DAC0='-1.0', DAC1='-0', DAC2='0'))
        self.root.append(ET.Element('Output_limits_high', DAC0='1.0', DAC1='1', DAC2='55'))
        self.root.append(ET.Element('Input_Output_gain', ADC0='1', ADC1='1', DAC0='1', DAC1='1'))
        self.root.append(ET.Element('Output_offset_in_volts', DAC0='0.0274', DAC1='0', DAC2='27'))
        self.root.append(ET.Element('PLL0_settings', kp='10', fi='45e3', fii='3.4e3', fd='1', fdf='1', chkKd='False', chkKp='False', chkLock='False', chkKpCrossing='False'))
        self.root.append(ET.Element('PLL1_settings', kp='-5.6', fi='141e3', fii='3.24e3', fd='1', fdf='1', chkKd='False', chkKp='True', chkLock='False', chkKpCrossing='True'))
        self.root.append(ET.Element('PLL2_settings', kp='-120', fi='1e-2', fii='0', fd='1', fdf='1', chkKd='False', chkKp='False', chkLock='False', chkKpCrossing='False'))

        self.root.append(ET.Element('PWM0_settings', standard='3.3', levels='256', default='0.0', minval='0.0', maxval='3.3'))

        self.root.append(ET.Element('Main_window_settings', refresh_delay='500', N_samples_adc='1.75e3', N_samples_ddc='1e6', Integration_limit='5e6'))
        self.root.append(ET.Element('Triangular_averaging', DAC1='1', DAC0='1'))

        self.root.append(ET.Element('Dither_frequency', DAC1='5.1e3', DAC0='1e3'))
        self.root.append(ET.Element('Dither_integration_time', DAC1='0.1', DAC0='0.1'))
        self.root.append(ET.Element('Dither_amplitude', DAC1='1e.3', DAC0='1e-3'))
        self.root.append(ET.Element('Dither_mode', DAC1='2', DAC0='2'))

        self.root.append(ET.Element('VCO_settings', VCO_offset='0.00', VCO_amplitude='0.5', VCO_connection='0'))
        self.root.append(ET.Element('RP_settings', Fan_state='0', PLL1_connection='0'))
        self.root.append(ET.Element('Filter_select', DAC1='0', DAC0='0'))
        self.root.append(ET.Element('Angle_select', DAC1='0', DAC0='0'))

    def NewTree(self, strFilename, settings):
#        strFilename = "new_settings_for_Jonah"
            # Create the tree structure:
        self.newroot = ET.Element('SuperLaserLandPLL_settings')
        
        # Send update values from controller
        
        # Organize values from controller
        
        # useful stuff
        self.newroot.append(ET.Element('Reference_frequency', DDC0='35.96e6', DDC1='35.96e6'))
        self.newroot.append(ET.Element('VCO gain', DAC0='2e8', DAC1='0.5e6', DAC2='9e6'))
        self.newroot.append(ET.Element('Output_limits_low', DAC0='-1.0', DAC1='-0', DAC2='0'))
        self.newroot.append(ET.Element('Output_limits_high', DAC0='1.0', DAC1='1', DAC2='55'))
        self.newroot.append(ET.Element('Input_Output_gain', ADC0='1', ADC1='1', DAC0='1', DAC1='1'))
        self.newroot.append(ET.Element('Output_offset_in_volts', DAC0='0.0274', DAC1='0', DAC2='27'))
        self.newroot.append(ET.Element('PLL0_settings', kp='10', fi='45e3', fii='3.4e3', fd='1', fdf='1', chkKd='False', chkKp='False', chkLock='False', chkKpCrossing='False'))
        self.newroot.append(ET.Element('PLL1_settings', kp='-5.6', fi='141e3', fii='3.24e3', fd='1', fdf='1', chkKd='False', chkKp='True', chkLock='False', chkKpCrossing='True'))
        self.newroot.append(ET.Element('PLL2_settings', kp='-120', fi='1e-2', fii='0', fd='1', fdf='1', chkKd='False', chkKp='False', chkLock='False', chkKpCrossing='False'))

        self.newroot.append(ET.Element('PWM0_settings', standard='3.3', levels='256', default='0.0', minval='0.0', maxval='3.3'))

        self.newroot.append(ET.Element('Main_window_settings', refresh_delay='500', N_samples_adc='1.75e3', N_samples_ddc='1e6', Integration_limit='5e6'))
        self.newroot.append(ET.Element('Triangular_averaging', DAC1='1', DAC0='1'))

        self.newroot.append(ET.Element('Dither_frequency', DAC1='5.1e3', DAC0='1e3'))
        self.newroot.append(ET.Element('Dither_integration_time', DAC1='0.1', DAC0='0.1'))
        self.newroot.append(ET.Element('Dither_amplitude', DAC1='1e.3', DAC0='1e-3'))
        self.newroot.append(ET.Element('Dither_mode', DAC1='2', DAC0='2'))

        self.newroot.append(ET.Element('VCO_settings', VCO_offset='0.00', VCO_amplitude='0.5', VCO_connection='0'))
        self.newroot.append(ET.Element('RP_settings', Fan_state='0', PLL1_connection='0'))
        self.newroot.append(ET.Element('Filter_select', DAC1='0', DAC0='0'))
        self.newroot.append(ET.Element('Angle_select', DAC1='0', DAC0='0'))
        
        self.newtree = ET.ElementTree(self.newroot)
        
        self.newtree.write(strFilename + ".xml")
        # create a new XML file with the results        
#        mydata = ET.tostring(self.newtree)  
#        myfile = open(strFilename, "w")  
#        myfile.write(mydata)
        
#        mydata = str(self.newtree)  
#        myfile = open(strFilename + '.xml', "w")  
#        myfile.write(mydata)
        print("Wrote new file to %s" % strFilename)
        return
    
#         Collect values for all the parameters:
#         Organization and locations
#         Location: XEM_GUI_MainWindow
#         strDAC = 'DAC{:01d}'.format(k)
#         Values: 
#             VCO_gain, strDAC (3)                       <-- self.qedit_vco_gain[k].setText(str_VCO_gain)
#             Output_offset_in_volts, strDAC (3)         <-- self.sl.DACs_offset[k]
#             Output_limits_low, strDAC (3)              <-- self.sl.DACs_limit_low[k]
#             Output_limits_high, strDAC (3)             <-- self.sl.DACs_limit_high[k]
#             'PWM0_settings', 'standard'
#             'PWM0_settings', 'levels'
#             'PWM0_settings', 'default'
#             'PWM0_settings', 'minval'
#             'PWM0_settings', 'maxval'
#             'Reference_frequency', strDDC (3)          <-- self.qedit_ref_freq.getText(str_ref_freq)
#             
#        Location: FreqErrorWindowWithTempControlV2
#        Values:
#            'Triangular_averaging', strDAC (2)          <-- self.qchk_triangular.setChecked(bTriangularAveraging)
#
#        Location: DisplayDitherSettings
#        strDAC = 'DAC{:01d}'.format(self.output_number)
#        Values:
#            'Dither_frequency', strDAC                  <-- self.qedit_dither_freq.getText()
#            'Dither_amplitude', strDAC                  <-- self.qedit_dither_amplitude.setText()
#            'Dither_integration_time', strDAC           <-- self.qedit_integration_time.getText()
#            'Dither_mode', strDAC
#                depends on the values of 
#                self.qchk_mode_manual_off.setChecked(True) <-- 0
#                self.qchk_mode_manual_on.setChecked(True) <-- 1
#                self.qchk_mode_auto.setChecked(True) <-- 2
#            
#        Location: ConfigurationRPSettingsUI
#        Values:
#            'RP_settings', "Fan_state"                  <-- 0: self.qradio_fan_on.checked() = False | 1: self.qradio_fan_on.checked() = True
#            'RP_settings', "PLL1_connection"            <-- 0: self.qradio_ddc1_to_pll1.setChecked(True) | 1: self.qradio_ddc0_to_pll1.setChecked(True) | 2: self.qradio_pll0_to_pll1.setChecked(True)
#            'VCO_settings', "VCO_connection"            <-- 0: self.qradio_no_VCO.setChecked(True) | 1: self.qradio_VCO_to_DAC0.setChecked(True) | 2: self.qradio_VCO_to_DAC1.setChecked(True)
#            'VCO_settings', "VCO_amplitude"             <-- self.qedit_int_vco_amplitude.getText()
#            'VCO_settings', "VCO_offset"                <-- self.qedit_int_vco_offset.getText()
#
#        Location: DisplayDividerAndResidualsStreamingSettingsWindow
#        Values:
#            'Filter_select', "DAC1"                     <-- 0: self.qchk_Wideband1.setChecked(True) | 1: self.qchk_Narrowband1.setChecked(True) | 2: self.qchk_WidebandFIR1.setChecked(True)
#            'Filter_select', "DAC0"                     <-- 0: self.qchk_Wideband0.setChecked(True) | 1: self.qchk_Narrowband0.setChecked(True) | 2: self.qchk_WidebandFIR0.setChecked(True)
#            'Angle_select', "DAC1"                      <-- 0: self.qchk_cordic1.setChecked(True) | 1: self.qchk_quadrature_msb1.setChecked(True) | 2: self.qchk_quadrature_lsb1.setChecked(True) | 3: self.qchk_inphase_msb1.setChecked(True) | 4: self.qchk_inphase_lsb1.setChecked(True)
#            'Angle_select', "DAC0"                      <-- 0: self.qchk_cordic0.setChecked(True) | 1: self.qchk_quadrature_msb0.setChecked(True) | 2: self.qchk_quadrature_lsb0.setChecked(True) | 3: self.qchk_inphase_msb0.setChecked(True) | 4: self.qchk_inphase_lsb0.setChecked(True)
#
#        Location: LoopFiltersUI
#        strPLL = 'PLL{:01d}_settings'.format(self.filter_number)
#        Values:
#            strPLL, 'kp'                                <-- self.qedit_kp.setText('{:.3}'.format(kp))
#            strPLL, 'fi'                                <-- self.qedit_fi.setText('{:.3e}'.format(fi))
#            strPLL, 'fii'                               <-- self.qedit_fii.setText('{:.3e}'.format(fii))
#            strPLL, 'fd'                                <-- self.qedit_fd.setText('{:.3e}'.format(fd))
#            strPLL, 'fdf'                               <-- self.qedit_fdf.setText('{:.3e}'.format(fdf))
#            strPLL, 'chkKp').lower() == 'true'          <-- self.qchk_kp.setChecked(bKp)
#            strPLL, 'chkKd').lower() == 'true'          <-- self.qchk_kd.setChecked(bKd)
#            strPLL, 'chkLock').lower() == 'true'        <-- self.qchk_lock.setChecked(bLock)
#            strPLL, 'chkKpCrossing').lower() == 'true'  <-- self.qchk_bKpCrossing.setChecked(kKpCrossing)
#
#        Missing: Input_Output_gain, PLL0_settings, PLL1_settings, PLL2_settings, MainWindowSettings
#        PLL settings should be in loadParameters in LoopFiltersUI



    def loadFromFile(self, strFilename):
#        parser = ET.XMLParser(encoding="utf-8")
#        self.tree = ET.fromstring(strFilename, parser=parser)
        self.tree = ET.parse(strFilename)
        self.root = self.tree.getroot()

        # we used to do error checking at this level, but now it is implemented one layer higher in the hierarchy (currently in XEM_GUI3.py)
        # try:
            # self.tree = ET.parse(strFilename)
            # self.root = self.tree.getroot()
        # except IOError:
        #     print("IOError when trying to parse configuration file %s. using default values" % (strFilename))
        #     self.populateDefaults()
        # return

    def saveToFile(self, strFilename):
        self.tree.write(strFilename)
        return

    def getValue(self, strKey, strParameter):
        return self.tree.find(strKey).attrib[strParameter]

    def setValue(self, strKey, strParameter, strValue):
        self.tree.find(strKey).attrib[strParameter] = strValue

    def sendToFPGA(self, bSendToFPGA = True):

        # Set the DAC output limits:
        limit_low = float(self.getValue('Output_limits_low', 'DAC0'))    # the limit is in volts
        limit_high = float(self.getValue('Output_limits_high', 'DAC0'))    # the limit is in volts
        self.sl.set_dac_limits(0,
                               int(limit_low/self.sl.dev.DAC_V_INT),
                               int(limit_high/self.sl.dev.DAC_V_INT))
        limit_low = float(self.getValue('Output_limits_low', 'DAC1'))    # the limit is in volts
        limit_high = float(self.getValue('Output_limits_high', 'DAC1'))    # the limit is in volts
        self.sl.set_dac_limits(1,
                               int(limit_low/self.sl.dev.DAC_V_INT),
                               int(limit_high/self.sl.dev.DAC_V_INT))
        limit_low = float(self.getValue('Output_limits_low', 'DAC2'))    # the limit is in volts
        limit_high = float(self.getValue('Output_limits_high', 'DAC2'))    # the limit is in volts
        self.sl.set_dac_limits(2,
                               int(limit_low/self.sl.dev.DAC_V_INT),
                               int(limit_high/self.sl.dev.DAC_V_INT))
        ##
        ## HB, 4/27/2015, Added PWM support on DOUT0
        ##
        PWM0_standard = float(self.getValue('PWM0_settings', 'standard'));
        PWM0_levels   = int(self.getValue('PWM0_settings', 'levels'));
        PWM0_default  = float(self.getValue('PWM0_settings', 'default'));
        # Convert to counts
        value_in_counts = self.sl.convertPWMVoltsToCounts(PWM0_standard, PWM0_levels, PWM0_default)
        # Send to FPGA
        self.sl.set_pwm_settings(PWM0_levels, value_in_counts, bSendToFPGA)


def main():
    # Create a system parameters object, just for testing:
    sp = SLLSystemParameters()
    sp.saveToFile('test.xml')

    rep = sp.tree.find('Reference_frequency')
    print(rep)
    print(rep.attrib['DDC0'])

    print(sp.getValue('Reference_frequency', 'DDC0'))
    sp.setValue('Reference_frequency', 'DDC0', '5.1e6')
    print(sp.getValue('Reference_frequency', 'DDC0'))

#    for el in list(sp.root.iter('Default output offset')):
#        print(el)
#        print(el.attrib['DAC0'])

    return

if __name__ == '__main__':
    main()