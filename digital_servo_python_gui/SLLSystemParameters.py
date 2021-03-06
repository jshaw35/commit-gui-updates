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

import json

class SLLSystemParameters():

    values_dict = {}

    def __init__(self, sl):
        assert isinstance(sl, SuperLaserLand_JD_RP.SuperLaserLand_JD_RP)
        self.sl = sl

        self.populateDefaults()
#        self.Writejson('hi_jshaw', "") # settings remain empty
        self.data = {}
        self.readfromjson = False

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

    def Readjson(self,jsonfile):            # Reads .json file into self.data dictionary
        print('reading %s' % jsonfile)
        
        
        with open(jsonfile) as json_file:  
            data = json.load(json_file)
            for p in data['Reference_frequency']:
                self.data['Reference_frequency'] = []
                self.data['Reference_frequency'].append({
                    'DDC0': p['DDC0'], 
                    'DDC1': p['DDC1']
                })
            for p in data['VCO_gain']:
                self.data['VCO_gain'] = []
                self.data['VCO_gain'].append({
                    'DAC0': p['DAC0'],
                    'DAC1': p['DAC1'],
                    'DAC2': p['DAC2']
                })
            for p in data['Output_limits_low']:
                self.data['Output_limits_low'] = []
                self.data['Output_limits_low'].append({
                    'DAC0': p['DAC0'],
                    'DAC1': p['DAC1'],
                    'DAC2': p['DAC2']
                })
            for p in data['Output_limits_high']:
                self.data['Output_limits_high'] = []
                self.data['Output_limits_high'].append({
                    'DAC0': p['DAC0'],
                    'DAC1': p['DAC1'],
                    'DAC2': p['DAC2']
                })
            for p in data['Input_Output_gain']:
                self.data['Input_Output_gain'] = []
                self.data['Input_Output_gain'].append({
                    'ADC0': p['ADC0'], 
                    'ADC1': p['ADC1'], 
                    'DAC0': p['DAC0'], 
                    'DAC1': p['DAC1']
                })
            for p in data['Output_offset_in_volts']:
                self.data['Output_offset_in_volts'] = []
                self.data['Output_offset_in_volts'].append({
                    'DAC0': p['DAC0'],
                    'DAC1': p['DAC1'],
                    'DAC2': p['DAC2']
                })
            for p in data['PLL0_settings']:
                self.data['PLL0_settings'] = []
                self.data['PLL0_settings'].append({
                    'kp': p['kp'], 
                    'fi': p['fi'], 
                    'fii': p['fii'], 
                    'fd': p['fd'], 
                    'fdf': p['fdf'], 
                    'chkKd': p['chkKd'],
                    'chkKp': p['chkKp'], 
                    'chkLock': p['chkLock'], 
                    'chkKpCrossing': p['chkKpCrossing']
                })
            for p in data['PLL1_settings']:
                self.data['PLL1_settings'] = []
                self.data['PLL1_settings'].append({
                    'kp': p['kp'], 
                    'fi': p['fi'], 
                    'fii': p['fii'], 
                    'fd': p['fd'], 
                    'fdf': p['fdf'], 
                    'chkKd': p['chkKd'],
                    'chkKp': p['chkKp'], 
                    'chkLock': p['chkLock'], 
                    'chkKpCrossing': p['chkKpCrossing']
                })
            for p in data['PLL2_settings']:
                self.data['PLL2_settings'] = []
                self.data['PLL2_settings'].append({
                    'kp': p['kp'], 
                    'fi': p['fi'], 
                    'fii': p['fii'], 
                    'fd': p['fd'], 
                    'fdf': p['fdf'], 
                    'chkKd': p['chkKd'],
                    'chkKp': p['chkKp'], 
                    'chkLock': p['chkLock'], 
                    'chkKpCrossing': p['chkKpCrossing']
                })
            for p in data['PWM0_settings']:
                self.data['PWM0_settings'] = []
                self.data['PWM0_settings'].append({
                    'standard': p['standard'], 
                    'levels': p['levels'], 
                    'default': p['default'], 
                    'minval': p['minval'], 
                    'maxval': p['maxval']
                })
            for p in data['Main_window_settings']:
                self.data['Main_window_settings'] = []
                self.data['Main_window_settings'].append({
                    'refresh_delay': p['refresh_delay'], 
                    'N_samples_adc': p['N_samples_adc'], 
                    'N_samples_ddc': p['N_samples_ddc'], 
                    'Integration_limit': p['Integration_limit']
                })
            for p in data['Triangular_averaging']:
                self.data['Triangular_averaging'] = []
                self.data['Triangular_averaging'].append({
                    'DAC0': p['DAC0'], 
                    'DAC1': p['DAC1']
                })
            for p in data['Dither_frequency']:
                self.data['Dither_frequency'] = []
                self.data['Dither_frequency'].append({
                    'DAC0': p['DAC0'], 
                    'DAC1': p['DAC1']
                })
            for p in data['Dither_integration_time']:
                self.data['Dither_integration_time'] = []
                self.data['Dither_integration_time'].append({
                    'DAC0': p['DAC0'], 
                    'DAC1': p['DAC1']
                })
            for p in data['Dither_amplitude']:
                self.data['Dither_amplitude'] = []
                self.data['Dither_amplitude'].append({
                    'DAC0': p['DAC0'], 
                    'DAC1': p['DAC1']
                })
            for p in data['Dither_mode']:
                self.data['Dither_mode'] = []
                self.data['Dither_mode'].append({
                    'DAC0': p['DAC0'], 
                    'DAC1': p['DAC1']
                })
            for p in data['VCO_settings']:
                self.data['VCO_settings'] = []
                self.data['VCO_settings'].append({
                    'VCO_offset': p['VCO_offset'], 
                    'VCO_amplitude': p['VCO_amplitude'], 
                    'VCO_connection': p['VCO_connection']
                })
            for p in data['RP_settings']:
                self.data['RP_settings'] = []
                self.data['RP_settings'].append({
                    'Fan_state': p['Fan_state'], 
                    'PLL1_connection': p['PLL1_connection']
                })
            for p in data['Filter_select']:
                self.data['Filter_select'] = []
                self.data['Filter_select'].append({
                    'DAC0': p['DAC0'], 
                    'DAC1': p['DAC1']
                })
            for p in data['Angle_select']:
                self.data['Angle_select'] = []
                self.data['Angle_select'].append({
                    'DAC0': p['DAC0'], 
                    'DAC1': p['DAC1']
                })
        print('%s read into self.data' % jsonfile)
#        self.Printjson(jsonfile)
    
    def Printjson(self,jsonfile): # Display for checking things
        print('printing .json')
        
        with open(jsonfile) as json_file:  
            data = json.load(json_file)
            for p in data['Reference_frequency']:
                print('DDC0: ' + p['DDC0'])
                print('DDC1: ' + p['DDC1'])
#                print('From: ' + p['from'])
                print('')
            for p in data['VCO_gain']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('DAC2: ' + p['DAC2'])
                print('')
            for p in data['Output_limits_low']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('DAC2: ' + p['DAC2'])
                print('')
            for p in data['Output_limits_high']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('DAC2: ' + p['DAC2'])
                print('')
            for p in data['Input_Output_gain']:
                print('ADC0: ' + p['ADC0'])
                print('ADC1: ' + p['ADC1'])
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('')
            for p in data['Output_offset_in_volts']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('DAC2: ' + p['DAC2'])
                print('')
            for p in data['PLL0_settings']:
                print('kp: ' + p['kp'])
                print('fi: ' + p['fi'])
                print('fii: ' + p['fii'])
                print('fd: ' + p['fd'])
                print('fdf: ' + p['fdf'])
                print('chkKd: ' + p['chkKd'])
                print('chkKp: ' + p['chkKp'])
                print('chkLock: ' + p['chkLock'])
                print('chkKpCrossing: ' + p['chkKpCrossing'])
                print('')
            for p in data['PLL1_settings']:
                print('kp: ' + p['kp'])
                print('fi: ' + p['fi'])
                print('fii: ' + p['fii'])
                print('fd: ' + p['fd'])
                print('fdf: ' + p['fdf'])
                print('chkKd: ' + p['chkKd'])
                print('chkKp: ' + p['chkKp'])
                print('chkLock: ' + p['chkLock'])
                print('chkKpCrossing: ' + p['chkKpCrossing'])
                print('')
            for p in data['PLL2_settings']:
                print('kp: ' + p['kp'])
                print('fi: ' + p['fi'])
                print('fii: ' + p['fii'])
                print('fd: ' + p['fd'])
                print('fdf: ' + p['fdf'])
                print('chkKd: ' + p['chkKd'])
                print('chkKp: ' + p['chkKp'])
                print('chkLock: ' + p['chkLock'])
                print('chkKpCrossing: ' + p['chkKpCrossing'])
                print('')
            for p in data['PWM0_settings']:
                print('standard: ' + p['standard'])
                print('levels: ' + p['levels'])
                print('default: ' + p['default'])
                print('minval: ' + p['minval'])
                print('maxval: ' + p['maxval'])
                print('')
            for p in data['Main_window_settings']:
                print('refresh_delay: ' + p['refresh_delay'])
                print('N_samples_adc: ' + p['N_samples_adc'])
                print('N_samples_ddc: ' + p['N_samples_ddc'])
                print('Integration_limit: ' + p['Integration_limit'])
                print('')
            for p in data['Triangular_averaging']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('')
            for p in data['Dither_frequency']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('')
            for p in data['Dither_integration_time']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('')
            for p in data['Dither_amplitude']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('')
            for p in data['Dither_mode']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('')
            for p in data['VCO_settings']:
                print('VCO_offset: ' + p['VCO_offset'])
                print('VCO_amplitude: ' + p['VCO_amplitude'])
                print('VCO_connection: ' + p['VCO_connection'])
                print('')
            for p in data['RP_settings']:
                print('Fan_state: ' + p['Fan_state'])
                print('PLL1_connection: ' + p['PLL1_connection'])
                print('')
            for p in data['Filter_select']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('')
            for p in data['Angle_select']:
                print('DAC0: ' + p['DAC0'])
                print('DAC1: ' + p['DAC1'])
                print('')

    def Writejson(self, strFilename, settings):
        # organizing settings 
        window1_outs = settings['window1_outs']
        window2_outs = settings['window2_outs']
        freq_window1_outs = settings['freq_window1_outs']
        freq_window2_outs = settings['freq_window2_outs']
        dither1_outs = settings['dither1_outs']
        dither2_outs = settings['dither2_outs']
        rp_settings_outs = settings['rp_settings_outs']
        divider_settings_outs = settings['divider_settings_outs']
        
        self.dataoutput = {}  
        
        self.dataoutput['Reference_frequency'] = []  
        self.dataoutput['Reference_frequency'].append({
            'DDC0': window1_outs['ref_freq'], 
            'DDC1': window2_outs['ref_freq']
        })

        self.dataoutput['VCO_gain'] = []
        self.dataoutput['VCO_gain'].append({
            'DAC0': window1_outs['vco_gain'],
            'DAC1': window2_outs['vco_gain'],
            'DAC2': '9e6'
        })
        self.dataoutput['Output_limits_low'] = []
        self.dataoutput['Output_limits_low'].append({
            'DAC0': window1_outs['limit_low'],
            'DAC1': window2_outs['limit_low'],
            'DAC2': '0'
        })
        self.dataoutput['Output_limits_high'] = []
        self.dataoutput['Output_limits_high'].append({
            'DAC0': window1_outs['limit_high'],
            'DAC1': window2_outs['limit_high'],
            'DAC2': '55'
        })
        self.dataoutput['Input_Output_gain'] = []
        self.dataoutput['Input_Output_gain'].append({
            'ADC0': '1', 
            'ADC1': '1', 
            'DAC0': '1', 
            'DAC1': '1'
        })
        self.dataoutput['Output_offset_in_volts'] = []
        self.dataoutput['Output_offset_in_volts'].append({
            'DAC0': window1_outs['dac_offset'],
            'DAC1': window2_outs['dac_offset'],
            'DAC2': '27'
        })
        self.dataoutput['PLL0_settings'] = []
        self.dataoutput['PLL0_settings'].append({
            'kp': window1_outs['kp'], 
            'fi': window1_outs['fi'], 
            'fii': window1_outs['fii'], 
            'fd': window1_outs['fd'], 
            'fdf': window1_outs['fdf'], 
            'chkKd': window1_outs['chkKd'],
            'chkKp': window1_outs['chkKp'], 
            'chkLock': window1_outs['chkLock'], 
            'chkKpCrossing': window1_outs['chkKpCrossing']
        })
        self.dataoutput['PLL1_settings'] = []
        self.dataoutput['PLL1_settings'].append({
            'kp': window2_outs['kp'], 
            'fi': window2_outs['fi'], 
            'fii': window2_outs['fii'], 
            'fd': window2_outs['fd'], 
            'fdf': window2_outs['fdf'], 
            'chkKd': window2_outs['chkKd'],
            'chkKp': window2_outs['chkKp'], 
            'chkLock': window2_outs['chkLock'], 
            'chkKpCrossing': window2_outs['chkKpCrossing']
        })
        self.dataoutput['PLL2_settings'] = []
        self.dataoutput['PLL2_settings'].append({
            'kp': '-5.6',
            'fi': '141e3', 
            'fii': '3.24e3', 
            'fd': '1', 
            'fdf': '1', 
            'chkKd': 'False',
            'chkKp': 'True', 
            'chkLock': 'False', 
            'chkKpCrossing': 'True'
        })
        self.dataoutput['PWM0_settings'] = []
        self.dataoutput['PWM0_settings'].append({
            'standard': '3.3', 
            'levels': '256', 
            'default': '0.0', 
            'minval': '0.0', 
            'maxval': '3.3'
        })
        self.dataoutput['Main_window_settings'] = []
        self.dataoutput['Main_window_settings'].append({
            'refresh_delay': '500', 
            'N_samples_adc': '1.75e3', 
            'N_samples_ddc': '1e6', 
            'Integration_limit': '5e6'
        })
        self.dataoutput['Triangular_averaging'] = []
        self.dataoutput['Triangular_averaging'].append({
            'DAC0': freq_window1_outs, 
            'DAC1': freq_window2_outs
        })
        self.dataoutput['Dither_frequency'] = []
        self.dataoutput['Dither_frequency'].append({
            'DAC0': dither1_outs['modulation_freq'],
            'DAC1': dither2_outs['modulation_freq'] 
        })
        self.dataoutput['Dither_integration_time'] = []
        self.dataoutput['Dither_integration_time'].append({
            'DAC0': dither1_outs['integration_time_in_seconds'],
            'DAC1': dither2_outs['integration_time_in_seconds'] 
        })
        self.dataoutput['Dither_amplitude'] = []
        self.dataoutput['Dither_amplitude'].append({
            'DAC0': dither1_outs['amplitude'],
            'DAC1': dither2_outs['amplitude'] 
        })
        self.dataoutput['Dither_mode'] = []
        self.dataoutput['Dither_mode'].append({
            'DAC0': dither1_outs['mode'],
            'DAC1': dither2_outs['mode'] 
        })
        self.dataoutput['VCO_settings'] = []
        self.dataoutput['VCO_settings'].append({
            'VCO_offset': rp_settings_outs['VCO_offset'], 
            'VCO_amplitude': rp_settings_outs['VCO_amplitude'], 
            'VCO_connection': rp_settings_outs['VCO_connection']
        })
        self.dataoutput['RP_settings'] = []
        self.dataoutput['RP_settings'].append({
            'Fan_state': rp_settings_outs['Fan_state'], 
            'PLL1_connection': rp_settings_outs['PLL1_connection']
        })
        self.dataoutput['Filter_select'] = []
        self.dataoutput['Filter_select'].append({
            'DAC0': divider_settings_outs['filter_select_0'], 
            'DAC1': divider_settings_outs['filter_select_1']
        })
        self.dataoutput['Angle_select'] = []
        self.dataoutput['Angle_select'].append({
            'DAC0': divider_settings_outs['angle_select_0'], 
            'DAC1': divider_settings_outs['angle_select_1']
        })
    
        with open(strFilename, 'w') as outfile:  
            json.dump(self.dataoutput, outfile)
        
#        self.Readjson(strFilename)
            
#        strFilename = "new_settings_for_Jonah"
            # Create the tree structure:
#        self.newroot = ET.Element('SuperLaserLandPLL_settings')
#        
#        # Send update values from controller
#        
#        # Organize values from controller
#        
#        # useful stuff
#        
#        self.newtree = ET.ElementTree(self.newroot)
#        
#        self.newtree.write(strFilename + ".xml")
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
        print('loading from %s' % strFilename)
        if strFilename[-5:] == '.json':         # if settings file is .json format
            print('trying to load .json file')
            self.readfromjson = True
            self.Readjson(strFilename)
#            print("Reading from .json: %s" % self.readfromjson)
#        parser = ET.XMLParser(encoding="utf-8")
#        self.tree = ET.fromstring(strFilename, parser=parser)
        else:
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

    def getValuejson(self, strKey, strParameter):
        for p in self.data[strKey]:
            return p[strParameter]

    def getValue(self, strKey, strParameter):        
        if self.readfromjson == True:                       # to handle .json files
            return self.getValuejson(strKey,strParameter)
        
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