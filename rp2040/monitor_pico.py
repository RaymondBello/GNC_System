import os 
import sys
import subprocess
import shutil
from datetime import datetime
from shutil import copy
from time import sleep
import serial
import glob
    
# In Linux use: alias pp='sudo putty -load PICO' 
#               alias bp='python3 build_pico.py'

CMAKE_PROJECT_NAME = 'RP2040_UAV'
PICO_SDK_PATH = os.environ.get('PICO_SDK_PATH')
PICO_PROJECT = os.path.dirname(os.path.realpath(__file__))
# PICO_USB_DRIVE = '/media/ray/RPI-RP2'
PICO_USB_DRIVE = '/Volumes/RPI-RP2'
MAIN_UF2 = f'{PICO_PROJECT}/build/{CMAKE_PROJECT_NAME}.uf2'

def get_pico_usb_name(usb_device_list):
    found_name = ""
    found_usb = False
    
    for i, device in enumerate(usb_device_list):
        if 'usbmodem' in str(device):
            found_name = usb_device_list[i]
            found_usb = True

    if not found_usb:
        return None
    
    return found_name

print("\n**************** Open Serial Monitor ****************")
# Open Putty and watch serial output
# sleep(2) # wait for PICO to restart

# os.system('sudo putty -load PICO')
# os.system(f'screen /dev/tty.usbmodem13401 115200')

list_of_connected_devices = glob.glob('/dev/tty.*')
pico_usb_name = get_pico_usb_name(list_of_connected_devices)

dateTimeObj = datetime.now()
timestampStr = dateTimeObj.strftime("%d_%b_%Y_%H-%M")
logfile = f'{PICO_PROJECT}/logs/log_{timestampStr}.txt'

if pico_usb_name:
    pico = serial.Serial(port = pico_usb_name, baudrate=115200)
    pico_log = open(logfile, "w")
    
    while pico.isOpen():
        pico_raw_line = pico.readline()
        try:
            pico_line = str(pico_raw_line).split("b'")[1].split('\\')[0]
        except:
            pico_line = str(pico_raw_line).split("b'")
        
        print(pico_line)  # Display serial data in console
        
        pico_log.write(f'{pico_line}\n')
        pico_log.flush() 
else:
    print('Exiting...')



