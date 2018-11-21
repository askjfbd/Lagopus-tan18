###################################################
#Author:H.Sato
#20180711
#add gps,FullScreen to 2018TF3-2.py
#
###################################################

import subprocess
import threading
import locale
import sys
import os
import fcntl
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.basemap import Basemap
from PIL import Image
lat, lon= 35.655623, 139.539250
#fuji
minlat, minlon, maxlat, maxlon = 35.1160039, 138.6210271, 35.1259951, 138.6407326
#UEC
#minlat, minlon, maxlat, maxlon = 35.655623, 139.539250, 35.658918, 139.545403
#japan
#minlat, minlon, maxlat, maxlon = 34.78086, 136.9963732, 36.0000714, 139.7108033

import serial
import csv
from PyQt4 import  QtGui,QtCore
from window0703 import Ui_MainWindow
import time
from PyQt4.QtGui import *
import signal
import smbus
import datetime
from time import sleep
import concurrent.futures
from multiprocessing import Process
from multiprocessing import Queue
from multiprocessing import Value
from multiprocessing import Array
from ctypes import c_wchar_p
#root_count = 0
counter_lcd = 0
counter_time = 0
flag = 0
args = sys.argv
argc = len(args)
listData = []
csvfile = args[1]
gpsfile = args[2]
f = open(csvfile,"a")
f2 = open(gpsfile,"a")
writer  = csv.writer(f, lineterminator='\n')
writer2 = csv.writer(f2, lineterminator='\n')
rvocount = 0
count = 0
sensor = "cadence"
val_cadence = 0
val_pitch_x = 0
val_pitch_y = 0
bus = smbus.SMBus(1)
min_time =0
address = 0x25
registerMap = {
  "cadence"       :[0xF0,0xF1],
  "pitch"      :[0xF2,0xF3],
  "alt"    :[0xF4,0xF5],
  "speed"   :[0xF6,0xF7],
  }
#########################################################
#GUI
#########################################################
class Example(QtGui.QMainWindow):
    def __init__(self,gps_flag,alt_flag,data_cadence,data_alt,data_speed,data_rudder,data_elvator,data_flag,parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.interval = 1
        self.ui.setupUi(self)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(self.interval)
        self.timer.timeout.connect(self.display)
        self.timer_lcd = QtCore.QTimer(self)
        self.timer_lcd.setInterval(self.interval)
        self.timer_lcd.timeout.connect(self.count_lcd)
        self.timer_pitch = QtCore.QTimer(self)
        self.timer_pitch.setInterval(self.interval)
        self.timer_pitch.timeout.connect(self.count_pitch)
        self.timer_time = QtCore.QTimer(self)
        self.timer_time.setInterval(self.interval)
        self.timer_time.timeout.connect(self.count_time)
        self.start_count_display()
        self.start_count_lcd()
        self.start_count_pitch()
        self.start = time.time()
        self.start_count_time()
        self.ui.label_3.setPixmap(QtGui.QPixmap("/home/pi/work/Python/wings18/viggen/i.png"))
#########################################################
#pitch update
#########################################################
    def start_count_pitch(self):
        self.timer_pitch.start()
    def count_pitch(self):
        if (data_flag.value==1):
                self.ui.horizontalSlider.setValue(data_rudder.value)
                self.ui.verticalSlider.setValue(data_elvator.value)
        self.start_count_pitch()
#########################################################
#7Segment update
#########################################################
    def start_count_lcd(self):
        self.timer_lcd.start()        
    def count_lcd(self):
        global counter_lcd
        counter_lcd += 1
        if counter_lcd == 500:
            if (data_flag.value==1):
                self.ui.lcdNumber_2.display(data_cadence.value)
#                self.ui.lcdNumber_2.display(data_elvator.value)                
                self.ui.lcdNumber_3.display(data_alt.value)
#                self.ui.lcdNumber_3.display(data_rudder.value)                
            counter_lcd = 0
        self.start_count_lcd()                          
#########################################################
#time update
#########################################################
    def start_count_time(self):
        self.timer_time.start()        
    def count_time(self):
        global counter_time
        global min_time        
        counter_time += 1
        if counter_time == 100:
            self.sec_time =int(time.time() - self.start)
            if self.sec_time == 60:
                self.start = time.time()
                self.sec_time = 0
                min_time += 1
            self.ui.lcdNumber_4.display(min_time)                
            self.ui.lcdNumber_5.display(self.sec_time)
            counter_time = 0
        self.start_count_time()                          
#########################################################
#AltGraph and GPSMap update
#########################################################
    def start_count_display(self):
        self.timer.start()
    def display(self):
        if gps_flag.value==1:      
            self.update_gps()
            gps_flag.value = 0
#            print('display gps')
        if alt_flag.value==1:      
            self.update_alt()
            alt_flag.value = 0
#            print('display AltGraph')
        self.start_count_display()
    
    def update_gps(self):
        self.ui.label.setPixmap(QtGui.QPixmap("/tmp/haha.tiff"))        
    def update_alt(self):
        self.ui.label_2.setPixmap(QtGui.QPixmap("/tmp/chart.tiff"))        
#########################################################
#GPS data get
#parse*()           :divide GPS raw data
#degree to decimal():ido,keido data convert to decimal
#########################################################
ser = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )

def parse_GPRMC(data,data_time,data_ido,data_keido,time_flag):
    data1=data.split(',')
    dict = {
        'fix_time':data1[1],
        'validity':data1[2],
        'latitude':data1[3],
        'latitude_hemisphere':data1[4],
        'longitude':data1[5],
        'longitude_hemisphere':data1[6],
        'speed':data1[7],
        'true_course':data1[8],
        'fix_date':data1[9],
        'variation':data1[10],
        'variation_e_w':data1[11],
        'checksum':data1[12]
    }
    dict['decimal_latitude']=degrees_to_decimal(dict['latitude'],dict['latitude_hemisphere'])
    dict['decimal_longitude']=degrees_to_decimal(dict['longitude'],dict['longitude_hemisphere'])
    data_t = float(str(dict['fix_time']))
    data_i = float(str(dict['decimal_latitude']))
    data_k = float(str(dict['decimal_longitude']))    
    print(time_flag.value)
    if time_flag.value == 0:
        data_time.value  = data_t
        data_ido.value   = data_i
        data_keido.value = data_k
        time_flag.value = 1
    print(data_t)
    return dict

def parse_GPGGA(data,data_time,data_ido,data_keido,time_flag):
    data1=data.split(',')
    dict = {
        'fix_time':data1[1],
        'latitude':data1[2],
        'latitude_hemisphere':data1[3],
        'longitude':data1[4],
        'longitude_hemisphere':data1[5]
    }
    dict['decimal_latitude']=degrees_to_decimal(dict['latitude'],dict['latitude_hemisphere'])
    dict['decimal_longitude']=degrees_to_decimal(dict['longitude'],dict['longitude_hemisphere'])
    data_t = float(str(dict['fix_time']))
    data_i = float(str(dict['decimal_latitude']))
    data_k = float(str(dict['decimal_longitude']))    
    if time_flag.value == 0:
        data_time.value = data_t
        data_ido.value   = data_i
        data_keido.value = data_k
        time_flag.value = 1
    print(data_t)
    return dict

def degrees_to_decimal(data, hemisphere):
    try:
        decimalPointPosition = data.index('.')
        degrees = float(data[:decimalPointPosition-2])
        minutes = float(data[decimalPointPosition-2:])/60
        output= degrees + minutes
        if hemisphere is 'N' or hemisphere is 'E':
            return output
        if hemisphere is 'S' or hemisphere is 'W':
            return -output
    except:
        return ''
#########################################################
#GPS picture make
#########################################################        
def gps_main(flag,data_time,data_ido,data_keido,time_flag):
#    keido = 139.54000
#    ido = 35.6560000
#    keido = 138.6303940
#    ido = 35.1246588
    keido2 = 139.54000
    ido2 = 35.6560000
#UEC
#    file_name='/home/pi/work/Python/wings18/map.png'
#fuji
    file_name='/home/pi/work/Python/wings18/map_fuji.png'
#japan
#    file_name='/home/pi/work/Python/wings18/map_japan.png'

    bg_img=None
    bg_img = Image.open(file_name)
    x,y = 0,0
    bmap = Basemap(
        projection='merc',
        llcrnrlat=minlat,
        urcrnrlat=maxlat,
        llcrnrlon=minlon,
        urcrnrlon=maxlon,
        lat_ts=0,
        resolution='l'
    )

    bmap.imshow(bg_img, origin='upper')

    while True:
        line1=ser.readline().decode('utf-8')
        writer2.writerow(line1)
        if "GPRMC" in line1:
            try:
                gpsdata = parse_GPRMC(line1,data_time,data_ido,data_keido,time_flag)
                ser.flushInput()
            except:
                print('dont get GPRMC data ')
            if gpsdata['validity']=='A':
                try:
                    ido=gpsdata['decimal_latitude']
                    keido=gpsdata['decimal_longitude']
                except:
                    print('valibility is A.But dont get GPRMC data')
        if "GPGGA" in line1:
            try:
                gpsdata2 = parse_GPGGA(line1,data_time,data_ido,data_keido,time_flag)
                ido = gpsdata2['decimal_latitude']
                keido = gpsdata2['decimal_longitude']
                ser.flushInput()
            except:
                print("dont get GPGGA data")
        try:
            ido2 = float(ido)
            keido2 =float(keido)
        except:
            print('ido or keido data broken')
        if ((keido2>minlon) and (keido2<maxlon) and (ido2>minlat) and (ido2<maxlat) ):
            x,y=bmap(keido2,ido2)
        bmap.plot(x,y,'m.',markersize=3)
#        print('GPS data plot')
        if (flag.value==0):
#            print('make GPS Map picture')
            plt.savefig('/tmp/haha.tiff',bbox_inches='tight',pad_inches=0.0,dpi=120)            
            flag.value=1

 
#########################################################
#debug count
#########################################################
#def my_count(debug_count):
#    while 1==1:
#        sleep(0.1)
#        debug_count.value += 1
#        if debug_count.value==500:
#            debug_count.value = 0

#########################################################
#AltGraph make function
#########################################################
def my_chart(flag2,data_alt,data_flag):
    fig=plt.figure()
    ax=fig.add_subplot(1,1,1)
    t = np.zeros(10)
    y = np.full(10,1000)
    x = 0
    data = 1000
    ax.set_xlabel('Times[not s]')
    ax.set_ylabel('Alt[mm]')
    ax.set_ylim(0, 255)
    while 1==1:
        x += 1
        if (data_flag.value == 1):
            data = data_alt.value
        data = data
        t = np.append(t,x)        
        t = np.delete(t,0)
        y = np.append(y,data)
        y = np.delete(y,0)        
        ax.set_xlim(min(t),max(t))
        ax.plot(t,y)
        if (flag2.value==0):
#            print('make AltGraph picture')
            plt.savefig('/tmp/chart.tiff',bbox_inches='tight',pad_inches=0.0,dpi=20)            
            flag2.value=1
#########################################################
#GUI start function
#########################################################
def main_disp(gps_flag,alt_flag,data_cadence,data_alt,data_speed,data_rudder,data_elvator,data_flag):
    app = QtGui.QApplication(sys.argv)
    form = Example(gps_flag,alt_flag,data_cadence,data_alt,data_speed,data_rudder,data_elvator,data_flag)
    form.showFullScreen()
    sys.exit(app.exec_())
    #app.exec_()
#########################################################
#exit command (Ctrl-C)
#########################################################
def handler(signal, frame):
    print('barusu')
    sys.exit(0)
########################################################
#main program
#########################################################
def read_pitch_value():
     global address
     address = 0x25         
     try:
         bus.write_byte(address,0)
         time.sleep(0.01)
         x_l = bus.read_byte(address)
         bus.write_byte(address,1)
         time.sleep(0.01)
         x_m = bus.read_byte(address)
         pitch_x = ((x_m << 8)| x_l)
         bus.write_byte(address,2)
         time.sleep(0.01)
         y_l = bus.read_byte(address)
         bus.write_byte(address,3)
         time.sleep(0.01)
         y_m = bus.read_byte(address)
         pitch_y = ((y_m << 8)| y_l)
     except:
         pitch_x = 0
         pitch_y = 0
     finally:
        listData.append(pitch_x)
        listData.append(pitch_y)             
        return (pitch_x, pitch_y)

def read_cadence_value():
     global address
     address = 0x21
     try:
         bus.write_byte(address,0xF0)
         time.sleep(0.01)
         data = bus.read_byte(address)
     except:
         data = 0xff
     finally:
        listData.append(data)             
        return data 

def read_speed_value():
    global address
    address = 0x11
    try:
        bus.write_byte(address,0xF0)
        time.sleep(0.01)
        data_1=bus.read_byte(address)
        
        bus.write_byte(address,0XF1)
        time.sleep(0.01)
        data_2=bus.read_byte(address)

        data3=((data_2 << 8)| data_1)
        data = data3/10
    except:
        data = 0x11

    finally:
        return data

def read_alt_value():
   global address
   address = 0x23
   try:
        bus.write_byte(address,0)
        time.sleep(0.01)
        data_1 = bus.read_byte(address)
        print("data1")
        print(data_1)
        
        bus.write_byte(address,1)
        time.sleep(0.01)
        data_2 = bus.read_byte(address)
        print("data2")
        print(data_2)
        
        data = ((data_2 << 8)| data_1)
        print("data")
        print(data)
   except:
        data = 0x11
   finally:
        return data
    
if __name__ == "__main__":
    signal.signal(signal.SIGINT, handler)
    gps_flag     = Value('i',0,lock=True)
    alt_flag    = Value('i',0,lock=True)
    data_time    = Value('f',0,lock=True)
    data_ido     = Value('f',0,lock=True)
    data_keido   = Value('f',0,lock=True)
    data_alt    = Value('i',0,lock=True)
    data_cadence    = Value('i',0,lock=True)
    data_speed   = Value('f',0,lock=True)
    data_rudder   = Value('i',0,lock=True)
    data_elvator = Value('i',0,lock=True)
    data_flag    = Value('i',0,lock=True)
    time_flag    = Value('i',0,lock=True)
    gps_process=Process(target=gps_main,args=(gps_flag,data_time,data_ido,data_keido,time_flag))
    gps_process.start()
    AltGraph_process=Process(target=my_chart,args=(alt_flag,data_alt,data_flag))  
    AltGraph_process.start()
    display_process=Process(target=main_disp,args=(gps_flag,alt_flag,data_cadence,data_alt,data_speed,data_rudder,data_elvator,data_flag))
    display_process.start()

#mainlagopus-system
#to play sound on raspberry pi, 'aplay /sound file directparth'
    while(True):
        rvocount += 1 #counter val for random voice play
        count += 1 #counter val for sensor communication(SPI, I2C)
        if rvocount == 1000000:
            vonum = str(random.randint(0,N))
            #vnum = random.randint(0, N)
            #vonum = str(vnum)
            basepaths = '/home/pi/Desktop/voice/v'
            #ext = '.wav'
            #aplay = 'aplay '
            vopath = 'aplay ' + basepaths + vonum + '.wav' #'aplay /home/pi/Desktop/voice/vN.wav'
            os.system(vopath)
            rvocount = 0
        if count == 100:
            data_flag.value=1
            writer.writerow(listData)
            listData = []
            count = 0
        elif count == 90:
            sensor = "alt"
            alt_data = read_alt_value()
            print(alt_data)
        elif count == 75:
            sensor = "speed"
            speed_data = read_speed_value()
        elif count == 50:
            sensor = "pitch"
            val_rudder, val_elvator = read_pitch_value()
            if val_rudder<20000:
                if (val_rudder != ( data_rudder.value + 128 ) ):
                    data_rudder.value = val_rudder

            if val_elvator<20000:
                if (val_elvator != ( data_elvator.value + 128 ) ):
                    data_elvator.value = val_elvator
#            data_rudder.value=val_rudder
#            data_elvator.value=val_elvator
#            print(data_rudder.value)
#            print(data_elvator.value)            
        elif count == 25:
            sensor = "cadence"
            data_cadence.value = read_cadence_value()
#            print(data_cadence.value)
        elif count == 10:
            time_data = datetime.datetime.now()
#            print(time_data)
            listData.append(time_data)
            listData.append(data_time.value)
            listData.append(data_ido.value)            
            listData.append(data_keido.value)
            if time_flag.value == 1:
                time_flag.value = 0



########################################################
#sample code
##########################################################
#    time.sleep(10)         #
#    print('10!!!!!!!!!')   #
#    data_cadence.value=70     #
#    data_alt.value=500    #
#    data_rudder.value = 6000
#    data_elvator.value=10000
#    data_flag.value = 1    #
#
#    time.sleep(7)          #
#    print('17!!!!!!!!!')   #
#    data_cadence.value=20     #
#    data_alt.value=200    #
#    data_rudder.value = 7000
#    data_elvator.value=4000
#    data_flag.value = 1    #
#########################################################
