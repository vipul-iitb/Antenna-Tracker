import serial
from math import *
import RPi.GPIO as GPIO
import time
def cvtpwm(angle):
 m = abs(angle)

 p = (m*1/18)
 return p+2.5
def haversine(lon1, lat1, lon2, lat2):

    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r

def angle_fromc(lon1, lat1, lon2, lat2):
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    dLon = lon2-lon1
    y = sin(dLon) * cos(lat2)
    x = (cos(lat1) * sin(lat2)) - (sin(lat1)*cos(lat2)*cos(dLon))
    brng = (degrees(atan2(y,x)) + 360) % 360
    #brng =  brng   
    return brng

GPIO.setmode(GPIO.BCM)

SERVO1 = 23
SERVO2 = 24
L1 = 4
GPIO.setwarnings(False)
GPIO.setup(SERVO1,GPIO.OUT)
GPIO.setup(SERVO2,GPIO.OUT)
GPIO.setup(L1,GPIO.OUT)
GPIO.output(L1,True)
#pulse width modulation, pin SER @ 50 htz
s1 = GPIO.PWM(SERVO1,50)
s2 = GPIO.PWM(SERVO2,50)

#duty cycle of 7.5
s1.start(2.5)
s2.start(2.5)

ser1 = serial.Serial(
  port='/dev/ttyUSB0',
  baudrate = 57600,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS,
  timeout=2
)
ctr = 0
while True:
 if (not ser1.isOpen()):
  ser1.open()
 s= ser1.readline()
 s = s.split("||")
 #print s
 if (len(s) == 2):
  #print s
  s=s[0]
  if(s.count(".") != 3):
   continue
  s=s.split("|")

 #print len(s)
  if(len(s)==3):
   if(s[0].strip().count(".") != 1):
    continue
   if(s[1].strip().count(".") != 1):
    continue
   if(s[2].strip().count(".") != 1):
    continue
   #time.sleep(1)
   if(ctr == 0):

    lat=float(s[0].strip())
    long=float(s[1].strip())
    alt=float(s[2].strip())
    ctr = 1
    time.sleep(15)
    t_end = time.time() + 15
    print "Discarding values for 15 seconds."
    tim = 1
    while time.time() < t_end:
     #pis = ser1.readline()
     if (tim==1):
      GPIO.output(L1,False)
      tim = 0
     else:
      GPIO.output(L1,True)
      tim = 1
     time.sleep(1)
    GPIO.output(L1,False)
   elif (ctr == 1):
    ctr = 2
    ref_lat=float(s[0].strip())
    ref_long=float(s[1].strip())
    ref_alt=float(s[2].strip())
    print "Calibrated values : " , lat , " " , long , " " , ref_lat , " " , ref_long
   else:
    dist=haversine(long,lat,float(s[1]),float(s[0]))
    if(dist!=0):
      diff_alt = float(s[2])-alt
      ele = degrees(atan((diff_alt/dist)))

      #print s[0] , " " , s[1] , " " , s[2]
    else:
      ele = 0.0
    pan = angle_fromc(ref_long,ref_lat,long,lat) - angle_fromc(ref_long,ref_lat,float(s[1]),float(s[0]))
    print "Elevation : ", ele
    print "\nPan : ", pan
    #ele = float(1/18 * ele)
    #pan = float(1/18 * pan)
    s2.ChangeDutyCycle(cvtpwm(ele))
    s1.ChangeDutyCycle(cvtpwm(pan))
    print cvtpwm(pan)
    time.sleep(2)
 ser1.close()
