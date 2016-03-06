import cv2
import math
import numpy as np
import time

log = open("log." + str(time.time()), "w")
def wlog(*args):
    s = ''
    for arg in args:
        if s:
            s += ' '
        s += str(arg)
    log.write(s + '\n')

#cap = cv2.VideoCapture("cap4.avi")
cap = cv2.VideoCapture(1)

ret, frame = cap.read()
assert ret

params = cv2.SimpleBlobDetector_Params()
params.filterByConvexity = True
params.minConvexity = 0.0
# Don't pick up dusty footprints :-)
params.maxThreshold = 140
#params.filterByColor = True
#params.blobColor = 255
detector = cv2.SimpleBlobDetector_create(params)

def findCopter(frame):
    frame = frame[160:800, 750:1150]
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    frame = 255 - frame
    keypoints = detector.detect(frame)
    bs = 0
    bpt = None
    for k in keypoints:
        print k.size, k.pt
        if k.size > bs:
            bs = k.size
            bpt = k.pt
    frame2 = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255),
                               cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('frame', frame2)

    return (bpt, bs)

findCopter(frame)
cv2.waitKey(100)

import serial
import sys

ser = serial.Serial('/dev/cu.usbserial-DC008KDX', 115200)

while True:
    x = ser.read()
    sys.stdout.write(x)
    sys.stdout.flush()
#    print ord(x), x
    if x == 'Z':
        break

print "Ready!"

def send(throttle, rudder, elevator, aileron):
    echo()
#    print "t =", throttle, "r =", rudder, "e =", elevator, "a =", aileron
    send16(aileron)
    send16(elevator)
    send16(throttle)
    send16(rudder)
    ack()

def send16(n):
    send8((n >> 8) & 0xff)
    send8(n & 0xff)

def send8(n):
    ser.write(chr(n))

def echo():
    while ser.inWaiting():
        t = ser.read()
        sys.stdout.write(t)
        sys.stdout.flush()

def ack():
    while True:
        t = ser.read()
        sys.stdout.write(t)
        if t == '+':
            break

MAX = 1000
MIN = 0
MID = 500

# very similar to limit!
def vlimit(x, c):
    if x < -c:
        x = -c
    elif x > c:
        x = c
    return x

def limit(x, min, mx):
    if x < min:
        return min
    elif x > max:
        return max
    return x

# ARM (Don't - now auto-armed by the proxy)
#send(MIN, MID, MID, MID)
#time.sleep(.1)
#send(MAX, MID, MID, MID)
#time.sleep(.1)
#send(MIN, MID, MID, MID)
#time.sleep(.1)

# Wait for first craft to arm
t = time.time() + 2
while time.time() < t:
    echo()

# Copter is allegedly ready :-)

TN = MID - 120
throttle = TN
EN = MID
#EN = MID + 50
elevator = EN
#AN = MID
AN = MID + 40
aileron = AN
#LIMIT = 400

TZ = math.sqrt(40)
TX = None
TY = None
TXV = 0
TYV = 0

TA = 1

# PID parameters
KP = 1.15
KI = .1
KD = .5

KPZ = 5
KIZ = 2
KDZ = 0

# Strength
S = .03

# Y strength is SY * Y
SY = 1

# Z strength (independent of S)
ZS = .05

VCAP = 50

TC = .01
TCZ = .1

while True:
    ret, frame = cap.read()
    t = time.time()
    (bpt, bs) = findCopter(frame)

    if bpt is not None:
        if TX is None:
            TX = bpt[0]
            TY = bpt[1]
            px = TX
            py = TY
            pz = math.sqrt(bs)
            pdx = 0
            pdy = 0
            pdz = TZ - pz
            idx = 0
            idy = 0
            idz = 0
            ddz = 0
            x = TX
            y = TY
            z = pz
            pt = t
            print 'TARGET =', TX, TY
            continue

        # Time
        dt = t - pt
        alpha = dt / (TC + dt)
        alphaz = dt / (TCZ + dt)

        # Z
        zm = math.sqrt(bs)
        z = alphaz * zm + (1 - alphaz) * z
        dz = TZ - z

        # I(dz), D(dz)
        idz += dz * dt
        # D(dz) needs smoothing
        ddzm = (dz - pdz) / dt
        ddz = alphaz * ddzm + (1 - alphaz) * ddz

        throttle = TN + KPZ * dz + KIZ * idz + KDZ * ddz
        throttle = limit(throttle, MIN, MAX)
        
        # X
        xm = bpt[0]
        x = alpha * xm + (1 - alpha) * x
        dx = TX - x

        # I(dx), D(dx)
        idx += dx * dt
        ddx = (dx - pdx) / dt

        aileron = AN + KP * dx + KI * idx + KD * ddx
        aileron = limit(aileron, MIN, MAX)

        # Y
        ym = bpt[1]
        y = alpha * ym + (1 - alpha) * y
        dy = TY - y

        # I(dy), D(dy)
        idy += dy * dt
        ddy = (dy - pdy) / dt

        elevator = EN - KP * dy - KI * idy - KD * ddy
        elevator = limit(elevator, MIN, MAX)

        sende = int(elevator)
        senda = int(aileron)

        print bs, throttle, '|', x, senda, '|', y, sende

        wlog(t, dt,
             xm, dx, idx, ddx, aileron,
             ym, dy, idy, ddy, elevator,
             zm * 100, dz * KPZ , idz * KIZ, ddz * KDZ, throttle)

        # Prev
        px = x
        py = y
        pz = z
        pdx = dx
        pdy = dy
        pdz = dz
        pt = t

    send(int(throttle), MID, sende, senda)
    k = cv2.waitKey(1)
    if k != -1:
        send(0, MID, MID, MID)
        break

cap.release()
cv2.destroyAllWindows()
