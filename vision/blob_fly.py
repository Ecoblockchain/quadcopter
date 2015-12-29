import cv2
import numpy as np
import math

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
import time

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

throttle = MAX/2 - 100
#EN = MID
EN = MID + 50
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

# Strength
S = .03

# Y strength is SY * Y
SY = 1

# Z strength (independent of S)
ZS = .05

VCAP = 50

TC = .1

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
            x = TX
            y = TY
            z = pz
            xv = 0
            yv = 0
            zv = 0
            pxv = 0
            pyv = 0
            pzv = 0
            xa = 0
            ya = 0
            za = 0
            pt = t
            pa = aileron
            pe = elevator
            pth = throttle
            print 'TARGET =', TX, TY
            continue

        # Time
        dt = t - pt
        alpha = dt / (TC + dt)

        # Z
        zm = math.sqrt(bs)
        z = alpha * zm + (1 - alpha) * z
        dz = TZ - z

        # Z'
        tzv = dz
        zvm = (z - pz) / dt
        zv = alpha * zvm + (1 - alpha) * zv

        # Z''
        tza = (tzv - zv) / TA
        zam = (zv - pzv) / dt
        za = alpha * zam + (1 - alpha) * za

        throttle += (tza - za) * ZS
        throttle = limit(throttle, MIN, MAX)

        #if bs < TH:
        #throttle += (TH - bs) / 80
        #else:
        #    throttle -= (bs - TH) / 40

        # X
        xm = bpt[0]
        x = alpha * xm + (1 - alpha) * x
        dx = TX - x
#        aileron += dx * S

        # X'
        txv = dx  # target velocity takes us back to TX in 1s
        txv = vlimit(txv, VCAP)
        xvm = (x - px) / dt
        xv = alpha * xvm + (1 - alpha) * xv
#        aileron += (txv - xv) * S

        # X''
        txa = (txv - xv) / TA # target acceleration takes us to txv in .2s
        xam = (xv - pxv) / dt
        xa = alpha * xam + (1 - alpha) * xa
        aileron += (txa - xa) * S

        # damping
        #ap = (aileron - pa) / (t - pt)
        #aileron -= ap / 50 * S

#        aileron = limit(aileron, AN - LIMIT, AN + LIMIT)
        aileron = limit(aileron, MIN, MAX)

        # Y
        ym = bpt[1]
        y = alpha * ym + (1 - alpha) * y
        dy = TY - y
#        elevator -= dy * S * SY

        # Y'
        tyv = dy
        tyv = vlimit(tyv, VCAP)
        yvm = (y - py) / dt
        yv = alpha * yvm + (1 - alpha) * yv
#        elevator -= (tyv - yv) * S * SY
        
        # Y''
        tya = (tyv - yv) / TA # target acceleration takes us to txv in .2s
        yam = (yv - pyv) / dt
        ya = alpha * yam + (1 - alpha) * ya
        elevator -= (tya - ya) * S * SY

        # damping
        #ep = (elevator - pe) / (t - pt)
        #elevator -= ep / 50 * S
        
        elevator = limit(elevator, MIN, MAX)

        sende = int(elevator)
        senda = int(aileron)

        print bs, tza, za, throttle, '|', x, txv, xvm, xv, txa, xa, senda, '|', y, tyv, yv, yvm, tya, ya, sende

        # Prev
        px = x
        py = y
        pz = z
        pt = t
        pxv = xv
        pyv = yv
        pzv = zv
        pa = aileron
        pe = elevator
        pth = throttle

    send(int(throttle), MID, sende, senda)
    k = cv2.waitKey(1)
    if k != -1:
        send(0, MID, MID, MID)
        break

cap.release()
cv2.destroyAllWindows()
