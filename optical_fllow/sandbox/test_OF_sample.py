#!/usr/bin/python
#coding: utf-8

# 参考
# http://labs.eecs.tottori-u.ac.jp/sd/Member/oyamada/OpenCV/html/py_tutorials/py_video/py_lucas_kanade/py_lucas_kanade.html


import numpy as np
import cv2

threshold =60
# cap = cv2.VideoCapture('optical_fllow/data/20211207/magenta.avi')
cap = cv2.VideoCapture(4)

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Create some random colors
color = np.random.randint(0,255,(100,3))

# Take first frame and find corners in it
ret, old_frame = cap.read()
old_frame_hsv = cv2.cvtColor(old_frame, cv2.COLOR_BGR2HSV)
old_hue = cv2.extractChannel(old_frame_hsv, 0)
_, old_p = cv2.threshold(old_hue, threshold, 179, cv2.THRESH_BINARY)
# old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(old_p, mask = None, **feature_params)

# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)

while(1):
    ret,frame = cap.read()
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_hue = cv2.extractChannel(frame_hsv, 0)
    _, frame_p = cv2.threshold(frame_hue, threshold, 179, cv2.THRESH_BINARY)

    # calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_p, frame_p, p0, None, **lk_params)

    # Select good points
    try:
        good_new = p1[st==1]
        good_old = p0[st==1]

        # draw the tracks
        for i,(new,old) in enumerate(zip(good_new,good_old)):
            a,b = new.ravel().astype(np.int)
            c,d = old.ravel().astype(np.int)
            mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
            frame = cv2.circle(frame,(a,b),5,color[i].tolist(),-1)
        img = cv2.add(frame,mask)

    except:
        img = frame

    

    cv2.imshow('frame',img)
    cv2.imshow("gray", frame_p)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

    # Now update the previous frame and previous points
    old_p = frame_p.copy()
    p0 = good_new.reshape(-1,1,2)

cv2.destroyAllWindows()
cap.release()