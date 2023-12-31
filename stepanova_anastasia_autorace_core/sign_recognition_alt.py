import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import os.path 
import sys
from ament_index_python.packages import get_package_share_directory

def object_Finder(desclist, background, matcher, orb, tmpbackground, min_area=20000, max_area= 50000): # add a parameter for minimum area
    temp = []
    list_of_sign = []
    for el in desclist:
        while True:
            kp4, desc4 = orb.detectAndCompute(tmpbackground, None)
            matches = matcher.match(el[1], desc4)
            matches = sorted(matches, key=lambda x: x.distance)
            good_matches = [m for m in matches if m.distance < 80]
            if len(good_matches) >= 4: # check if there are at least 4 good matches
                src_pts = np.float32([el[0][i.queryIdx].pt for i in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp4[i.trainIdx].pt for i in good_matches]).reshape(-1, 1, 2)
                line, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                h, w = el[2].shape
                pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
                
                if isinstance(line, np.ndarray):
                    print("line is a numpy array")
                else:
                    break
                # Before calling cv2.perspectiveTransform:
                assert isinstance(pts, np.ndarray), "pts must be a numpy array"
                assert pts.ndim == 3, "pts must be a 3-dimensional array"
                assert pts.shape[2] in [2, 3], "The length of the third dimension of pts must be 2 or 3"

                assert isinstance(line, np.ndarray), "line must be a numpy array"
                assert line.shape in [(3, 3), (4, 4)], "line must be a 3x3 or 4x4 matrix"

                if pts.shape[2] == 2:
                    assert line.shape == (3, 3), "For 2D points, line must be a 3x3 matrix"
                elif pts.shape[2] == 3:
                    assert line.shape == (4, 4), "For 3D points, line must be a 4x4 matrix"

                # Now you can call cv2.perspectiveTransform safely
                try:
                    dst = cv2.perspectiveTransform(pts, line)
                except cv2.error as e:
                    print(f"OpenCV error: {e}")
                    sys.exit(1)
                
                
                #dst = cv2.perspectiveTransform(pts, line)
                area = cv2.contourArea(dst) # calculate the area of the detected sign
                is_convex = cv2.isContourConvex(dst) # check if the contour is convex
                if area > min_area and area < max_area and is_convex: # check if the area is larger than the minimum area and the contour is convex
                    #tmpbackground = cv2.fillPoly(tmpbackground, pts=[np.int32(dst)], color=(0, 0, 0))
                    #plt.imshow(tmpbackground)
                    temp.append([np.int32(dst)])
                    list_of_sign.append(el[3])
                    #print(f"Detected sign: {el[3]}") # print the name of the detected sign
                break
            else:
                break
    return temp, list_of_sign
i = 0
signs = [['traffic_intersection1'], ['traffic_left1', 'traffic_right1'], ['traffic_construction1', 'traffic_parking1', 'tunnel1', 'pedestrian_crossing_sign1']]
def lab():
    global i
    orb = cv2.ORB_create()
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    desclist = []

    pkg = get_package_share_directory('stepanova_anastasia_autorace_core')

    for sign in signs[i]:
        full_path =  os.path.join(pkg, 'signes', str(sign) + '.png')
        gsign = cv2.imread(full_path, cv2.IMREAD_GRAYSCALE)
       
        #cv2.imshow('sign', gsign)
        #cv2.waitKey(0)

        #gsign = cv2.cvtColor(sign_img, cv2.COLOR_BGR2GRAY)
        
        kp, desc = orb.detectAndCompute(gsign, None)
        desclist.append([kp, desc, gsign, sign]) # add the name of the sign to the list
        
    return desclist, matcher, orb

    
def recognition(img, desclist, matcher, orb):
    global i
    background = img
    gback = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)
    out = np.copy(background)    
    lst1, list_of_sign = object_Finder(desclist, gback, matcher, orb, out, min_area=5000)
    if(len(list_of_sign) != 0):
        i+=1
        return i, lst1, list_of_sign[0]
    return i, lst1, 'no sign'
