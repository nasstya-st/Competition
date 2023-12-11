import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import os.path 
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
                dst = cv2.perspectiveTransform(pts, line)
                area = cv2.contourArea(dst) # calculate the area of the detected sign
                is_convex = cv2.isContourConvex(dst) # check if the contour is convex
                if area > min_area and area < max_area and is_convex: # check if the area is larger than the minimum area and the contour is convex
                    tmpbackground = cv2.fillPoly(tmpbackground, pts=[np.int32(dst)], color=(0, 0, 0))
                    plt.imshow(tmpbackground)
                    temp.append([np.int32(dst)])
                    list_of_sign.append(el[3])
                    print(f"Detected sign: {el[3]}") # print the name of the detected sign
                break
            else:
                break
    return temp, list_of_sign

def lab():
    signs = ['traffic_construction1', 'traffic_intersection1', 'traffic_left1', 'traffic_right1', 'traffic_parking1', 'tunnel1', 'pedestrian_crossing_sign1']
    orb = cv2.ORB_create()
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    desclist = []
    path = os.path.abspath('signes')
    
    for sign in signs:
        full_path =path + '/' + sign + '.png'
        sign_img = cv2.imread(full_path)

        gsign = cv2.cvtColor(sign_img, cv2.COLOR_BGR2GRAY)
        
        kp, desc = orb.detectAndCompute(gsign, None)
        desclist.append([kp, desc, gsign, sign]) # add the name of the sign to the list
        
    return desclist, matcher, orb

    
def recognition(img, desclist, matcher, orb):
    background = img
    gback = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)
    out = np.copy(background)    
    lst1, list_of_sign = object_Finder(desclist, gback, matcher, orb, out, min_area=5000)
    return list_of_sign
