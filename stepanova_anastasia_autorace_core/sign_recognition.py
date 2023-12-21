import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import os.path 
import sys
from ament_index_python.packages import get_package_share_directory
dec = cv2.ORB_create()

bf = cv2.BFMatcher()

stages = 0
#['traffic_intersection']
signs = [['traffic_intersection'], ['traffic_left', 'traffic_right'], ['traffic_construction'], ['traffic_parking'], ['pedestrian_crossing_sign'], ['tunnel'], ]

PATH_TEMPLATE = "signes"
images = []
classNames = []
classFound = [0, 0]
path_list = get_package_share_directory('stepanova_anastasia_autorace_core')

def update_classes():
    global stages
    global classNames
    global images
    images = []
    classNames = []
    for className in signs[stages]:
        full_path =  os.path.join(path_list, PATH_TEMPLATE, str(className) + '.jpg')
        currentImage = cv2.imread(full_path, 0)
        images.append(currentImage)
        classNames.append(className.split(".")[0])

update_classes()


def draw(current_image, sign_image, kp1, kp2, goods, matchesMask):
    draw_params = dict(matchColor = (0,255,0), singlePointColor = None, matchesMask = matchesMask, flags = 2)
    good = [m[0] for m in goods]
    img3 = cv2.drawMatches(sign_image, kp2, current_image, kp1, good,None,**draw_params)
    return img3
    
# Create Descriptor for each image class
def createDesc(images):
    descList = []
    keypointList = []
    for img in images:
        keypoint, descriptor = dec.detectAndCompute(img, None)
        keypointList.append(keypoint)
        descList.append(descriptor)
    return descList, keypointList

descList, keypointList = createDesc(images)

def checkMatch(img_keypoint, img_descriptor, descList, keypointList, point_limit, distant):
    matchListLen = []
    matchList = []
    classIndices = []  # List to keep track of class indices
    global_coor = []
    for i, desc in enumerate(descList):  # Add index to enumerate
        if (img_descriptor is not None):
            matches = bf.knnMatch(desc, img_descriptor, k=2)
            goodMatches = []
            goods = []
            coordinates = []
            for m, n in matches:
                if m.distance < 0.70 * n.distance and m.queryIdx < len(img_keypoint):
                    goodMatches.append([m])
                    goods.append([m, n])
                    coordinates.append(img_keypoint[m.queryIdx].pt)
            if(len(goodMatches) >= point_limit):
                while(len(goodMatches) >= 1):
                    centrlenght = []
                    res_centrlenght = []
                    for j in range(len(goodMatches)):
                        new_coordinates = coordinates[:j] + coordinates[j+1:]
                        centroid = np.mean(new_coordinates, axis=0)
                        centrlenght.append([np.linalg.norm(np.array(img_keypoint[m[0].queryIdx].pt) - centroid) for m in goodMatches])
                    res_centrlenght = [np.mean(column) for column in zip(*centrlenght)]
                    if(max(res_centrlenght) > distant):
                        goodMatches.pop(res_centrlenght.index(max(res_centrlenght)))
                        goods.pop(res_centrlenght.index(max(res_centrlenght)))
                        coordinates.pop(res_centrlenght.index(max(res_centrlenght)))
                    else:
                        break             
            matchListLen.append(len(goodMatches))
            matchList.append(goodMatches)
            global_coor.append(coordinates)
            classIndices.append(i)
    return matchList, matchListLen, global_coor, classIndices  # Return classIndices
    


# Find detected class
def getClass(current_image, sign_image, matches, matchLen, classIndices, img_keypoint, keypointList, threshold):
    finalClass = -1
    matchesMask = None
    maxMatchIndex = -1
    if (len(matchLen) != 0):
        maxMatchCount = max(matchLen)
        if maxMatchCount > threshold:
            maxMatchIndex = matchLen.index(maxMatchCount)
            finalClass = classIndices[maxMatchIndex]
            src_pts = np.float32([keypointList[finalClass][m[0].queryIdx].pt for m in matches[maxMatchIndex] ]).reshape(-1,1,2)
            dst_pts  = np.float32([img_keypoint[m[0].trainIdx].pt for m in matches[maxMatchIndex] ]).reshape(-1,1,2)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()
            h,w = sign_image[finalClass].shape
            pts = np.float32([ [0,0],[0,h],[w,h],[w,0] ]).reshape(-1,1,2)
            if M is not None and pts is not None and pts.shape[2] == M.shape[0]:
                dst = cv2.perspectiveTransform(pts,M)
                current_image = cv2.polylines(current_image,[np.int32(dst)],True,255,3, cv2.LINE_AA)
    return finalClass, matchesMask, current_image, maxMatchIndex
    

def detectTrafficSigns(img, threshold):
    global stages
    global descList
    global keypointList
    global images
    classFound = [0] * len(classNames)
    currentImage = np.copy(img)
    gray_image = cv2.cvtColor(currentImage, cv2.COLOR_BGR2GRAY)
    img_keypoint, img_descriptor = dec.detectAndCompute(gray_image, None)
    matchRaw, matchLen, coordinates, classIndices = checkMatch(img_keypoint, img_descriptor, descList, keypointList, threshold + 5, 50)
    classID, matchesMask, currentImage, maxMatchIndex = getClass(gray_image, images, matchRaw, matchLen, classIndices, img_keypoint, keypointList, threshold)
    findedClass = 'none'
    if (classID != -1):
        stages = (stages + 1) % 6
        findedClass = classNames[classID]
        classFound[classID] += 1
        cv2.putText(currentImage, f'Detected: {classNames[classID]} deb: {matchLen}', (img.shape[1] // 2, img.shape[0] // 2), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
        points = np.array(coordinates[maxMatchIndex], dtype=np.int32)
        if points.size > 0:
            x, y, w, h = cv2.boundingRect(points)
            cv2.rectangle(currentImage, (x, y), (x + w, y + h), (0, 255, 0), 2)
        if(matchesMask is not None):
            current_image = draw(currentImage, images[classID], img_keypoint, keypointList[classID], matchRaw[maxMatchIndex], matchesMask)
        update_classes()
        descList, keypointList = createDesc(images)
        
    else:
        cv2.putText(currentImage, f"Undetected deb: {matchLen}", (img.shape[1] // 2, img.shape[0] // 2), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

    return currentImage, findedClass
  
        
def recognition(img, depth_image):
    #mask = depth_image > 128
    #img[mask] = [0, 0, 0]
    cur_img, findedClass = detectTrafficSigns(img, 10)
    return  cur_img, findedClass, classNames
