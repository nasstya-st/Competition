import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import os.path 
import sys
from ament_index_python.packages import get_package_share_directory

i = 0
signs = [['traffic_intersection'], ['traffic_left', 'traffic_right'], ['traffic_construction', 'traffic_parking', 'tunnel', 'pedestrian_crossing_sign']]
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
    
orb = cv2.ORB_create(100)

PATH_TEMPLATE = "signes"
images = []
classNames = []
classFound = [0, 0]
path_list = get_package_share_directory('stepanova_anastasia_autorace_core')

def update_classes():
    global classNames
    classNames = []
    for className in signs[i]:
        full_path =  os.path.join(path_list, PATH_TEMPLATE, str(className) + '.png')
        currentImage = cv2.imread(full_path, 0)
        images.append(currentImage)
        classNames.append(className.split(".")[0])

update_classes()

# Create Descriptor for each image class
def createDesc(images):
    descList = []
    keypointList = []
    for img in images:
        keypoint, descriptor = orb.detectAndCompute(img, None)
        keypointList.append(keypoint)
        descList.append(descriptor)
    return descList, keypointList

descList, keypointList = createDesc(images)
print(len(keypointList))
# Create matcher
def checkMatch(img, descList, keypointList):
    img_keypoint, img_descriptor = orb.detectAndCompute(img, None)
    bf = cv2.BFMatcher()
    matchListLen = []
    matchList = []
    coordinates = []
    classIndices = []
    global_coor = []
    for i, desc in enumerate(descList): 
        if (img_descriptor is not None):
            matches = bf.knnMatch(desc, img_descriptor, k=2)
            goodMatches = []
            for m, n in matches:
                if m.distance < 0.75 * n.distance:
                    goodMatches.append([m])
                    if img_keypoint and 0 <= m.queryIdx < len(img_keypoint):
                        coordinates.append(img_keypoint[m.queryIdx].pt)
            if(len(goodMatches) >= 10):
                matchListLen.append(len(goodMatches))
                matchList.append(goodMatches)
                classIndices.append(i)  
                global_coor = global_coor + coordinates
            coordinates = []
    return matchList, matchListLen, global_coor, classIndices

# Find detected class
def getClass(matches, classIndices, threshold=15):
    finalClass = -1
    if (len(matches) != 0):
        maxMatchCount = max(matches)
        print([classNames[i] for i in classIndices])
        if maxMatchCount > threshold:
            maxMatchIndex = matches.index(maxMatchCount)
            finalClass = classIndices[maxMatchIndex]
    return finalClass

def remove_outliers(points, threshold):
    if points.ndim != 2 or points.size == 0:
        return points

    # Calculate the centroid of the points
    centroid = np.mean(points, axis=0)
    
    # Calculate the Euclidean distance from each point to the centroid
    distances = np.linalg.norm(points - centroid, axis=1)
    
    # Create a mask for points within the threshold
    mask = distances < threshold
    
    # Return only the points within the threshold
    return points[mask]


def detectTrafficSigns(img, threshold=15):
    global i
    classFound = [0] * len(classNames)
    currentImage = np.copy(img)
    gray_image = cv2.cvtColor(currentImage, cv2.COLOR_BGR2GRAY)
    matchRaw, matchLen, coordinates, classIndices = checkMatch(gray_image, descList, keypointList)
    classID = getClass(matchLen, classIndices, threshold)
    findedClass = 'No sing'
    if (classID != -1):
        i+=1
        update_classes()
        findedClass = classNames[classID]
        classFound[classID] += 1
        cv2.putText(currentImage, f'Detected: {classNames[classID]} deb: {matchLen}', (img.shape[1] // 2, img.shape[0] // 2), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
        # Convert your list of points to a numpy array of type int32
        points = np.array(coordinates, dtype=np.int32)

        # Remove outliers
        filtered_points = remove_outliers(points, threshold=100)

        # Convert filtered points to the correct type
        filtered_points = np.array(filtered_points, dtype=np.int32)

        # Calculate the bounding rectangle only if there are points left after outlier removal
        if filtered_points.size > 0:
            x, y, w, h = cv2.boundingRect(filtered_points)

            # Draw the rectangle on your image
            cv2.rectangle(currentImage, (x, y), (x + w, y + h), (0, 255, 0), 2)

    else:
        cv2.putText(currentImage, f"Undetected deb: {matchLen}", (img.shape[1] // 2, img.shape[0] // 2), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
    #cv2.imshow(f'{filename}', img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    #print(f"Detected {sum(classFound)} traffic signs:")
    #for i, count in enumerate(classFound):
        #print(f"{classNames[i]}: {count}")
    return currentImage, findedClass


    
def recognition(img):
    cur_img, findedClass = detectTrafficSigns(img, threshold=15)
    return  cur_img, findedClass, classNames
