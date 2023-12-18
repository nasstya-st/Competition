import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import os.path 
import sys
from ament_index_python.packages import get_package_share_directory
orb = cv2.ORB_create(1000000)
orb1 = cv2.ORB_create(100)
i = 0
signs = [['traffic_intersection'], ['traffic_left', 'traffic_right'], ['traffic_construction'], ['traffic_parking'], ['pedestrian_crossing_sign'], ['tunnel'], ]
orb = cv2.ORB_create()
orb1 = cv2.ORB_create()
def lab():
    global i
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
        keypoint, descriptor = orb1.detectAndCompute(img, None)
        keypointList.append(keypoint)
        descList.append(descriptor)
    return descList, keypointList

descList, keypointList = createDesc(images)

def checkMatch(img, descList, keypointList, threshold=10.0):
    img_keypoint, img_descriptor = orb.detectAndCompute(img, None)
    bf = cv2.BFMatcher()
    matchListLen = []
    matchList = []
    classIndices = []  # List to keep track of class indices
    global_coor = []
    for i, desc in enumerate(descList):  # Add index to enumerate
        if (img_descriptor is not None):
            matches = bf.knnMatch(desc, img_descriptor, k=2)
            img3 = cv2.drawMatches(images[i],keypointList[i],img, img_keypoint,[c for sublist in matches for c in sublist],None, flags=2)


            #Display the image
            #cv2.imshow('Matches', img3)
            #cv2.waitKey(0)
            goodMatches = []
            goods = []
            coordinates = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance and m.queryIdx < len(img_keypoint):
                    goodMatches.append([m])
                    goods.append([m, n])
                    coordinates.append(img_keypoint[m.queryIdx].pt)            
            if(len(goodMatches) >= 15):
                # Remove matches that are far away
                img3 = cv2.drawMatches(images[i],keypointList[i],img, img_keypoint,[c for sublist in  goods for c in sublist],None, flags=2)
                #cv2.imshow('Matches', img3)
                #cv2.waitKey(0)
                while(len(goodMatches) >= 1):
                    centrlenght = []
                    res_centrlenght = []
                    for j in range(len(goodMatches)):
                        new_coordinates = coordinates[:j] + coordinates[j+1:]
                        centroid = np.mean(new_coordinates, axis=0)
                        centrlenght.append([np.linalg.norm(np.array(img_keypoint[m[0].queryIdx].pt) - centroid) for m in goodMatches])
                    res_centrlenght = [np.mean(column) for column in zip(*centrlenght)]
                    #print(res_centrlenght)
                    #print(len(goodMatches))
                    #prop = np.copy(img)
                    #for g in goods:
                        #x, y = img_keypoint[g[0].queryIdx].pt
                        # Now you can use these coordinates to draw a point on the image
                        #prop = cv2.circle(prop, (int(x), int(y)), 5, (0, 255, 0), -1)
                    #cv2.imshow('prop', prop)
                    #cv2.waitKey(0)
                    if(max(res_centrlenght) > threshold):
                        #print(max(res_centrlenght))
                        goodMatches.pop(res_centrlenght.index(max(res_centrlenght)))
                        goods.pop(res_centrlenght.index(max(res_centrlenght)))
                        coordinates.pop(res_centrlenght.index(max(res_centrlenght)))
                    else:
                        break
                    prop = np.copy(img)
                    #for g in goods:
                        #x, y = img_keypoint[g[0].queryIdx].pt
                        # Now you can use these coordinates to draw a point on the image
                        #prop = cv2.circle(prop, (int(x), int(y)), 5, (0, 255, 0), -1)
                    #cv2.imshow('prop', prop)
                    #cv2.waitKey(0)
                matchListLen.append(len(goodMatches))
                matchList.append(goodMatches)
                classIndices.append(i)  # Append the class index
                global_coor.append(coordinates)
                img3 = cv2.drawMatches(images[i],keypointList[i],img, img_keypoint,[c for sublist in  goods for c in sublist],None, flags=2)
                #cv2.imshow('Matches', img3)
                #cv2.waitKey(0)
    
            coordinates = []
    return matchList, matchListLen, global_coor, classIndices  # Return classIndices
    


# Find detected class
def getClass(matches, classIndices, threshold=10):
    finalClass = -1
    if (len(matches) != 0):
        maxMatchCount = max(matches)
        print([classNames[i] for i in classIndices])
        if maxMatchCount > threshold:
            maxMatchIndex = matches.index(maxMatchCount)
            finalClass = classIndices[maxMatchIndex]
    return finalClass
    
    


def detectTrafficSignsOnDataset(img, threshold=15):
    global i
    classFound = [0] * len(classNames)
    currentImage = np.copy(img)
    gray_image = cv2.cvtColor(currentImage, cv2.COLOR_BGR2GRAY)
    matchRaw, matchLen, coordinates, classIndices = checkMatch(gray_image, descList, keypointList, 200)
    classID = getClass(matchLen, classIndices, threshold)
    findedClass = 'none'
    if (classID != -1):
        i = (i + 1) % 6
        update_classes()
        findedClass = classNames[classID]
        classFound[classID] += 1
        cv2.putText(currentImage, f'Detected: {classNames[classID]} deb: {matchLen}', (img.shape[1] // 2, img.shape[0] // 2), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)
        # Convert your list of points to a numpy array of type int32
        points = np.array(coordinates[classIndices.index(classID)], dtype=np.int32)

        # Calculate the bounding rectangle only if there are points left after outlier removal
        if points.size > 0:
            x, y, w, h = cv2.boundingRect(points)

            # Draw the rectangle on your image
            cv2.rectangle(currentImage, (x, y), (x + w, y + h), (0, 255, 0), 2)

    else:
        cv2.putText(currentImage, f"Undetected deb: {matchLen}", (img.shape[1] // 2, img.shape[0] // 2), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

    return currentImage, findedClass
        
def recognition(img, depth_image):
    #mask = depth_image > 128
    #img[mask] = [0, 0, 0]
    cur_img, findedClass = detectTrafficSignsOnDataset(img, 10)
    return  cur_img, findedClass, classNames
