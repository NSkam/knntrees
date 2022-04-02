#Class for the points
#The dimensions hold the values of the k dimensions of a point
class Point:
    def __init__(self, dimensions):
        self.dimensions = dimensions

import numpy as np
import bisect
from operator import itemgetter

#The K-d tree node
#splittingAxis is the dimension at which the split happens
#siplittingValue is the spliting value
#dataElement is the element at the node
#leftSub is the left sub tree
#rightSub is the right sub tree
class K_dTreeNode():
    def __init__(self, splittingAxis, splittingValue, dataElement, leftSub, rightSub):
        self.splittingAxis = splittingAxis
        self.splittingValue = splittingValue
        self.dataElement = dataElement
        self.leftSub = leftSub
        self.rightSub = rightSub
        
    def traverseTree(self, point, r , distPoint, pointsFound):
        #Get Examined node Axis values
        examinedNodeSplittingAxis = self.splittingAxis
        #print(examinedNodeSplittingAxis)
        examinedNodeSplittingValue = self.splittingValue
        #print(examinedNodeSplittingValue)
        
        #Debugging Checks
        #print(self.leftSub)
        #print(self.rightSub)
        #print(not bool(self.leftSub))
        #print(not bool(self.rightSub))
        #print(not bool(self.leftSub) and bool(self.rightSub))
        
        #Traverse the tree until we reach a leaf and not bool(self.rightSub):
        if point.dimensions[examinedNodeSplittingAxis]<=examinedNodeSplittingValue:
            if not bool(self.leftSub):
                #This is a leaf node
                print("Reached a leaf node")
                
                #Calculate current nodes distance from point and if it is better then update r
                pointDim = np.array(point.dimensions)
                examinedPointDim = np.array(self.dataElement)
                euclDist = np.linalg.norm(pointDim-examinedPointDim)
                
                bisect.insort(distPoint, euclDist)
                pointIdx = distPoint.index(euclDist)
                pointsFound.insert(pointIdx, self)
                        
                #Check if distance is less than radius r
                if euclDist<=r:
                    r = euclDist
                    #print("Point Found")
                    
                return pointsFound, distPoint, r
            
            else:
                pointsFound, distPoint, r = self.leftSub.traverseTree(point, r,  distPoint, pointsFound)
                print("Traversing the left subtree")
                
                #Calculate current nodes distance from point and if it is better then update r
                pointDim = np.array(point.dimensions)
                examinedPointDim = np.array(self.dataElement)
                euclDist = np.linalg.norm(pointDim-examinedPointDim)
                
                bisect.insort(distPoint, euclDist)
                pointIdx = distPoint.index(euclDist)
                pointsFound.insert(pointIdx, self)
                        
                #Check if distance is less than radius r
                if euclDist<=r:
                    r = euclDist
                    #print("Point Found")
                 
                
                #If there is a right side
                if bool(self.rightSub):
                    
                    #Calculate the distance between the current node and the right side node
                    nodeDist = np.linalg.norm(self.splittingValue-self.rightSub.splittingValue)
                
                    #If current best is bigger than the distance then traverse the right side too
                    if r>nodeDist:
                        self.rightSub.traverseTree(point, r, distPoint, pointsFound)
                
                return pointsFound, distPoint, r    
                
        else:
            if not bool(self.rightSub):
                #This is a leaf node
                print("Reached a leaf node")
                
                #Calculate current nodes distance from point and if it is better then update r
                pointDim = np.array(point.dimensions)
                examinedPointDim = np.array(self.dataElement)
                euclDist = np.linalg.norm(pointDim-examinedPointDim)
                
                bisect.insort(distPoint, euclDist)
                pointIdx = distPoint.index(euclDist)
                pointsFound.insert(pointIdx, self)
                        
                #Check if distance is less than radius r
                if euclDist<=r:
                    r = euclDist
                    #print("Point Found")
                    
                return pointsFound, distPoint, r
            
            else:
                pointsFound, distPoint, r = self.rightSub.traverseTree(point, r, distPoint, pointsFound)
                print("Traversing the rightSub subtree")
                
                #Calculate current nodes distance from point and if it is better then update r
                pointDim = np.array(point.dimensions)
                examinedPointDim = np.array(self.dataElement)
                euclDist = np.linalg.norm(pointDim-examinedPointDim)
                
                bisect.insort(distPoint, euclDist)
                pointIdx = distPoint.index(euclDist)
                pointsFound.insert(pointIdx, self)
                        
                #Check if distance is less than radius r
                if euclDist<=r:
                    r = euclDist
                    #print("Point Found")
                
                #If there is a left side
                if bool(self.leftSub):
               
                    #Calculate the distance between the current node and left right side node
                    nodeDist = np.linalg.norm(self.splittingValue-self.leftSub.splittingValue)
                
                    #If current best is bigger than the distance then traverse the right side too
                    if r>nodeDist:
                        self.leftSub.traverseTree(point, r, distPoint, pointsFound)
                
                return pointsFound, distPoint, r    
        

#The K_d tree class
#startingPoints are the points to be added to the tree
#startingX/Y/Z are the x, y, z, starting dimensions
#maxX/Y/Z are the maximum x, y, z dimensions on the dataset
class K_dTree():
    def __init__(self,startingPoints, startingX, startingY, startingZ, maxX, maxY, maxZ):
        self.startingX = startingX
        self.startingY = startingY
        self.startingZ = startingZ
        self.maxX = maxX
        self.maxY = maxY
        self.maxZ = maxZ
        
        #Extract the dimensions of the Points
        pointDimensions = []
        
        for point in startingPoints:
            pointDimensions.append(point.dimensions)
        
        self.root_node = splitK_dTree(0,pointDimensions)
        
    #kNearestNeighbors Query
    def kNearestNeighbors(self, point):
    
        #Set default distance
        r = max(self.maxX, self.maxY, self.maxZ)
        
        #The distance Matrix
        distPoint = []
                
        #The Point Matrix
        pointsFound = []
                
        
        pointsFound, distPoint, r = self.root_node.traverseTree(point, r, distPoint, pointsFound)
        
        return pointsFound, distPoint
        
        
def splitK_dTree(splittingAxis, startingPoints):
        
        #Check if there are no more points
        if len(startingPoints) == 0:
            return
            
        #Divide by finding median
        #Sort the points according to splittingAxis
        startingPoints.sort(key=itemgetter(splittingAxis))
        
        #Find the median towards the end of the list
        median = int(len(startingPoints)/2)
        medianData = startingPoints[median]
        temp =  median + 1
        for temp in range(len(startingPoints)-1):
            if startingPoints[temp][splittingAxis] == medianData[splittingAxis]:
                median += 1
                temp =  median + 1
        #print(median)
        #print(len(startingPoints))
        if median >= len(startingPoints):
            median = int(median/2)
        medianData = startingPoints[median]
        
        #Update the splitting axis
        newSplittingAxis = (splittingAxis+1) % 3
        
        #Find left and right points
        leftSubPoints = startingPoints[0:median]
        rightSubPoints = startingPoints[median+1:-1]
        
        #Make a new k_d tree node
        node = K_dTreeNode(splittingAxis, medianData[splittingAxis], medianData, splitK_dTree(newSplittingAxis,leftSubPoints), splitK_dTree(newSplittingAxis,rightSubPoints))
        
        return node
    

#import random
#dimensions =[]
#for x in range(25):
#    dimensions.append(Point([random.uniform(0, 100), random.uniform(0, 100), random.uniform(0, 100)]))    

#for x in range(len(dimensions)):
#       print(str(dimensions[x].dimensions))

from DocumentReprsentationAsPoints import *

#Get the dataset points from the collection
documentVec = DocumentReprsentationAsPoints()
print("Finsihed Analyzing the Collection")
#print(documentVec)

#Find the boundaries
minValue = np.min(documentVec)
print(minValue)
for i in range(len(documentVec)):
    documentVec[i] = documentVec[i]+ abs(minValue)
maxValue = np.max(documentVec)
print(maxValue)

#Transform the documentVec into Point objects
pointMatrix = []

for i in range(len(documentVec)):
    pointMatrix.append(Point(documentVec[i]))
    
#print(pointMatrix)
#print(len(pointMatrix))
#for x in range(len(pointMatrix)):
    #print(str(pointMatrix[x].dimensions))

#Construct the Tree
print("Constructing the tree")
tree = K_dTree(pointMatrix[0:1209], 0, 0, 0, maxValue, maxValue, maxValue)
print("Finished Constructing and dividing the tree")

point_in_map = pointMatrix[1299]

points, distances = tree.kNearestNeighbors(point_in_map)

print(points[-1])
print(distances)

euclDist =[]
for i in range(1209):
    
    pointDim = np.array(pointMatrix[1299].dimensions)
    euclDist.append(np.linalg.norm(pointDim-documentVec[i]))
        
euclDist = sorted(euclDist, reverse=False)
print(euclDist)