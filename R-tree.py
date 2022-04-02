#Class for the points
#The dimensions hold the values of the k dimensions of a point
class Point:
    def __init__(self, dimensions):
        self.dimensions = dimensions
        
#Class for the minimum bounding cube        
#x/y/z low is the lowest x/y/z coordinate
#x/y/z high is the highest x/y/z coordinate
class MBR():
    def __init__(self,x_low, x_high, y_low, y_high, z_low, z_high):
        self.x_low = x_low
        self.x_high = x_high
        self.y_low = y_low
        self.y_high = y_high
        self.z_low = z_low
        self.z_high = z_high
        
#class for the nodes of the r-tree
#nodeMBR is the MBR that describes the node
#objectPtr points to the data in the node --> either another node or points
#isLeaf determes the type of data
#parentNode points to the parent node
class rTreeNode():
    def __init__(self,x_low, x_high, y_low, y_high, z_low, z_high, data, leafFlag, parentNode):
        self.nodeMBR = MBR(x_low, x_high, y_low, y_high, z_low, z_high)
        self.objectPtr = data
        self.isLeaf = leafFlag
        self.parentNode = parentNode

from scipy.spatial import ConvexHull
import numpy
from operator import itemgetter
import bisect

#class for the r-tree
class rTree():
    def __init__(self, m, M, data):
        self.mEntries = m
        self.MEntries = M
        self.rootNode = rTreeConstruction(m, M, data)
        
    #k Nearest Neighbors Query
    def kNearestNeighbors(self, point, startingR):
        
        #Set default distance
        r = startingR
        
        #Put root node on a stack
        stack =[]
        stack.append(self.rootNode)
        
        #Points found
        pointsFound =[]
        
        #Distance of points
        distPoint =[]
        
        #Check nodes in respect to r
        while stack:
            nodeToCheck = stack.pop()
            #print(stack)
            for child in nodeToCheck.objectPtr:
                #print(len(child.children))
                #print(child.getPoints())
                if child.isLeaf:
                    for examinedPoint in child.objectPtr: 
                        #print("Checking point: " + str(examinedPoint))
                        #Get distance
                        pointDim = numpy.array(point.dimensions)
                        examinedPointDim = examinedPoint
                        euclDist = numpy.linalg.norm(pointDim-examinedPointDim)
                        #print(euclDist)
                        #print(r)
                        bisect.insort(distPoint, euclDist)
                        pointIdx = distPoint.index(euclDist)
                        pointsFound.insert(pointIdx, examinedPoint)
                        
                        #Check if distance is less than radius r
                        if euclDist<=r:
                            r = euclDist
                            #print("Point Found")
                elif intersection(point, r, child):
                    stack.append(child)
                    #print("Pushed to Stack")
        #Return closest point
        return pointsFound,distPoint
        

#Constructs the r-tree
#data is our points
def rTreeConstruction(m, M, data):
    
    #Extract the dimensions of the Points
    pointDimensions = []
        
    for point in data:
        pointDimensions.append(point.dimensions)
        
    pointDimensions = numpy.array(pointDimensions)
    
    #Find the maximum and minimum x/y/z
    x_low, x_high, y_low, y_high, z_low, z_high = findBoundaries(pointDimensions)
    
    rootNode = rTreeNode(x_low, x_high, y_low, y_high, z_low, z_high, splitRTree(None,m, M, pointDimensions), False, None)
    
    return rootNode

#Splits the r-tree
def splitRTree(self,m, M, pointDimensions):
    
    #Partition the point matrix into 2 groups
    #First find the two seed points using the convex hull of the data
    #Extract the points forming the hull
    try:
        convexHull = ConvexHull(pointDimensions)
        convexHullPoints = pointDimensions[convexHull.vertices,:]
    except Exception:
        convexHullPoints = pointDimensions
    #print(convexHull.vertices)
    #print(pointDimensions)
    #print(pointDimensions[convexHull.vertices,:])
    
    #Find the distance naively using the convex hull points
    maxDist = 0
    pointA = pointDimensions[0]
    pointB = pointDimensions[-1]
    for x in range(len(convexHullPoints)):
        for y in range(len(convexHullPoints)):
            euclDist = numpy.linalg.norm(convexHullPoints[x]-convexHullPoints[y])
            if euclDist>maxDist:
                pointA = convexHullPoints[x]
                pointB = convexHullPoints[y]
                maxDist = euclDist
    
    #assign each point in one group
    A_group_points = []
    B_group_points = []
    for i in range(len(pointDimensions)):
        euclDistA = numpy.linalg.norm(pointA-pointDimensions[i])
        euclDistB = numpy.linalg.norm(pointB-pointDimensions[i])
        if euclDistA>euclDistB:
            B_group_points.append(pointDimensions[i])
        else:
            A_group_points.append(pointDimensions[i])

    
    #Check if both groups have m elements and if they dont balance them
    if len(A_group_points)<m:
        dist = []
        for i in range(len(B_group_points)):
            dist.append([i,numpy.linalg.norm(pointA-B_group_points[i])])
        diff = m - len(A_group_points)
        dist.sort(key=itemgetter(0))
        for y in range(diff):
            A_group_points.append(B_group_points[dist[y][0]])
            del B_group_points[dist[y][0]]
    
    if len(B_group_points)<m:
        dist = []
        for i in range(len(A_group_points)):
            dist.append([i,numpy.linalg.norm(pointB-A_group_points[i])])
        diff = m - len(B_group_points)
        dist.sort(key=itemgetter(0))
        #print(dist[1][1])
        for y in range(diff):
            B_group_points.append(A_group_points[dist[y][0]])
            del A_group_points[dist[y][0]]
        
    #print(len(A_group_points))
    #print(len(B_group_points))
    A_group_points = numpy.array(A_group_points)
    B_group_points = numpy.array(B_group_points)
    #Find the boudaries for each group   
    boundA_x_low, boundA_x_high, boundA_y_low, boundA_y_high, boundA_z_low, boundA_z_high = findBoundaries(A_group_points)
    boundB_x_low, boundB_x_high, boundB_y_low, boundB_y_high, boundB_z_low, boundB_z_high = findBoundaries(B_group_points)
        
    #Split the tree recursively
    if len(A_group_points)>M:
        print("Still Spliting")
        nodeA = rTreeNode(boundA_x_low, boundA_x_high, boundA_y_low, boundA_y_high, boundA_z_low, boundA_z_high, splitRTree(self,m, M, A_group_points), False, self)
    else:
        print("Created a leaf")
        nodeA = rTreeNode(boundA_x_low, boundA_x_high, boundA_y_low, boundA_y_high, boundA_z_low, boundA_z_high, A_group_points, True, self)
    if len(B_group_points)>M:
        print("Still Spliting")
        nodeB = rTreeNode(boundB_x_low, boundB_x_high, boundB_y_low, boundB_y_high, boundB_z_low, boundB_z_high, splitRTree(self,m, M, B_group_points), False, self)
    else:
        print("Created a leaf")
        nodeB = rTreeNode(boundB_x_low, boundB_x_high, boundB_y_low, boundB_y_high, boundB_z_low, boundB_z_high, B_group_points, True, self)
    
    return [nodeA, nodeB]

#Find Boundaries of points
def findBoundaries(pointDimensions):
    #print(pointDimensions)
    
    #Find the maximum and minimum x/y/z
    x_low = pointDimensions[0][0]
    x_high = pointDimensions[0][0]
    y_low = pointDimensions[0][1]
    y_high = pointDimensions[0][1]
    z_low = pointDimensions[0][2]
    z_high = pointDimensions[0][2]
    
    for x in range(1,len(pointDimensions)):
        
        #x dimension
        if pointDimensions[x][0] < x_low:
            x_low = pointDimensions[x][0]
        elif pointDimensions[x][0]> x_high:
            x_high = pointDimensions[x][0]
            
        #y dimension
        if pointDimensions[x][1] < y_low:
            y_low = pointDimensions[x][1]
        elif pointDimensions[x][1]> y_high:
            y_high = pointDimensions[x][1]
        
        #z dimesion
        if pointDimensions[x][2] < z_low:
            z_low = pointDimensions[x][2]
        elif pointDimensions[x][2]> z_high:
            z_high = pointDimensions[x][2]
    return x_low, x_high, y_low, y_high, z_low, z_high

#Intersection between a ball and a cube
def intersection(center, radius, cube):
    dmin = 0
    dmax = 0
    face = False
    Bmin = [cube.nodeMBR.x_low, cube.nodeMBR.y_low, cube.nodeMBR.z_low]
    Bmax = [cube.nodeMBR.x_high, cube.nodeMBR.y_high, cube.nodeMBR.z_high]
    for i in range(3):
        if center.dimensions[i] < Bmin[i]:
            dmin += (center.dimensions[i] - Bmin[i])**2
        elif center.dimensions[i] > Bmax[i]:
            dmin += (center.dimensions[i] - Bmax[i])**2
    if dmin <= radius**2: 
        return True
    
    return False   


from DocumentReprsentationAsPoints import *

#Get the dataset points from the collection
documentVec = DocumentReprsentationAsPoints()
print("Finsihed Analyzing the Collection")
#print(documentVec)

#Find the boundaries
minValue = numpy.min(documentVec)
print(minValue)
for i in range(len(documentVec)):
    documentVec[i] = documentVec[i]+ abs(minValue)
maxValue = numpy.max(documentVec)
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
tree = rTree(25,50, pointMatrix)
print("Finished Constructing and spliting the tree")

point_in_map = pointMatrix[1299]
print("Checking Point:" + str(point_in_map.dimensions))

point_found,r = tree.kNearestNeighbors(point_in_map,100)

print(point_found[1])
print(r)

euclDist =[]
for i in range(1209):
    
    pointDim = numpy.array(pointMatrix[1299].dimensions)
    euclDist.append(numpy.linalg.norm(pointDim-documentVec[i]))
        
euclDist = sorted(euclDist, reverse=False)
print("Testing List:")
print(euclDist)
