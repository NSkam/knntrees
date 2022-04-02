#Class for the points
#The dimensions hold the values of the k dimensions of a point
class Point:
    def __init__(self, dimensions):
        self.dimensions = dimensions
        
import numpy as np
import bisect

#The quad tree node
#x_dim is the starting point of the square in x axis
#y_dim is the starting point of the square in y axis
#z_dim is the starting point of the square in z axis
#length is the length of the square along the z axis
#width is the length of the square along the x axis
#height is the length of the square along the y axis
#points are the points it contains
#children are the children nodes it contains (8 in total)
class QuadTreeNode():
    def __init__(self, x_dim, y_dim, z_dim, width, height, length, points):
        self.xdim = x_dim
        self.ydim = y_dim
        self.zdim = z_dim
        self.length = length
        self.width = width
        self.height = height
        self.points = points
        self.children = []
    
    #Get the length of the tree
    def getLength(self):
        return self.length

    #Get the windth of the tree
    def getWidth(self):
        return self.width
    
    #Get the height of the tree 
    def getHeight(self):
        return self.height
    
    #Get the points contained in the node
    def getPoints(self):
        return self.points

#Class for the quad tree
#maximumPoints are the number of points allowed in a node
#startingPoints are the default points at the start
#startingX/Y/Z are the x, y, z, starting dimensions
#maxX/Y/Z are the maximum x, y, z dimensions on the dataset
class QuadTree():
    def __init__(self, maximumPoints, startingPoints, startingX, startingY, startingZ, maxX, maxY, maxZ):
        self.maximum_points = maximumPoints
        self.points = startingPoints
        self.root = QuadTreeNode(startingX, startingY, startingZ,  maxX, maxY, maxZ, self.points)
    
    #Get the points of the tree
    def getPoints(self):
        return self.points
    
    #Divide the tree in 8 different nodes
    def divideTree(self):
        treeDivision(self.root, self.maximum_points)
        
    #k Nearest Neighbors Query
    def kNearestNeighbors(self, point):
        
        #Set default distance
        r = max(self.root.width, self.root.height, self.root.length)
        
        #Put root node on a stack
        stack =[]
        stack.append(self.root)
        
        #Points found
        pointsFound =[]
        
        #Distance of points
        distPoint =[]
        
        #Check nodes in respect to r
        while stack:
            nodeToCheck = stack.pop()
            #print(stack)
            for child in nodeToCheck.children:
                #print(len(child.children))
                #print(child.getPoints())
                if intersection(point, r, child):
                    stack.append(child)
                    #print("Pushed to Stack")
                if len(child.children) == 0:
                    for examinedPoint in child.points: 
                        #print("Checking point: " + str(examinedPoint))
                        #Get distance
                        pointDim = np.array(point.dimensions)
                        examinedPointDim = np.array(examinedPoint.dimensions)
                        euclDist = np.linalg.norm(pointDim-examinedPointDim)
                        #print(euclDist)
                        #print(r)
                        bisect.insort(distPoint, euclDist)
                        pointIdx = distPoint.index(euclDist)
                        pointsFound.insert(pointIdx, examinedPoint)
                        
                        #Check if distance is less than radius r
                        if euclDist<=r:
                            r = euclDist
                            #print("Point Found")
        
        #Return closest point
        return pointsFound,distPoint
                        
    
    
#Recursively diviedes the nodes in 8 different pieces -->  Back north west (bnw), front north west (fnw), back north east (bne), front north east (fne), Back south west (bsw), front south west (fsw), back south east (bse), front south east (fse)
def treeDivision(nodeToBeDivided, maximumPointsAllowed):
    
    #If node contains less than allowed points dont divide it
    if len(nodeToBeDivided.getPoints())<=maximumPointsAllowed:
        #print(len(nodeToBeDivided.getPoints()))
        return
    
    #The new length, width and height
    l = float(nodeToBeDivided.getLength()/2)
    
    w = float(nodeToBeDivided.getWidth()/2)
    
    h = float(nodeToBeDivided.getHeight()/2)
   
    #Find the points that belong to fsw
    ptsfsw = []
    for point in nodeToBeDivided.getPoints():
        if point.dimensions[0] >= nodeToBeDivided.xdim and point.dimensions[0] <= (nodeToBeDivided.xdim + w) and point.dimensions[1]>=nodeToBeDivided.ydim and point.dimensions[1]<=(nodeToBeDivided.ydim+h) and point.dimensions[2]>=nodeToBeDivided.zdim and point.dimensions[2]<=(nodeToBeDivided.zdim+l):
            ptsfsw.append(point)
    
    #Create a new node
    fsw = QuadTreeNode(nodeToBeDivided.xdim, nodeToBeDivided.ydim, nodeToBeDivided.zdim, l, w, h, ptsfsw)
    
    #Divide the node again
    treeDivision(fsw,maximumPointsAllowed)
    
    #Find the points that belong to bsw
    ptsbsw = []
    for point in nodeToBeDivided.getPoints():
        if point.dimensions[0] >= nodeToBeDivided.xdim and point.dimensions[0] <= (nodeToBeDivided.xdim + w) and point.dimensions[1]>=nodeToBeDivided.ydim and point.dimensions[1]<=(nodeToBeDivided.ydim+h) and point.dimensions[2]>=nodeToBeDivided.zdim+l and point.dimensions[2]<=(nodeToBeDivided.zdim+2*l):
            ptsbsw.append(point)
    
    #Create a new node
    bsw = QuadTreeNode(nodeToBeDivided.xdim, nodeToBeDivided.ydim, nodeToBeDivided.zdim+l, l, w, h, ptsbsw)
    
    #Divide the node again
    treeDivision(bsw,maximumPointsAllowed)
    
    #Find the points that belong to fnw
    ptsfnw = []
    for point in nodeToBeDivided.getPoints():
        if point.dimensions[0] >= nodeToBeDivided.xdim and point.dimensions[0] <= (nodeToBeDivided.xdim + w) and point.dimensions[1]>=nodeToBeDivided.ydim+h and point.dimensions[1]<=(nodeToBeDivided.ydim+2*h) and point.dimensions[2]>=nodeToBeDivided.zdim and point.dimensions[2]<=(nodeToBeDivided.zdim+l):
            ptsfnw.append(point)
    
    #Create a new node
    fnw = QuadTreeNode(nodeToBeDivided.xdim, nodeToBeDivided.ydim+h, nodeToBeDivided.zdim, l, w, h, ptsfnw)
    
    #Divide the node again
    treeDivision(fnw,maximumPointsAllowed)
    
    #Find the points that belong to bnw
    ptsbnw = []
    for point in nodeToBeDivided.getPoints():
        if point.dimensions[0] >= nodeToBeDivided.xdim and point.dimensions[0] <= (nodeToBeDivided.xdim + w) and point.dimensions[1]>=nodeToBeDivided.ydim+h and point.dimensions[1]<=(nodeToBeDivided.ydim+2*h) and point.dimensions[2]>=nodeToBeDivided.zdim+l and point.dimensions[2]<=(nodeToBeDivided.zdim+2*l):
            ptsbnw.append(point)
    
    #Create a new node
    bnw = QuadTreeNode(nodeToBeDivided.xdim, nodeToBeDivided.ydim+h, nodeToBeDivided.zdim+l, l, w, h, ptsbnw)
    
    #Divide the node again
    treeDivision(bnw,maximumPointsAllowed)
    
    #Find the points that belong to fse
    ptsfse = []
    for point in nodeToBeDivided.getPoints():
        if point.dimensions[0] >= nodeToBeDivided.xdim+w and point.dimensions[0] <= (nodeToBeDivided.xdim+2*w) and point.dimensions[1]>=nodeToBeDivided.ydim and point.dimensions[1]<=(nodeToBeDivided.ydim+h) and point.dimensions[2]>=nodeToBeDivided.zdim and point.dimensions[2]<=(nodeToBeDivided.zdim+l):
            ptsfse.append(point)
    
    #Create a new node
    fse = QuadTreeNode(nodeToBeDivided.xdim+w, nodeToBeDivided.ydim, nodeToBeDivided.zdim, l, w, h, ptsfse)
    
    #Divide the node again
    treeDivision(fse,maximumPointsAllowed)
    
    #Find the points that belong to bse
    ptsbse = []
    for point in nodeToBeDivided.getPoints():
        if point.dimensions[0] >= nodeToBeDivided.xdim+w and point.dimensions[0] <= (nodeToBeDivided.xdim+2*w) and point.dimensions[1]>=nodeToBeDivided.ydim and point.dimensions[1]<=(nodeToBeDivided.ydim+h) and point.dimensions[2]>=nodeToBeDivided.zdim+l and point.dimensions[2]<=(nodeToBeDivided.zdim+2*l):
            ptsbse.append(point)
    
    #Create a new node
    bse = QuadTreeNode(nodeToBeDivided.xdim+w, nodeToBeDivided.ydim, nodeToBeDivided.zdim+l, l, w, h, ptsbse)
    
    #Divide the node again
    treeDivision(bse,maximumPointsAllowed)
    
    #Find the points that belong to fne
    ptsfne = []
    for point in nodeToBeDivided.getPoints():
        if point.dimensions[0] >= nodeToBeDivided.xdim+w and point.dimensions[0] <= (nodeToBeDivided.xdim+2*w) and point.dimensions[1]>=nodeToBeDivided.ydim+h and point.dimensions[1]<=(nodeToBeDivided.ydim+2*h) and point.dimensions[2]>=nodeToBeDivided.zdim and point.dimensions[2]<=(nodeToBeDivided.zdim+l):
            ptsfne.append(point)
    
    #Create a new node
    fne = QuadTreeNode(nodeToBeDivided.xdim+w, nodeToBeDivided.ydim+h, nodeToBeDivided.zdim, l, w, h, ptsfne)
    
    #Divide the node again
    treeDivision(fne,maximumPointsAllowed)
    
    #Find the points that belong to bne
    ptsbne = []
    for point in nodeToBeDivided.getPoints():
        if point.dimensions[0] >= nodeToBeDivided.xdim+w and point.dimensions[0] <= (nodeToBeDivided.xdim+2*w) and point.dimensions[1]>=nodeToBeDivided.ydim+h and point.dimensions[1]<=(nodeToBeDivided.ydim+2*h) and point.dimensions[2]>=nodeToBeDivided.zdim+l and point.dimensions[2]<=(nodeToBeDivided.zdim+2*l):
            ptsbne.append(point)
    
    #Create a new node
    bne = QuadTreeNode(nodeToBeDivided.xdim+w, nodeToBeDivided.ydim+h, nodeToBeDivided.zdim+l, l, w, h, ptsbne)
    
    #Divide the node again
    treeDivision(bne,maximumPointsAllowed)
    
    nodeToBeDivided.children = [fsw, bsw, fnw, bnw, fse, bse, fne, bne]
    
#Intersection between a ball and a cube
def intersection(center, radius, cube):
    dmin = 0
    dmax = 0
    face = False
    Bmin = [cube.xdim, cube.ydim, cube.zdim]
    Bmax = [cube.xdim+cube.width, cube.ydim+cube.height, cube.zdim+cube.length]
    for i in range(3):
        if center.dimensions[i] < Bmin[i]:
            dmin += (center.dimensions[i] - Bmin[i])**2
        elif center.dimensions[i] > Bmax[i]:
            dmin += (center.dimensions[i] - Bmax[i])**2
    if dmin <= radius**2: 
        return True
    
    return False   


#Test Dataset

#import random
#dimensions =[]
#for x in range(50):
#    dimensions.append(Point([random.uniform(0, 2), random.uniform(0, 2), random.uniform(0, 2)]))
#for x in range(len(dimensions)):
#   print(str(dimensions[x].dimensions))
    
#for i in range(50):
    
#    pointDim = np.array([4.830773532275872, 2.7522813990472184, 3.2440926730548925])
#    examinedPointDim =  np.array(dimensions[i].dimensions)
#    euclDist = np.linalg.norm(pointDim-examinedPointDim)
        
#    print(euclDist)


from DocumentReprsentationAsPoints import *

#Get the dataset points from the collection
documentVec = DocumentReprsentationAsPoints()
print("Finsihed Analyzing the Collection")
#print(len(documentVec))

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
tree_points = pointMatrix[0:1209]
print("Constructing the tree")
tree = QuadTree(70, tree_points[0:1209], 0, 0, 0, maxValue, maxValue, maxValue)
print("Dividing the tree")
tree.divideTree()
print("Finished Constructing and dividing the tree")

point_in_map = pointMatrix[1299]

point_found,r = tree.kNearestNeighbors(point_in_map)

print(point_found[0].dimensions)
print(r)

euclDist =[]
for i in range(1209):
    
    pointDim = np.array(pointMatrix[1299].dimensions)
    euclDist.append(np.linalg.norm(pointDim-documentVec[i]))
        
euclDist = sorted(euclDist, reverse=False)
print(euclDist)