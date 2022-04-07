import math
import matplotlib.pyplot as plt
import json
import random


#random.seed(3)

# REMAINING TASKS
#
# * Account for altitude, make radius of camera change as altitude changes
# * Account for location of UGV
#
# NOT SO IMPORTANT, DO AT END IF TIME/EFFORT PERMITS
#
# find a way to save efficiency in the middle of the boundary points
#
# LATER ON MORE COMPLEX TASKS
# * Try to think of a way to not search around an obstacle twice
# * Try to think of a way to prioritize certain regions that have a higher chance of containing targets mid-flight (this will likely be another program)




# FUNCTIONS FROM LEONARD
def calc_bearing(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    dlat = lat2 - lat1
    dlon = lon2- lon1
    return math.atan2(math.sin(dlon) * math.cos(lat2), math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))

def calc_haversine(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    dlat = lat2 - lat1
    dlon = lon2- lon1
    a = pow(math.sin(dlat / 2),2) + math.cos(lat1) * math.cos(lat2) * pow(math.sin(dlon / 2), 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = 6371e3 * c  * 3.28084 #  feet
    return d

def cartesian_to_decimal(x, y, lat1, lon1):
    bearing = math.atan(y / x)
    d = x / math.cos(bearing)
    r = 6371e3 * 3.28084 # feet
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.degrees(math.asin(math.sin(lat1) * math.cos(d / r) +
                        math.cos(lat1) * math.sin(d / r) * math.cos(bearing)))
    lon2 =  math.degrees(lon1 + math.atan2(math.sin(bearing) * math.sin(d / r) * math.cos(lat1),
                                math.cos(d / r) - math.sin(lat1) * math.sin(math.radians(lat2))))
    return lat2, lon2



def decimal_to_cartesian(lat1, lon1, lat2, lon2):
    d = calc_haversine(lat2, lon2, lat1, lon1)
    bearing = calc_bearing(lat2, lon2, lat1, lon1)
    x = d * math.cos(bearing)
    y = d * math.sin(bearing)
    return x, y

# END OF FUNCTIONS FROM LEONARD TODO use Mapping.py



#Remove obstacles that are not in search grid
def obstacleInGrid(feetSearchGridPoints, feetStationaryObstacles):
    for ob in feetStationaryObstacles:
        hasLeft = 0
        hasRight = 0
        hasUp = 0
        hasDown = 0
        for searchpt in feetSearchGridPoints:
            if searchpt["latitude"] < ob["latitude"]:
                hasDown = 1
            if searchpt["latitude"] > ob["latitude"]:
                hasUp = 1
            if searchpt["longitude"] > ob["longitude"]:
                hasRight = 1
            if searchpt["longitude"] < ob["longitude"]:
                hasLeft = 1
        if (hasLeft == 0 or hasRight == 0 or hasUp == 0 or hasDown == 0):
            feetStationaryObstacles.remove(ob)
            


def calcAngle(x1, y1, x2, y2):
    
    if (x2 - x1) == 0:
        x2 += 0.0001
        
    slope = (y2 - y1) / (x2 - x1)
    
    ang = math.atan(slope)
    
    
    if (x2 < x1):
        ang += math.pi
    elif (math.fabs(x1 - x2) < 0.0000001 and y2 < y1):
        ang += math.pi
    
    
    ang %= 2 * math.pi
    
    return round(ang, 3)
    
def getAngle(x, y, x1, y1, x2, y2):
    angleLine1 = calcAngle(x, y, x1, y1)
    angleLine2 = calcAngle(x, y, x2, y2)
    
    
    
    
    angle1 = ((angleLine1 + angleLine2) / 2) % (2 * math.pi)
    angle2 = (angle1 + math.pi) % (2 * math.pi)
    
    angle = angle1
    
    if (angleLine2 > angle1 and angle1 > angleLine1):
        angle = angle1
    else:
        angle = angle2
    
    return angle
    

class Cell:
    
    
    def __init__(self, pts, number):
        self.searchPts = pts
        self.number = number
        
        self.maxLon = -99999
        self.minLon = 99999
        
        for i in self.searchPts:
            if (i["longitude"] > self.maxLon):
                self.maxLon = i["longitude"]
            if (i["longitude"] < self.minLon):
                self.minLon = i["longitude"]
                
            
        
    def inCell(self, x, y):
        hasLeft = 0
        hasRight = 0
        hasUp = 0
        hasDown = 0
        for searchpt in self.searchPts:
            if searchpt["latitude"] < y:
                hasDown = 1
            if searchpt["latitude"] > y:
                hasUp = 1
            if searchpt["longitude"] > x:
                hasRight = 1
            if searchpt["longitude"] < x:
                hasLeft = 1
        if (hasLeft == 0 or hasRight == 0 or hasUp == 0 or hasDown == 0):
            return False
        else:
            return True
    
    def addPt(self, pt):
        self.searchPts.append(pt)
        
        for i in self.searchPts:
            if (i["longitude"] > self.maxLon):
                self.maxLon = i["longitude"]
            if (i["longitude"] < self.minLon):
                self.minLon = i["longitude"]
        
    def insertPt(self, i, pt):
        self.searchPts.insert(i, pt)
        
        for i in self.searchPts:
            if (i["longitude"] > self.maxLon):
                self.maxLon = i["longitude"]
            if (i["longitude"] < self.minLon):
                self.minLon = i["longitude"]
        
    def addStartPt(self):
        self.searchPts.append(self.searchPts[0])
        
    def removeDuplicates(self):
        newPts = []
        for i in self.searchPts:
            if i not in newPts:
                newPts.append(i)
        
        self.searchPts = newPts
        
    def orderPoints(self):
        
        # Using mean distance in each dimension
        
        midLat = 0
        midLon = 0
        for i in self.searchPts:
            midLat += i["latitude"]
            midLon += i["longitude"]
            
        midLat /= len(self.searchPts)
        midLon /= len(self.searchPts)
        
        # Using center of bounding box
        
        '''
        minLat = 99999
        maxLat = -99999
        minLon = 99999
        maxLon = -99999
        
        for i in self.searchPts:
            if (i["latitude"] < minLat):
                minLat = i["latitude"]
            if (i["latitude"] > maxLat):
                maxLat = i["latitude"]
            if (i["longitude"] < minLon):
                minLon = i["longitude"]
            if (i["longitude"] > maxLon):
                maxLon = i["longitude"]
        
        midLat = (minLat + maxLat) / 2
        midLon = (minLat + maxLat) / 2
        '''

        angles = []
        for i in self.searchPts:
            angles.append({"point" : i, "angle" : calcAngle(midLon, midLat, i["longitude"], i["latitude"])})
        
        angles = sorted(angles, key=lambda x:x["angle"])
        
        orderedPts = []
        
        for i in angles:
            orderedPts.append(i["point"])
            
        self.searchPts = orderedPts
        
    def plot(self):
        
        plt.figure(1)
        
        tempxgrid = []
        tempygrid = []
        
        for i in self.searchPts:
            tempxgrid.append(i["longitude"])
            tempygrid.append(i["latitude"])
        
        
        plt.plot(tempxgrid, tempygrid)
        





    
def intersectionPtDown(ob, feetSearchGridPoints):

    for i in range(len(feetSearchGridPoints) - 1):
        if ((feetSearchGridPoints[i]["longitude"] > ob["longitude"] and feetSearchGridPoints[i + 1]["longitude"] < ob["longitude"])):
            
            m = (feetSearchGridPoints[i + 1]["latitude"] - feetSearchGridPoints[i]["latitude"]) / (feetSearchGridPoints[i + 1]["longitude"] - feetSearchGridPoints[i]["longitude"])
            # y = mx + b
            # b = y - mx
            b = feetSearchGridPoints[i]["latitude"] - m * feetSearchGridPoints[i]["longitude"]
            
            newY = m * ob["longitude"] + b
            
            if (newY < ob["latitude"]):
                returnPt = {"latitude" : m * ob["longitude"] + b, "longitude" : ob["longitude"]}
                
                return returnPt

def intersectionPtUp(ob, feetSearchGridPoints):
    for i in range(len(feetSearchGridPoints) - 1):
        if ((feetSearchGridPoints[i]["longitude"] < ob["longitude"] and feetSearchGridPoints[i + 1]["longitude"] > ob["longitude"])):
            
            m = (feetSearchGridPoints[i + 1]["latitude"] - feetSearchGridPoints[i]["latitude"]) / (feetSearchGridPoints[i + 1]["longitude"] - feetSearchGridPoints[i]["longitude"])
            # y = mx + b
            # b = y - mx
            b = feetSearchGridPoints[i]["latitude"] - m * feetSearchGridPoints[i]["longitude"]
            
            newY = m * ob["longitude"] + b
            
            if (newY > ob["latitude"]):
                returnPt = {"latitude" : m * ob["longitude"] + b, "longitude" : ob["longitude"]}
                
                return returnPt

def getObstaclePts(ob, startAng, cameraWidth):
    numPts = ob["radius"] / 50
    
    ptRad = ob["radius"] + (cameraWidth / 3)
    angle = math.pi * 2 / numPts
    
    pts = []
    
    curAng = startAng
    
    finAng = curAng + math.pi
    
    while (curAng <= finAng):
        pts.append({"latitude" : ob["latitude"] + ptRad * math.sin(curAng), "longitude" : ob["longitude"] + ptRad * math.cos(curAng)})
        curAng += angle
    
    pts.append({"latitude" : ob["latitude"] + ptRad * math.sin(finAng), "longitude" : ob["longitude"] + ptRad * math.cos(finAng)})
    #for i in range(round(numPts / 2) + 1):
    #    pts.append({"latitude" : ob["latitude"] + ptRad * math.sin(curAng), "longitude" : ob["longitude"] + ptRad * math.cos(curAng)})
    #    curAng += angle
    
    
    return pts




    
def inObstacle(x, y, feetStationaryObstacles):
    obInd = -1
    for i in range(len(feetStationaryObstacles)):
        obX = feetStationaryObstacles[i]["longitude"]
        obY = feetStationaryObstacles[i]["latitude"]
        obRad = feetStationaryObstacles[i]["radius"]
        
        if (math.sqrt(math.pow(x - obX, 2) + math.pow(y - obY, 2)) < obRad):
            obInd = i
    
    return obInd
        

# FUNCTION TO CREATE WAYPOINTS
# MODE 1 -- basic go up and down
# MODE 2 -- more efficient, go around boundary
def createPoints(mode, feetSearchGridPoints, numLoops, feetStationaryObstacles, xWayPts, yWayPts, altWayPts, cameraWidth, constAlt):

                
    if (mode == 2):
        angles = []
        pt1y = feetSearchGridPoints[-2]["latitude"] # FIXME x should be lat, y should be lon!!!!!!!
        pt2y = feetSearchGridPoints[1]["latitude"]
        pt1x = feetSearchGridPoints[-2]["longitude"]
        pt2x = feetSearchGridPoints[1]["longitude"]
        pty = feetSearchGridPoints[0]["latitude"]
        ptx = feetSearchGridPoints[0]["longitude"]
        
        
        angles.append(getAngle(ptx, pty, pt1x, pt1y, pt2x, pt2y) + math.pi)
        
        curXPts = []
        curYPts = []
        
        for i in range(1, len(feetSearchGridPoints) - 1):
            pt1y = feetSearchGridPoints[i-1]["latitude"]
            pt2y = feetSearchGridPoints[i+1]["latitude"]
            pt1x = feetSearchGridPoints[i-1]["longitude"]
            pt2x = feetSearchGridPoints[i+1]["longitude"]
            pty = feetSearchGridPoints[i]["latitude"]
            ptx = feetSearchGridPoints[i]["longitude"]
            
            angles.append(getAngle(ptx, pty, pt1x, pt1y, pt2x, pt2y) + math.pi)
        
        pt1y = feetSearchGridPoints[-2]["latitude"]
        pt2y = feetSearchGridPoints[1]["latitude"]
        pt1x = feetSearchGridPoints[-2]["longitude"]
        pt2x = feetSearchGridPoints[1]["longitude"]
        pty = feetSearchGridPoints[0]["latitude"]
        ptx = feetSearchGridPoints[0]["longitude"]
        
        angles.append(getAngle(ptx, pty, pt1x, pt1y, pt2x, pt2y) + math.pi)
        
        for i in range(len(angles)):
            curYPts.append(feetSearchGridPoints[i]["latitude"] - 5/4 * cameraWidth * math.sin(angles[i]))
            curXPts.append(feetSearchGridPoints[i]["longitude"] - 5/4 * cameraWidth * math.cos(angles[i]))
        
        minDistToCenter = 99999
        maxXPt = -99999;
        minXPt = 99999;
        maxYPt = -99999;
        minYPt = 99999;
        for i in range(len(feetSearchGridPoints)):
            if (feetSearchGridPoints[i]["latitude"] > maxYPt):
                maxYPt = feetSearchGridPoints[i]["latitude"]
            if (feetSearchGridPoints[i]["longitude"] > maxXPt):
                maxXPt = feetSearchGridPoints[i]["longitude"]
            if (feetSearchGridPoints[i]["latitude"] < minYPt):
                minYPt = feetSearchGridPoints[i]["latitude"];
            if (feetSearchGridPoints[i]["longitude"] < minXPt):
                minXPt = feetSearchGridPoints[i]["longitude"]
        
        cellWidth = maxXPt - minXPt
        cellHeight = maxYPt - minYPt
        
        if (cellWidth < cellHeight):
            minLoops = round(cellWidth / cameraWidth / 2 - cellHeight / cellWidth * 1.5)
        else:
            minLoops = round(cellHeight / cameraWidth / 2 - cellWidth / cellHeight * 1.5)
        cellCenterX = (maxXPt - minXPt) / 2 + minXPt
        cellCenterY = (maxYPt - minYPt) / 2 + minYPt
        
        curLoop = 0
        while (minDistToCenter > cameraWidth * 2 or minLoops > curLoop):
            curLoop += 1
            for i in range(len(angles)):
                if (inObstacle(curXPts[i] + cameraWidth * math.cos(angles[i]), curYPts[i] + cameraWidth * math.sin(angles[i]), feetStationaryObstacles) == -1):
                    
                    xWayPts.append(curXPts[i] + cameraWidth * 2 * math.cos(angles[i]))
                    yWayPts.append(curYPts[i] + cameraWidth * 2 * math.sin(angles[i]))
                    
                    distToCenter = math.sqrt(math.pow(curXPts[i] + cameraWidth * 2 * math.cos(angles[i]) - cellCenterX, 2) + math.pow(curYPts[i] + cameraWidth * 2 * math.sin(angles[i]) - cellCenterY, 2))
                    
                    if (distToCenter < minDistToCenter):
                        minDistToCenter = distToCenter
                    
                else:
                    angleToPrev = calcAngle(curXPts[i] + cameraWidth * math.cos(angles[i]), curYPts[i] + cameraWidth * math.sin(angles[i]), curXPts[i-1], curYPts[i-1])
                    
                    adder = cameraWidth
                    testPtX = curXPts[i] + cameraWidth * math.cos(angles[i]) + adder * math.cos(angleToPrev)
                    testPtY = curYPts[i] + cameraWidth * math.sin(angles[i]) + adder * math.sin(angleToPrev)
                    
                    while (inObstacle(testPtX, testPtY, feetStationaryObstacles) != -1):
                        testPtX += adder * math.cos(angleToPrev)
                        testPtY += adder * math.sin(angleToPrev)
                    
                    #testPtX += adder * math.cos(angleToPrev)
                    #testPtY += adder * math.sin(angleToPrev)
                    
                    print("Point is in obstacle")
                    xWayPts.append(testPtX)
                    yWayPts.append(testPtY)
                    
                    
                    
                    
                    
                    angleToNext = calcAngle(curXPts[i] + cameraWidth * math.cos(angles[i]), curYPts[i] + cameraWidth * math.sin(angles[i]), curXPts[(i + 1) % len(curXPts)] + cameraWidth * math.cos(angles[(i+1)%len(angles)]), curYPts[(i+1)%len(curYPts)] + cameraWidth * math.sin(angles[(i+1)%len(angles)]))
                    
                    testPtX = curXPts[i] + cameraWidth * math.cos(angles[i]) + adder * math.cos(angleToNext)
                    testPtY = curYPts[i] + cameraWidth * math.sin(angles[i]) + adder * math.sin(angleToNext)
                    
                    while (inObstacle(testPtX, testPtY, feetStationaryObstacles) != -1):
                        testPtX += adder * math.cos(angleToNext)
                        testPtY += adder * math.sin(angleToNext)
                    
                    #testPtX += adder * math.cos(angleToNext)
                    #testPtY += adder * math.sin(angleToNext)
                    
                    xWayPts.append(testPtX)
                    yWayPts.append(testPtY)
                    altWayPts.append(constAlt)
                    
                    # NEXT SESSION: WORK ON CODE TO MAKE NEW WAY POINTS BE AROUND OBSTACLES
                    
                curXPts[i] = curXPts[i] + cameraWidth * math.cos(angles[i])
                curYPts[i] = curYPts[i] + cameraWidth * math.sin(angles[i])
                altWayPts.append(constAlt)


def getSearchAndObPts(inputFile):
    interop_example_file = open(inputFile, "r")
    interop_example_obj = json.load(interop_example_file)
    interop_example_file.close()
    
    # import boundary points
    searchGridPoints = interop_example_obj["searchGridPoints"]
    stationaryObstacles = interop_example_obj["stationaryObstacles"]
        
    return searchGridPoints, stationaryObstacles

def getMinMaxGrid(searchGridPoints):
    minLat = 10000
    maxLat = -10000
    minLon = 10000
    maxLon = -10000
    minDist = 999999
    minDistInd = 0
    
    for pt in searchGridPoints:
        if (pt["latitude"] < minLat):
            minLat = pt["latitude"]
        if (pt["latitude"] > maxLat):
            maxLat = pt["latitude"]
        if (pt["longitude"] > maxLon):
            maxLon = pt["longitude"]
        if (pt["longitude"] < minLon):
            minLon = pt["longitude"]
    
    minX = 0
    minY = 0
    maxYSearch, maxXSearch = decimal_to_cartesian(maxLat, maxLon, minLat, minLon)
    
    return minLat, minLon, maxLat, maxLon, minDist, minDistInd, maxXSearch, maxYSearch, minX, minY

def convertDataToFeet(searchGridPoints, stationaryObstacles, minLat, minLon):
    feetSearchGridPoints = []
    feetStationaryObstacles = []
    
    for i in searchGridPoints:
        curLat, curLon = decimal_to_cartesian(i["latitude"], i["longitude"], minLat, minLon)
        feetSearchGridPoints.append({"latitude" : curLat, "longitude" : curLon})
    
    for i in stationaryObstacles:
        curLat, curLon = decimal_to_cartesian(i["latitude"], i["longitude"], minLat, minLon)
        feetStationaryObstacles.append({"latitude" : curLat, "radius" : i["radius"], "longitude" : curLon, "height" : i["height"]})
    
    return feetSearchGridPoints, feetStationaryObstacles
    
# START PROGRAM

constAlt = 120
cameraWidth = 130
inputFile = "../../../mission_plan/example/interop_example.json"
#numLoops = [0, 0, 3]

def runProgram(constAlt, cameraWidth, inputFile, animation=True): #TODO Add Buffer
    #cameraWidth = 2 * constAlt * math.tan(math.radians(fov / 2))
    
    # Import data from interop_example
    searchGridPoints, stationaryObstacles = getSearchAndObPts(inputFile)
    
    # Set grid values
    minLat, minLon, maxLat, maxLon, minDist, minDistInd, maxXSearch, maxYSearch, minX, minY = getMinMaxGrid(searchGridPoints)
    
    # Convert data to feet
    feetSearchGridPoints, feetStationaryObstacles = convertDataToFeet(searchGridPoints, stationaryObstacles, minLat, minLon)
    
    
    for i in range(5):
        obstacleInGrid(feetSearchGridPoints, feetStationaryObstacles)
    feetStationaryObstacles = sorted(feetStationaryObstacles, key=lambda x: x['longitude'])
    
    tempSortedSearch = sorted(feetSearchGridPoints, key=lambda x: x["longitude"])
    
    
    
    
    extraTemp = []
    for i in tempSortedSearch:
        if {"latitude" : i["latitude"], "longitude" : i["longitude"]} not in extraTemp:
            extraTemp.append(i)
    
    tempSortedSearch = extraTemp
    
    cells = [Cell([], 0)]
    
    ct = 0
    for i in feetStationaryObstacles:
        ct += 1
        cells.append(Cell([], ct))
        
    
    
    curCell = 0
    i = 0
    while i < len(tempSortedSearch):
        if (tempSortedSearch[i]["longitude"] < feetStationaryObstacles[curCell]["longitude"]):
            cells[curCell].addPt({"longitude" : tempSortedSearch[i]["longitude"], "latitude" : tempSortedSearch[i]["latitude"]})
            i += 1
        elif (curCell < len(feetStationaryObstacles) - 1):
            curCell += 1
        else:
            curCell += 1
            break
    
    while i < len(tempSortedSearch):
        cells[curCell].addPt(tempSortedSearch[i])
        i += 1
    
    
    
    tempxgrid = []
    tempygrid = []
    
    
    for i in tempSortedSearch:
        tempxgrid.append(i["longitude"])
        tempygrid.append(i["latitude"])
    if animation: 
        plt.figure(1)
        
        
        
        plt.scatter(tempxgrid, tempygrid)
        
        for i in range(len(tempSortedSearch)):
            plt.text(tempSortedSearch[i]["longitude"], tempSortedSearch[i]["latitude"], str(i))
    cells[0].addPt(intersectionPtUp(feetStationaryObstacles[0], feetSearchGridPoints))
    cells[0].addPt(intersectionPtDown(feetStationaryObstacles[0], feetSearchGridPoints))
    
    cells[0].orderPoints()
    
    tempPts = getObstaclePts(feetStationaryObstacles[0], math.pi / 2, cameraWidth)
    
    for i in range(len(cells[0].searchPts)):
        if cells[0].searchPts[i]["longitude"] == feetStationaryObstacles[0]["longitude"]:
            for j in tempPts:
                cells[0].insertPt(i, j)
            break
    
    cells[len(cells) - 1].addPt(intersectionPtUp(feetStationaryObstacles[len(feetStationaryObstacles) - 1], feetSearchGridPoints))
    cells[len(cells) - 1].addPt(intersectionPtDown(feetStationaryObstacles[len(feetStationaryObstacles) - 1], feetSearchGridPoints))
    
    cells[len(cells) - 1].orderPoints()
    
    tempPts = getObstaclePts(feetStationaryObstacles[len(feetStationaryObstacles) - 1], math.pi * 3/2, cameraWidth)
    
    for i in range(len(cells[len(cells) - 1].searchPts)):
        if cells[len(cells) - 1].searchPts[i]["longitude"] == feetStationaryObstacles[len(feetStationaryObstacles) - 1]["longitude"]:
            for j in tempPts:
                cells[len(cells) - 1].insertPt(i+1, j)
            break
        
    for i in range(1, len(cells) - 1):
        cells[i].addPt(intersectionPtUp(feetStationaryObstacles[i-1], feetSearchGridPoints))
        cells[i].addPt(intersectionPtDown(feetStationaryObstacles[i-1], feetSearchGridPoints))
        
        cells[i].addPt(intersectionPtUp(feetStationaryObstacles[i], feetSearchGridPoints))
        cells[i].addPt(intersectionPtDown(feetStationaryObstacles[i], feetSearchGridPoints))
        
        
        cells[i].orderPoints()
        
        tempPts = getObstaclePts(feetStationaryObstacles[i], math.pi / 2, cameraWidth)
        
        for j in range(len(cells[i].searchPts)):
            if cells[i].searchPts[j]["longitude"] == feetStationaryObstacles[i]["longitude"]:
                for k in tempPts:
                    cells[i].insertPt(j, k)
                break
        
        tempPts = getObstaclePts(feetStationaryObstacles[i-1], math.pi * 3 / 2, cameraWidth)
        
        for j in range(len(cells[i].searchPts)):
            if cells[i].searchPts[j]["longitude"] == feetStationaryObstacles[i - 1]["longitude"]:
                for k in tempPts:
                    cells[i].insertPt(j + 1, k)
                break
    
    
    
    for i in cells:
        i.addStartPt()    
        if animation:
            i.plot()
        
    maxXGrid = maxXSearch
    maxYGrid = maxYSearch
    minXGrid = 0
    minYGrid = 0
    
    for i in feetStationaryObstacles:
        if (i["latitude"] + i["radius"] > maxYGrid):
            maxYGrid = i["latitude"] + i["radius"]
        if (i["longitude"] + i["radius"] > maxXGrid):
            maxXGrid = i["longitude"] + i["radius"]
        if (i["latitude"] - i["radius"] < minYGrid):
            minYGrid = i["latitude"] - i["radius"]
        if (i["longitude"] - i["radius"] < minXGrid):
            minXGrid = i["longitude"] - i["radius"]
    
    gridLength = maxXGrid - minXGrid
    gridHeight = maxYGrid - minYGrid
    minXGrid = -0.1 * gridLength
    maxXGrid = gridLength * 1.1
    minYGrid = -0.1 * gridHeight
    maxYGrid = gridHeight * 1.1
    gridLength = maxXGrid - minXGrid
    gridHeight = maxYGrid - minYGrid
    
    # variables to edit based on the physical plane
    
    
    xWayPts = []
    yWayPts = []
    altWayPts = []
    
    maxLon = feetSearchGridPoints[0]["longitude"]
    for i in range(len(feetSearchGridPoints)):
        curLon = feetSearchGridPoints[i]["longitude"]
        if (curLon > maxLon):
            maxLon = curLon
            minDistInd = i
    
    feetSearchGridPoints.pop()
    for i in range(minDistInd):
        feetSearchGridPoints.append(feetSearchGridPoints[0])
        feetSearchGridPoints.pop(0)
    feetSearchGridPoints.append(feetSearchGridPoints[0])
    
    for i in range(len(cells)):
        createPoints(2, cells[i-1].searchPts, 0, feetStationaryObstacles, xWayPts, yWayPts, altWayPts, cameraWidth, constAlt) 
    
    wayPts = []
    
    for i in range(len(xWayPts)):
        if (inObstacle(xWayPts[i], yWayPts[i], feetStationaryObstacles) != -1):    
            #print("Point x:" + str(xWayPts[i] + ", y: " + str(yWayPts[i]) + " is in an obstacle"))
            print("oopsie")
        else:
            curLatX, curLonX = cartesian_to_decimal(yWayPts[i], xWayPts[i], minLat, minLon) # FIXME Change after fix on Line 349
            wayPts.append({"latitude" : curLatX, "longitude" : curLonX, "altitude" : altWayPts[i]})
            '''
    filepath = "../../../mission_plan/searchpath.json"
    with open(filepath, "w") as file:
        json.dump(wayPts, file)
    # boundary points
    xGridPts = []
    yGridPts = []
    
    # obstacle points
    
    for pt in feetSearchGridPoints:
        yGridPts.append(pt["latitude"])
        xGridPts.append(pt["longitude"])
        
    
    plt.figure(figsize = [5, 5])
    ax = plt.axes([0.1, 0.1, 0.8, 0.8], xlim=(minXGrid, maxXGrid), ylim=(minYGrid, maxYGrid))
    plt.plot(xWayPts, yWayPts, color = "orange", marker = "o", markerfacecolor = "green", markeredgecolor = "green", lineWidth = cameraWidth / gridLength * 5 * 0.8 * 72, markersize = 3) #area
    plt.plot(xWayPts, yWayPts, color = "black", lineWidth = 1)
    plt.plot(xGridPts, yGridPts, color = "blue", linewidth = 3)
    
    
    for i in range(len(xGridPts)):
        plt.text(xGridPts[i], yGridPts[i], "B" + str(i))
    for i in range(len(feetStationaryObstacles)):
        ax.scatter([feetStationaryObstacles[i]["longitude"]], [feetStationaryObstacles[i]["latitude"]], s=(2 * feetStationaryObstacles[i]["radius"] / gridLength * 5 * 0.8 * 72) ** 2, c = "yellow", zorder = 3)
        plt.text(feetStationaryObstacles[i]["longitude"], feetStationaryObstacles[i]["latitude"], "O" + str(i))
    plt.gca().set_aspect('equal', adjustable = 'box')
    plt.xlabel("Horizontal (feet)")
    plt.ylabel("Vertical (feet)")
    plt.title("Map of boundary points, obstacles, and path")
    plt.grid()
    plt.show()
    
    
    
    for i in range(len(xWayPts)):
        if (not inObstacle(xWayPts[i], yWayPts[i], feetStationaryObstacles)):
            print("Error: point " + str(i) + " in obstacle " + str(inObstacle(xWayPts[i], yWayPts[i], feetStationaryObstacles)))
   ''' 
    return wayPts
    
def main():
    runProgram(constAlt, cameraWidth, inputFile)
    
