#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *

# Brain should be defined by default
brain=Brain()

drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
down_eye = EyeSensor("downeye", 5)
front_distance = Distance("frontdistance", 2)
distance = front_distance
left_distance = Distance("leftdistance", 3)
right_distance = Distance("rightdistance", 4)
location = Location("location", 9)

#endregion VEXcode Generated Robot Configuration
# ------------------------------------------
# 
# 	Project:      VEXcdrivetrain.turn_to_heading(90, DEGREES)
#	Author:       VEX
#	Created:
#	Description:  VEXcode VR Python Project
# 
# ------------------------------------------

# Add project code in "main"

import sys
import gc
from enum import Enum
from queue import PriorityQueue

class CMazePath:
    def __init__(self, prevNode, thisLoc):
        self._thisLoc = thisLoc
        self._cost = -1
        self._prevNode = prevNode
        self._nextNodes = [None, None, None]

    def __del__(self):
        brain.print("delete path node")
        brain.new_line()
#end class

def PathAppend(prevNode, nextLocs):
    if prevNode == None:
        return [None, None, None]
    if prevNode._nextNodes[0] == None and prevNode._nextNodes[1] == None and prevNode._nextNodes[2] == None:
        pass
    else:
        return prevNode._nextNodes
    for i in range(0, 3):
        if nextLocs[i] != None:
            tempGrid = GetGridFromLocation(nextLocs[i])
            if not tempGrid._isvalid:
                nextNode = CMazePath(prevNode, nextLocs[i])
                prevNode._nextNodes[i] = nextNode

    return prevNode._nextNodes

def PathPrune(parent, child):
    idx = parent._nextNodes.index(child)
    parent._nextNodes[idx] = None
    brain.print("removing node:", child._thisLoc, child)
    brain.new_line()
    del child

def DeletePath(node):
    # TODO:
    pass

def PrintPath(node, level = 0):
    if node == None: return
    if node._prevNode == None:
        brain.print("PP", level, ": ", "None", " ", node._thisLoc, " ", node._nextNodes)
    else:
        brain.print("PP", level, ": ", node._prevNode._thisLoc, " ", node._thisLoc, " ", node._nextNodes)
    brain.new_line()
    level = level + 1
    for nextNode in node._nextNodes:
        if nextNode != None:
            # for some reason does not work if on same line
            PrintPath(nextNode, level)

class CMazePoint:
    def __init__(self, row, col):
        self._row = row
        self._col = col
        self._x = 0
        self._y = 0
        self._isvalid = False
        self._isblocked = False
    
    def SetLocation(self, x, y):
        self._x = x
        self._y = y
        self._isvalid = True

    def SetValid(self):
        self._isvalid = True

    def SetBlocked(self):
        self._isblocked = True
# end class

# enum for robot actions
class ETurnTo(Enum):
    EFORWARD = 0
    ERIGHT = 1
    EBACKWARDS = 2
    ELEFT = 3

# distance sensors are mapped (left, right, front) in tuples
SensorMapping = [ETurnTo.ELEFT, ETurnTo.ERIGHT, ETurnTo.EFORWARD]

# enum for actual direction in maze (coords used is robot starts at 0,0 and N is +x E is +y) 
class EHeading(Enum):
    ENORTH = 0
    EEAST = 1
    ESOUTH = 2
    EWEST = 3
    UNDEF = 4

# X stored in ROWS, Y stored in COLS
# TODO: Extents checking
MAZEROWS = 20
MAZECOLS = 20
maze2D = []

robotLocation = (0, 0)
currentHeading = EHeading.ENORTH
origin = (0, 0)
goal = None
minExtent = (0, 0)
maxExtent = (0, 0)

def InitGrid():
    global maze2D
    maze2D = [[CMazePoint(i, j) for j in range(MAZECOLS)] for i in range(MAZEROWS)]

def LocationToGrid(loc):
    grid_row = (MAZEROWS + loc[0]) % MAZEROWS
    grid_col = (MAZECOLS + loc[1]) % MAZECOLS
    return((grid_row, grid_col))

def GetGridFromLocation(loc):
    global maze2D
    gridIdx = LocationToGrid(loc)
    gridPoint = maze2D[gridIdx[0]][gridIdx[1]]
    return gridPoint

def UpdateGrid(loc):
    global minExtent
    global maxExtent

    gridPoint = GetGridFromLocation(loc)
    if (gridPoint._isvalid):
        return
    gridPoint.SetLocation(loc[0], loc[1])

    if (loc[0] < minExtent[0]): minExtent = (loc[0], minExtent[1])
    elif (loc[0] > maxExtent[0]): maxExtent = (loc[0], maxExtent[1])
    if (loc[1] < minExtent[1]): minExtent = (minExtent[0], loc[1])
    elif (loc[1] > maxExtent[1]): maxExtent = (maxExtent[0], loc[1])

def GetGridPointFromTurn(robotLocation, currentHeading, turnDir):
    tempHeading = HeadingFromDirection(currentHeading, turnDir)
    tempLocation =  LocationFromHeading(robotLocation, tempHeading)
    tempGrid = GetGridFromLocation(tempLocation)
    return tempGrid

def GetNextNotVisitedNode(nextNodes):
    #brain.print("GetNextNotVisitedNode", nextNodes)
    #brain.new_line()
    selectedNode = None
    for node in nextNodes:
        if node != None:
            tempGrid = GetGridFromLocation(node._thisLoc)
            if not tempGrid._isvalid:
                selectedNode = node
                break
    return selectedNode

def LoadMaze(savedMaze):
    global maze2D
    global origin
    global goal
    global robotLocation

    j = MAZECOLS - 1
    i = 0

    for char in savedMaze:
        newline = True if ((ord(char)) == 10) else False
        if not newline and char != ".":
            maze2D[i][j]._x = i
            maze2D[i][j]._y = j
            maze2D[i][j]._isvalid = True
            if char == "x" or char == "X":
                pass
            elif char == "d" or char == "D":
                maze2D[i][j]._isblocked = True
            elif char == "o" or char == "O":
                origin = (i, j)
                robotLocation = origin
            elif char == "g" or char == "G":
                goal = (i, j)
        if newline:
            if i == 0:
                # blank line
                pass
            else:
                j = j - 1
            i = 0
        else:
            i = i + 1

    UpdateExtents()

def PrintGrid():
    # TODO: modulate with X/Y extents
    brain.new_line()
    for c in range(MAZECOLS):
        for r in range(MAZEROWS):
            j = (MAZECOLS + ((MAZECOLS - 1) - c + minExtent[1])) % MAZECOLS
            i = (MAZEROWS + (r + minExtent[0])) % MAZEROWS
            if maze2D[i][j]._isvalid:
                if maze2D[i][j]._x == origin[0] and maze2D[i][j]._y == origin[1]: brain.print("o")
                elif goal != None and maze2D[i][j]._x == goal[0] and maze2D[i][j]._y == goal[1]: brain.print("g")
                elif maze2D[i][j]._isblocked: brain.print("D")
                else: brain.print("x")
            else: brain.print(".")
        brain.new_line()
    brain.new_line()

# relative to current robot heading
def PickRandDirection(gaps, visited = (False, False, False)):
    # use choice() to pick from valid list of options
    pickList = []
    if gaps[0]: pickList.append(ETurnTo.ELEFT)
    if gaps[1]: pickList.append(ETurnTo.ERIGHT)
    if gaps[2]: pickList.append(ETurnTo.EFORWARD)

    # dead end (0 options exist)
    if len(pickList) == 0:
        pickDir = ETurnTo.EBACKWARDS
        return pickDir
    
    # straight or bend (1 option exists)
    if len(pickList) == 1:
        pickDir = pickList[0];
        return pickDir

    # junction (2 or 3 options exist)
    numVisited = 0;
    if visited[0]: numVisited = numVisited + 1
    if visited[1]: numVisited = numVisited + 1
    if visited[2]: numVisited = numVisited + 1

    # if none or all branches have already been taken
    if numVisited == 0 or numVisited == len(pickList):
        pickDir = random.choice(pickList)
        return pickDir

    brain.print("pruning")
    brain.new_line()
    pickList = []
    if gaps[0] and not visited[0]: pickList.append(ETurnTo.ELEFT)
    if gaps[1] and not visited[1]: pickList.append(ETurnTo.ERIGHT)
    if gaps[2] and not visited[2]: pickList.append(ETurnTo.EFORWARD)
    pickDir = random.choice(pickList)
    return pickDir

def CheckVisited(robotLocation, currentHeading, dirs):
    visited = [False, False, False]
    for i in range(0, len(dirs)):
        if (dirs[i]):
            tempGrid = GetGridPointFromTurn(robotLocation, currentHeading, SensorMapping[i])
            if tempGrid._isvalid: visited[i] = True
    retval = (visited[0], visited[1], visited[2])
    return retval

def PruneBlocked(robotLocation, currentHeading, dirs):
    blocked = [False, False, False]
    for i in range(0, len(dirs)):
        if (dirs[i]):
            tempGrid = GetGridPointFromTurn(robotLocation, currentHeading, SensorMapping[i])
            if tempGrid._isblocked: blocked[i] = True

    if (blocked[0] or blocked[1] or blocked[2]):
        brain.print("blocked")
        brain.new_line()

    retval = (dirs[0] and not blocked[0], dirs[1] and not blocked[1], dirs[2] and not blocked[2])
    return retval

# convert robot-centric to field N-S-E-W heading
def HeadingFromDirection(currentHeading, nextDirection):
    intHeading = (currentHeading.value + nextDirection.value) % 4
    newHeading = EHeading(intHeading)
    return newHeading

# convert robot-centric to field N-S-E-W heading
def DirectionFromHeading(currentHeading, nextHeading):
    intDirection = (4 + nextHeading.value - currentHeading.value) % 4
    newDirection = ETurnTo(intDirection)
    return newDirection

# gets the next X/Y location based on field heading
def LocationFromHeading(curLocation, robotHeading):
    if (robotHeading == EHeading.ENORTH): thisMove = (0, 1)
    elif (robotHeading == EHeading.ESOUTH): thisMove = (0, -1)
    elif (robotHeading == EHeading.EEAST): thisMove = (1, 0)
    elif (robotHeading == EHeading.EWEST): thisMove = (-1, 0)
    newLocation = (curLocation[0] + thisMove[0], curLocation[1] + thisMove[1])
    return newLocation

# gets next X/Y location from current location given robot heading (N-S-E-W) and robot relative turn (L-R-F)
def LocationFromDirection(currentLocation, currentHeading, nextDirection):
    nextHeading = HeadingFromDirection(currentHeading, nextDirection)
    nextLocation = LocationFromHeading(currentLocation, nextHeading)
    return nextLocation

# takes sensor tuple and checks if 2 or more options are available
def IsJunction(dirs):
    count = 0
    if dirs[0]: count = count + 1
    if dirs[1]: count = count + 1
    if dirs[2]: count = count + 1
    return count > 1

# takes sensor tuple and checks if no options are available
def IsBlocked(dirs):
    count = 0
    if dirs[0]: count = count + 1
    if dirs[1]: count = count + 1
    if dirs[2]: count = count + 1
    if count == 0:
        return True
    else:
        return False

# junction stored in tree
def IsPathJunction(node):
    if node == None:
        Assert(False)
    count = 0
    for nn in node._nextNodes:
        if nn != None:
            count = count + 1

# gets adjoining cell X/Y coordinates based on robot location, heading and L-R-F turn options
def DirectionsToLocations(loc, head, dirs):
    nextLocs = []
    for i in range(0, len(dirs)):
        if (dirs[i]):
            tempLoc = LocationFromDirection(loc, head, SensorMapping[i])
            nextLocs.append(tempLoc)
        else:
            nextLocs.append(None)
    return nextLocs

def NextLocationToTurn(currentLocation, nextLocation, robotHeading):
    if currentLocation[0] < nextLocation[0]: nextHeading = EHeading.EEAST
    elif currentLocation[0] > nextLocation[0]: nextHeading = EHeading.EWEST
    elif currentLocation[1] < nextLocation[1]: nextHeading = EHeading.ENORTH
    elif currentLocation[1] > nextLocation[1]: nextHeading = EHeading.ESOUTH
    else:
        nextHeading = EHeading.ENORTH

    headingDelta = (4 + nextHeading.value - robotHeading.value) % 4
    nextTurn = ETurnTo(headingDelta)

    return nextTurn

class EPathState(Enum):
    EHOME = 0
    ESEARCH = 1
    EDEADEND = 2
    EBACKTRACK = 3

def DiscoverMaze():
    global robotLocation
    global currentHeading
    global goal

    brain.print("Discovering maze ...")
    brain.new_line()

    path = CMazePath(None, robotLocation)
    currentNode = path

    currentHeading = EHeading.ENORTH
    lastLocation = None
    # bBackTrack = False
    bDeadEnd = False
    bGoalPath = False

    done = False
    currentState = EPathState.ESEARCH
    
    while not done:
        brain.new_line()
        brain.print("--- start: ", robotLocation, ", ", currentHeading)
        brain.new_line()

        # squares are 300mm, but current robot has 0 width, so pick something between 150 and 300
        bLeftGap = left_distance.get_distance(MM) > 225
        bRightGap = right_distance.get_distance(MM) > 225
        bFrontGap = front_distance.get_distance(MM) > 225
        gaps = (bLeftGap, bRightGap, bFrontGap)

        # generate some trivial bools based on what distance sensors see
        # dead-end is sticky until the parent junction is reached so path can be trivially rejected in the future
        # TODO: don't reject forks with the goal or origin
        # TODO: only flags trivial case of leaf paths. dead-end paths with forks could be flagged as well
        bJunction = IsJunction(gaps)
        bBlocked = IsBlocked(gaps)
        # based on sensors, determine adjoining X/Y locations and then see if we have any new paths
        # TODO: Once we have known paths, could just steer directly this way and sanity checkw with sensors
        nextLocations = DirectionsToLocations(robotLocation, currentHeading, gaps)

        if not bGoalPath and not bDeadEnd and bBlocked:
            brain.print("dead-end")
            brain.new_line()
            bDeadEnd = True
        if bJunction: bGoalPath = False

        if currentState == EPathState.ESEARCH:
            brain.print("SEARCH")
            brain.new_line()
            nextNodes = PathAppend(currentNode, nextLocations)
            brain.print(nextNodes)
            brain.new_line()
            selectedNode = GetNextNotVisitedNode(nextNodes)
            if selectedNode != None:
                brain.print("--NEW SEARCH", selectedNode._thisLoc)
                brain.new_line()
                nextTurn = NextLocationToTurn(robotLocation, selectedNode._thisLoc, currentHeading)
                nextNode = selectedNode
                nextState = EPathState.ESEARCH
            else:
                brain.print("--END SEARCH")
                brain.new_line()
                nextTurn = ETurnTo.EBACKWARDS
                nextNode = currentNode._prevNode
                nextState = EPathState.EBACKTRACK
                PathPrune(nextNode, currentNode)

        elif currentState == EPathState.EBACKTRACK:
            brain.print("BACKTRACK")
            brain.new_line()
            prevNode = currentNode._prevNode
            bPathTaken = False
            if currentNode != None:
                selectedNode = GetNextNotVisitedNode(currentNode._nextNodes)
                if selectedNode != None:
                    brain.print("--NEW SEARCH", selectedNode._thisLoc)
                    brain.new_line()
                    nextTurn = NextLocationToTurn(robotLocation, selectedNode._thisLoc, currentHeading)
                    nextNode = selectedNode
                    nextState = EPathState.ESEARCH
                    bPathTaken = True
                elif prevNode != None:
                    brain.print("--CONTINUE BACK")
                    brain.new_line()
                    prevNode = currentNode._prevNode
                    nextTurn = NextLocationToTurn(robotLocation, prevNode._thisLoc, currentHeading)
                    nextNode = prevNode
                    nextState = EPathState.EBACKTRACK
                    PathPrune(nextNode, currentNode)
                    bPathTaken = True
                else:
                    nextNodes = PathAppend(currentNode, nextLocations)
                    brain.print(nextNodes)
                    brain.new_line()
                    selectedNode = GetNextNotVisitedNode(nextNodes)
                    if selectedNode != None:
                        brain.print("--NEW SEARCH", selectedNode._thisLoc)
                        brain.new_line()
                        nextTurn = NextLocationToTurn(robotLocation, selectedNode._thisLoc, currentHeading)
                        nextNode = selectedNode
                        nextState = EPathState.ESEARCH
                        bPathTaken = True
                        bDeadEnd = False

            else:
                brain.print("at path root")
                brain.new_line()                

            if not bPathTaken:
                brain.print("--AT HOME")
                brain.new_line()
                PrintPath(path)
                nextTurn = ETurnTo.EBACKWARDS
                nextState = EPathState.EHOME
                done = True

            if bJunction:
                pass
            else:
                pass

        elif currentState == EPathState.EHOME:
            brain.print("HOME")
            brain.new_line()

        else:
            pass

        currentNode = nextNode
        currentState = nextState

        nextHeading = HeadingFromDirection(currentHeading, nextTurn)

        if nextTurn == ETurnTo.ELEFT: drivetrain.turn_for(LEFT,90,DEGREES)
        elif nextTurn == ETurnTo.ERIGHT: drivetrain.turn_for(RIGHT,90,DEGREES)
        elif nextTurn == ETurnTo.EBACKWARDS: drivetrain.turn_for(LEFT,180,DEGREES)
        else: pass # go forward   
        currentHeading = nextHeading
        
        if not done:
            drivetrain.drive_for(FORWARD,300,MM)

            # update location, heading etc.
            # if not bBackTrack and nextTurn == ETurnTo.EBACKWARDS: bBackTrack = True
            if bJunction and bDeadEnd:
                bDeadEnd = False
            if bDeadEnd:
                brain.print("mark dead-end @", robotLocation)
                brain.new_line()
                gridPoint = GetGridFromLocation(robotLocation)
                gridPoint.SetBlocked()

            lastLocation = robotLocation
            robotLocation = LocationFromHeading(robotLocation, currentHeading)
            UpdateGrid(robotLocation)

        if goal == None and down_eye.detect(RED):
            goal = robotLocation
            brain.print("GOAL FOUND: ", goal)
            brain.new_line()
            PrintGrid()
            bGoalPath = True

    del path
    path = None
    gc.collect()

    brain.print("Discover Maze Done")
    brain.new_line()
    wait(1,SECONDS)


def DiscoverMazeRandom():
    global path
    global robotLocation
    global goal

    brain.print("Discovering maze ...")
    brain.new_line()

    currentHeading = EHeading.ENORTH
    lastLocation = None
    bDeadEnd = False

    done = False
    
    while not done:
        # squares are 300mm, but current robot has 0 width, so pick something between 150 and 300
        bLeftGap = left_distance.get_distance(MM) > 225
        bRightGap = right_distance.get_distance(MM) > 225
        bFrontGap = front_distance.get_distance(MM) > 225
        gaps = (bLeftGap, bRightGap, bFrontGap)

        gaps = PruneBlocked(robotLocation, currentHeading, gaps)
        visited = (False, False, False)

        bJunction = IsJunction(gaps)
        if bJunction:
            visited = CheckVisited(robotLocation, currentHeading, gaps)

        nextDir = PickRandDirection(gaps, visited)
        nextHeading = HeadingFromDirection(currentHeading, nextDir)

        if nextDir == ETurnTo.ELEFT: drivetrain.turn_for(LEFT,90,DEGREES)
        elif nextDir == ETurnTo.ERIGHT: drivetrain.turn_for(RIGHT,90,DEGREES)
        elif nextDir == ETurnTo.EBACKWARDS: drivetrain.turn_for(LEFT,180,DEGREES)
        else: pass # go forward   
        currentHeading = nextHeading
        
        if not bDeadEnd and nextDir == ETurnTo.EBACKWARDS: bDeadEnd = True
        if bJunction and bDeadEnd:
            gridPoint = GetGridFromLocation(lastLocation)
            gridPoint.SetBlocked()
            bDeadEnd = False

        drivetrain.drive_for(FORWARD,300,MM)

        lastLocation = robotLocation
        robotLocation = LocationFromHeading(robotLocation, currentHeading)
        UpdateGrid(robotLocation)

        if down_eye.detect(RED):
            goal = robotLocation
            done = True

def ValidActions(valids, qx, qy):
    valid_actions = []
    xsize = len(valids)
    ysize = len(valids[0])
    if qx < xsize - 1:
        if valids[qx+1][qy]:
            valid_actions.append(((qx+1, qy), EHeading.EEAST.value))
    if qy < ysize - 1:
        if valids[qx][qy+1]:
            valid_actions.append(((qx, qy+1), EHeading.ENORTH.value))
    if qx > 0:
        if valids[qx-1][qy]:
            valid_actions.append(((qx-1, qy), EHeading.EWEST.value))
    if qy > 0:
        if valids[qx][qy-1]:
            valid_actions.append(((qx, qy-1), EHeading.ESOUTH.value))
    return valid_actions

def UpdateExtents():
    global minExtent
    global maxExtent

    minExtent = (0, 0)
    maxExtent = (0, 0)

    for i in range(MAZEROWS):
        for j in range(MAZECOLS):
            if maze2D[i][j]._isvalid:
                x, y = maze2D[i][j]._x, maze2D[i][j]._y
                if x > maxExtent[0]: maxExtent = (x, maxExtent[1])
                if y > maxExtent[1]: maxExtent = (maxExtent[0], y)
                if x < minExtent[0]: minExtent = (x, minExtent[1])
                if y < minExtent[1]: minExtent = (minExtent[0], y)

    brain.print("Extents: ", minExtent, maxExtent)
    brain.new_line()

def ReverseOptimize(valids):
    # start at goal and trace back to origin
    # stores location, cost in distance and turns

    if goal == None: return

    mazeSize = (len(valids), len(valids[0]))
    goal_norm = (goal[0] - minExtent[0], goal[1] - minExtent[1])
    origin_norm = (origin[0] - minExtent[0], origin[1] - minExtent[1])

    visited = [[False for j in range(mazeSize[1])] for i in range(mazeSize[0])]
    costs = [[-1 for j in range(mazeSize[1])] for i in range(mazeSize[0])]

    visitlist = set(goal_norm)
    queue = PriorityQueue()
    queue.put((0, goal_norm, EHeading.UNDEF.value))
    max_cost = 1

    while not queue.empty():
        item = queue.get()
        queue_cost = item[0]
        queue_loc = item[1]
        queue_head = item[2]
        qx, qy = queue_loc[0], queue_loc[1]
        # TODO: Add check if got visited by the time we got here
        if visited[qx][qy]:
            if costs[qx][qy] > queue_cost:
                costs[qx][qy] = queue_cost
        else:
            costs[qx][qy] = queue_cost
        visited[qx][qy] = True
        
        valid_actions = ValidActions(valids, qx, qy)

        if queue_cost > max_cost:
            max_cost = queue_cost
        if queue_head == EHeading.UNDEF.value:
            first_action = valid_actions[0]
            queue_head = first_action[1]
        for va in valid_actions:
            loc = va[0]
            head = va[1]
            if visited[loc[0]][loc[1]]:
                continue
            turn_cost = 0 if queue_head == head else 2
            # turn_cost = 0
            next_cost = queue_cost + 1 + turn_cost
            brain.print(next_cost, loc, EHeading(head))
            brain.new_line()
            queue.put((next_cost, loc, head))

        if False:
            brain.new_line()
            for y in range(mazeSize[1] - 1, -1, -1):
                for x in range(mazeSize[0]):
                    if visited[x][y]: brain.print("x")
                    else: brain.print(".")
                brain.new_line()

    if True:
        brain.new_line()
        for y in range(mazeSize[1] - 1, -1, -1):
            for x in range(mazeSize[0]):
                if costs[x][y] >= 0:
                    dc = int(costs[x][y] * 9 / max_cost)
                    brain.print(dc)
                else: brain.print(".")
            brain.new_line()

    path = []
    current_loc = origin_norm
    current_cost = costs[current_loc[0]][current_loc[1]]
    path.append(origin_norm)
    bAbort = False
    while not bAbort and (current_loc != goal_norm):
        next_loc = loc
        next_cost = current_cost
        valid_actions = ValidActions(valids, current_loc[0], current_loc[1])
        bAbort = True
        for va in valid_actions:
            loc = va[0]
            if costs[loc[0]][loc[1]] < next_cost:
                next_loc = loc
                next_cost = costs[loc[0]][loc[1]] 
                bAbort = False
        if not bAbort:
            current_loc = next_loc
            current_cost = next_cost
            path.append(current_loc)

    if bAbort:
        brain.print("ABORTED SEARCH")
        brain.new_line()
        path = None
    brain.new_line()
    brain.print("Steps", len(path))
    brain.new_line()
    brain.print(path)
    brain.new_line()

    return path

def GenerateInstructions(path):

    if path == None: return None

    instructions = []
    last_heading = EHeading.UNDEF
    this_heading = EHeading.UNDEF
    step_count = 1
    last_point = None
    for point in path:
        if last_point != None:
            if point[0] > last_point[0]: this_heading = EHeading.EEAST
            elif point[0] < last_point[0]: this_heading = EHeading.EWEST
            elif point[1] > last_point[1]: this_heading = EHeading.ENORTH
            elif point[1] < last_point[1]: this_heading = EHeading.ESOUTH

            if last_heading != EHeading.UNDEF:
                if this_heading != last_heading:
                    instructions.append((step_count, last_heading))
                    step_count = 1
                else:
                    step_count = step_count + 1

            last_heading = this_heading

        last_point = point

    instructions.append((step_count, last_heading))

    brain.print(instructions)
    brain.new_line()

    return instructions

savedMaze1 = """
....................
....................
....................
....................
....................
..........xxx.g.....
DDDDDD....x.x.x.....
.....D..xxx.xxx.....
..DDDD..x...x.......
..D.....x...x.......
DDD..xxxxx..x.......
D....x.x.xxxxx......
D....x.x.D...x......
D..xxx.x.D...x......
DDDx...x.DDD.x......
...x...x.....x......
xxxxxx.x.....x......
x.D..xxxxxxxxx......
x.D..D.....x.x......
o.D..DDDD..xxx......
"""

savedMaze2 = """
....................
....................
....................
....................
....................
....................
....................
....................
....................
DDD.................
..xD................
DDx.................
..x.................
..x.................
xxxxx...............
x...xgDD............
x...................
xxx.................
..xD................
oxx.................
"""

def main():
    global robotLocation
    global currentHeading
    global origin
    global goal
    global minExtent
    global maxExtent

    brain.clear()
    brain.print("hello, ver", sys.version)
    brain.new_line()
    brain.new_line()    

    InitGrid()
    PrintGrid()

    savedMaze = None

    if savedMaze == None:
        UpdateGrid(robotLocation)
        DiscoverMaze()
        brain.new_line()
        brain.print("Discovered Maze: origin ", origin, ", goal ", goal, ", loc ", robotLocation)
        brain.new_line()
        PrintGrid()
    else:
        LoadMaze(savedMaze)
        brain.new_line()
        brain.print("Saved Maze: origin ", origin, ", goal ", goal, ", loc ", robotLocation)
        brain.new_line()
        PrintGrid()

    # followed structures used normalized to 0, 0 coordinates. original maze will not be normalized

    # UpdateExtents()
    
    mazeSize = (maxExtent[0] - minExtent[0] + 1, maxExtent[1] - minExtent[1] + 1)
    valids = [[False for j in range(mazeSize[1])] for i in range(mazeSize[0])]

    for i in range(MAZEROWS):
        for j in range(MAZECOLS):
            if maze2D[i][j]._isvalid and not maze2D[i][j]._isblocked:
                # maze used for discovery is not normalized
                x, y = maze2D[i][j]._x - minExtent[0], maze2D[i][j]._y - minExtent[1]
                valids[x][y] = True

    path = ReverseOptimize(valids)
    instructions = GenerateInstructions(path)

    if instructions == None: return

    brain.print("Following instructions ...", currentHeading)
    brain.new_line()

    for instr in instructions:
        nextHeading = instr[1]
        steps = instr[0]
        nextDir = DirectionFromHeading(currentHeading, nextHeading)
        if nextDir == ETurnTo.ELEFT: drivetrain.turn_for(LEFT,90,DEGREES)
        elif nextDir == ETurnTo.ERIGHT: drivetrain.turn_for(RIGHT,90,DEGREES)
        elif nextDir == ETurnTo.EBACKWARDS: drivetrain.turn_for(LEFT,180,DEGREES)
        else: pass # go forward   
        currentHeading = nextHeading
        drivetrain.drive_for(FORWARD,steps*300,MM)

# VR threads — Do not delete
vr_thread(main)
