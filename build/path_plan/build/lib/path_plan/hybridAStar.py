from .CurvesGenerator import reeds_shepp as rsCurve
from .obstacleMap import map

import math
import heapq
from heapdict import heapdict
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

class Car:
    maxSteerAngle = 0.6   # 最大转向角
    steerPresion = 10     # 转向精度  小车每次只能旋转 maxSteerAngle / steerPresion
    wheelBase = 0.18 / 0.05
    axleToFront = 0.28 / 0.05
    axleToBack = 0.02 / 0.05
    width = 0.24 / 0.05

class Cost:
    reverse = 100        
    directionChange = 150
    steerAngle = 1
    steerAngleChange = 5
    hybridCost = 50

class Node:
    def __init__(self, gridIndex, traj, steeringAngle, direction, cost, parentIndex):
        self.gridIndex = gridIndex         # grid block x, y, yaw index
        self.traj = traj                   # trajectory x, y  of a simulated node
        self.steeringAngle = steeringAngle # steering angle throughout the trajectory
        self.direction = direction         # direction throughout the trajectory
        self.cost = cost                   # node cost
        self.parentIndex = parentIndex     # parent node index

class HolonomicNode:
    def __init__(self, gridIndex, cost, parentIndex):
        self.gridIndex = gridIndex
        self.cost = cost
        self.parentIndex = parentIndex

class MapParameters:
    def __init__(self, mapMinX, mapMinY, mapMaxX, mapMaxY, xyResolution, yawResolution, ObstacleKDTree, obstacleX, obstacleY):
        self.mapMinX = mapMinX               # map min x coordinate(0)
        self.mapMinY = mapMinY               # map min y coordinate(0)
        self.mapMaxX = mapMaxX               # map max x coordinate
        self.mapMaxY = mapMaxY               # map max y coordinate
        self.xyResolution = xyResolution     # grid block length
        self.yawResolution = yawResolution   # grid block possible yaws
        self.ObstacleKDTree = ObstacleKDTree # KDTree representating obstacles
        self.obstacleX = obstacleX           # Obstacle x coordinate list
        self.obstacleY = obstacleY           # Obstacle y coordinate list

def calculateMapParameters(obstacleX, obstacleY, xyResolution, yawResolution):
        
        # calculate min max map grid index based on obstacles in map
        # 基于地图中的障碍物计算最小-最大网格下标
        mapMinX = round(min(obstacleX) / xyResolution)
        mapMinY = round(min(obstacleY) / xyResolution)
        mapMaxX = round(max(obstacleX) / xyResolution)
        mapMaxY = round(max(obstacleY) / xyResolution)
        # print(f"最小网格索引{mapMinX, mapMinY}, 最大网格索引{mapMaxX, mapMaxY}")
        # create a KDTree to represent obstacles
        ObstacleKDTree = KDTree([[x, y] for x, y in zip(obstacleX, obstacleY)])

        return MapParameters(mapMinX, mapMinY, mapMaxX, mapMaxY, xyResolution, yawResolution, ObstacleKDTree, obstacleX, obstacleY)  

def index(Node):
    # Index is a tuple consisting grid index, used for checking if two nodes are near/same
    # Index是由网格索引组成的元组,用于检查两个节点是否接近/相同
    return tuple([Node.gridIndex[0], Node.gridIndex[1], Node.gridIndex[2]])

def motionCommands():

    # Motion commands for a Non-Holonomic Robot like a Car or Bicycle (Trajectories using Steer Angle and Direction)
    # 汽车或自行车等非完整机器人的运动命令（使用转向角度和方向的轨迹）
    direction = 1
    motionCommand = []
    # 从最大转向角度开始递减到负的最大转向角度,步长为最大转向角度除以转向精度
    for i in np.arange(Car.maxSteerAngle, -(Car.maxSteerAngle + Car.maxSteerAngle/Car.steerPresion), -Car.maxSteerAngle/Car.steerPresion):
        motionCommand.append([i, direction])
        motionCommand.append([i, -direction])
    return motionCommand

def holonomicMotionCommands():

    # Action set for a Point/Omni-Directional/Holonomic Robot (8-Directions)
    # 点/全向/整体机器人的动作集（8个方向）
    holonomicMotionCommand = [[-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1]]
    return holonomicMotionCommand


def kinematicSimulationNode(currentNode, motionCommand, mapParameters, simulationLength=4, step = 0.8 ):
    # Simulate node using given current Node and Motion Commands
    # 使用给定的当前“节点”和“运动”命令模拟节点
    traj = []
    angle = rsCurve.pi_2_pi(currentNode.traj[-1][2] + motionCommand[1] * step / Car.wheelBase * math.tan(motionCommand[0]))
    traj.append([currentNode.traj[-1][0] + motionCommand[1] * step * math.cos(angle),
                currentNode.traj[-1][1] + motionCommand[1] * step * math.sin(angle),
                rsCurve.pi_2_pi(angle + motionCommand[1] * step / Car.wheelBase * math.tan(motionCommand[0]))])
    for i in range(int((simulationLength/step))-1):
        traj.append([traj[i][0] + motionCommand[1] * step * math.cos(traj[i][2]),
                    traj[i][1] + motionCommand[1] * step * math.sin(traj[i][2]),
                    rsCurve.pi_2_pi(traj[i][2] + motionCommand[1] * step / Car.wheelBase * math.tan(motionCommand[0]))])

    # Find grid index
    gridIndex = [round(traj[-1][0]/mapParameters.xyResolution), \
                 round(traj[-1][1]/mapParameters.xyResolution), \
                 round(traj[-1][2]/mapParameters.yawResolution)]

    # Check if node is valid
    if not isValid(traj, gridIndex, mapParameters):
        return None

    # Calculate Cost of the node
    cost = simulatedPathCost(currentNode, motionCommand, simulationLength)

    return Node(gridIndex, traj, motionCommand[0], motionCommand[1], cost, index(currentNode))

def reedsSheppNode(currentNode, goalNode, mapParameters):

    # Get x, y, yaw of currentNode and goalNode
    startX, startY, startYaw = currentNode.traj[-1][0], currentNode.traj[-1][1], currentNode.traj[-1][2]
    goalX, goalY, goalYaw = goalNode.traj[-1][0], goalNode.traj[-1][1], goalNode.traj[-1][2]

    # Instantaneous Radius of Curvature
    radius = math.tan(Car.maxSteerAngle)/Car.wheelBase

    # Find all possible reeds-shepp paths between current and goal node
    reedsSheppPaths = rsCurve.calc_all_paths(startX, startY, startYaw, goalX, goalY, goalYaw, radius, 1)
    # Check if reedsSheppPaths is empty
    if not reedsSheppPaths:
        return None

    # Find path with lowest cost considering non-holonomic constraints
    costQueue = heapdict()
    for path in reedsSheppPaths:
        costQueue[path] = reedsSheppCost(currentNode, path)

    # Find first path in priority queue that is collision free
    while len(costQueue)!=0:
        path = costQueue.popitem()[0]
        traj=[]
        traj = [[path.x[k],path.y[k],path.yaw[k]] for k in range(len(path.x))]
        # print(traj)
        if not collision(traj, mapParameters):
            cost = reedsSheppCost(currentNode, path)
            return Node(goalNode.gridIndex ,traj, None, None, cost, index(currentNode))
            
    return None

def isValid(traj, gridIndex, mapParameters):

    # Check if Node is out of map bounds
    if gridIndex[0]<=mapParameters.mapMinX or gridIndex[0]>=mapParameters.mapMaxX or \
       gridIndex[1]<=mapParameters.mapMinY or gridIndex[1]>=mapParameters.mapMaxY:
        return False

    # Check if Node is colliding with an obstacle
    if collision(traj, mapParameters):
        return False
    return True

# 轨迹碰撞检测
def collision(traj, mapParameters):

    carRadius = (Car.axleToFront + Car.axleToBack)/2 + 1
    dl = (Car.axleToFront - Car.axleToBack)/2
    for i in traj:
        cx = i[0] + dl * math.cos(i[2])
        cy = i[1] + dl * math.sin(i[2])
        pointsInObstacle = mapParameters.ObstacleKDTree.query_ball_point([cx, cy], carRadius)

        if not pointsInObstacle:
            continue

        for p in pointsInObstacle:
            xo = mapParameters.obstacleX[p] - cx
            yo = mapParameters.obstacleY[p] - cy
            dx = xo * math.cos(i[2]) + yo * math.sin(i[2])
            dy = -xo * math.sin(i[2]) + yo * math.cos(i[2])

            if abs(dx) < carRadius and abs(dy) < Car.width / 2 + 1:
                return True

    return False

def reedsSheppCost(currentNode, path):

    # Previos Node Cost
    cost = currentNode.cost

    # Distance cost
    for i in path.lengths:
        if i >= 0:
            cost += 1
        else:
            cost += abs(i) * Cost.reverse

    # Direction change cost
    for i in range(len(path.lengths)-1):
        if path.lengths[i] * path.lengths[i+1] < 0:
            cost += Cost.directionChange

    # Steering Angle Cost
    for i in path.ctypes:
        # Check types which are not straight line
        if i!="S":
            cost += Car.maxSteerAngle * Cost.steerAngle

    # Steering Angle change cost
    turnAngle=[0.0 for _ in range(len(path.ctypes))]
    for i in range(len(path.ctypes)):
        if path.ctypes[i] == "R":
            turnAngle[i] = - Car.maxSteerAngle
        if path.ctypes[i] == "WB":
            turnAngle[i] = Car.maxSteerAngle

    for i in range(len(path.lengths)-1):
        cost += abs(turnAngle[i+1] - turnAngle[i]) * Cost.steerAngleChange

    return cost

def simulatedPathCost(currentNode, motionCommand, simulationLength):

    # Previos Node Cost
    cost = currentNode.cost

    # Distance cost
    if motionCommand[1] == 1:
        cost += simulationLength 
    else:
        cost += simulationLength * Cost.reverse

    # Direction change cost
    if currentNode.direction != motionCommand[1]:
        cost += Cost.directionChange

    # Steering Angle Cost
    cost += motionCommand[0] * Cost.steerAngle

    # Steering Angle change cost
    cost += abs(motionCommand[0] - currentNode.steeringAngle) * Cost.steerAngleChange

    return cost

def eucledianCost(holonomicMotionCommand):
    # Compute Eucledian Distance between two nodes
    return math.hypot(holonomicMotionCommand[0], holonomicMotionCommand[1])

def holonomicNodeIndex(HolonomicNode):
    # Index is a tuple consisting grid index, used for checking if two nodes are near/same
    return tuple([HolonomicNode.gridIndex[0], HolonomicNode.gridIndex[1]])

def obstaclesMap(obstacleX, obstacleY, xyResolution):

    # Compute Grid Index for obstacles
    # 计算障碍物在网格地图中处于哪块网格内
    obstacleX = [round(x / xyResolution) for x in obstacleX]
    obstacleY = [round(y / xyResolution) for y in obstacleY]

    # Set all Grid locations to No Obstacle
    obstacles =[[False for i in range(max(obstacleY)+1)]for i in range(max(obstacleX)+1)]

    # Set Grid Locations with obstacles to True
    # 将有障碍物的栅格位置设置为True
    # for x in range(max(obstacleX)+1):
    #     for y in range(max(obstacleY)+1):
    #         for i, j in zip(obstacleX, obstacleY):
    #             if math.hypot(i-x, j-y) <= 1/2:
    #                 obstacles[i][j] = True
    #                 break

    # 优化三层循环
    for i, j in zip(obstacleX, obstacleY):
        if obstacles[i][j] == False:
            obstacles[i][j] = True

    return obstacles

def holonomicNodeIsValid(neighbourNode, obstacles, mapParameters):

    # Check if Node is out of map bounds
    if neighbourNode.gridIndex[0]<= mapParameters.mapMinX or \
       neighbourNode.gridIndex[0]>= mapParameters.mapMaxX or \
       neighbourNode.gridIndex[1]<= mapParameters.mapMinY or \
       neighbourNode.gridIndex[1]>= mapParameters.mapMaxY:
        return False

    # Check if Node on obstacle
    if obstacles[neighbourNode.gridIndex[0]][neighbourNode.gridIndex[1]]:
        return False

    return True

def holonomicCostsWithObstacles(goalNode, mapParameters):

    gridIndex = [round(goalNode.traj[-1][0]/mapParameters.xyResolution), round(goalNode.traj[-1][1]/mapParameters.xyResolution)]
    gNode =HolonomicNode(gridIndex, 0, tuple(gridIndex))

    # 获取网格地图布尔矩阵 True表示有障碍
    obstacles = obstaclesMap(mapParameters.obstacleX, mapParameters.obstacleY, mapParameters.xyResolution)

    # 机器人的动作集（8个方向）
    holonomicMotionCommand = holonomicMotionCommands()

    openSet = {holonomicNodeIndex(gNode): gNode}
    closedSet = {}

    priorityQueue =[]
    heapq.heappush(priorityQueue, (gNode.cost, holonomicNodeIndex(gNode)))

    while True:
        if not openSet:
            break
        # 取出优先队列中cost最小的节点网格下标
        _, currentNodeIndex = heapq.heappop(priorityQueue)
        # 根据网格下标在open集中取出节点
        currentNode = openSet[currentNodeIndex]
        # 将节点从open集中删除
        openSet.pop(currentNodeIndex)
        # 将节点添加进close集
        closedSet[currentNodeIndex] = currentNode

        # 从当前节点出发往八个方向寻找相邻节点 并用欧式距离计算邻居节点的H值(邻居节点与当前节点的估计消耗)
        for i in range(len(holonomicMotionCommand)):
            neighbourNode = HolonomicNode([currentNode.gridIndex[0] + holonomicMotionCommand[i][0],\
                                      currentNode.gridIndex[1] + holonomicMotionCommand[i][1]],\
                                      currentNode.cost + eucledianCost(holonomicMotionCommand[i]), currentNodeIndex)

            # 边界及碰撞检测
            if not holonomicNodeIsValid(neighbourNode, obstacles, mapParameters):
                continue

            neighbourNodeIndex = holonomicNodeIndex(neighbourNode)

            if neighbourNodeIndex not in closedSet: 
                # 如果邻居网格在close中 跳过           
                if neighbourNodeIndex in openSet:
                    if neighbourNode.cost < openSet[neighbourNodeIndex].cost:
                        # 如当前邻居节点所处网格不在close集中（未被添加进路径）,但如果它在open集（备选集）中,
                        # 检查这条路径 (即经由当前节点到达该节点) 是否更好,用 H 值作参考，更小的 H 值表示这是更好的路径。
                        # 如果是这样,把它的父亲设置为当前节点,并更新所处网格的 H 值。
                        openSet[neighbourNodeIndex].cost = neighbourNode.cost
                        openSet[neighbourNodeIndex].parentIndex = neighbourNode.parentIndex
                        # heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNodeIndex))
                else:
                    # 如果当前邻居节点所处网格不在open集中 就添加进入open集(备选)
                    openSet[neighbourNodeIndex] = neighbourNode
                    heapq.heappush(priorityQueue, (neighbourNode.cost, neighbourNodeIndex))

    # holonomicCost = [[np.inf for i in range(max(mapParameters.obstacleY))]for i in range(max(mapParameters.obstacleX))]
    holonomicCost = [[np.inf for i in range(mapParameters.mapMaxY+1)]for i in range(mapParameters.mapMaxX+1)]
    # 遍历结束后的close集就是距离终点的最短路径(扩展节点到目标点的代价最小)
    for nodes in closedSet.values():
        holonomicCost[nodes.gridIndex[0]][nodes.gridIndex[1]]=nodes.cost
    return holonomicCost


def backtrack(startNode, goalNode, closedSet):
    # Goal Node data
    startNodeIndex= index(startNode)
    currentNodeIndex = goalNode.parentIndex
    currentNode = closedSet[currentNodeIndex]
    x=[]
    y=[]
    yaw=[]

    # Iterate till we reach start node from goal node
    while currentNodeIndex != startNodeIndex:
        a, b, c = zip(*currentNode.traj)
        x += a[::-1] # 取从后向前（相反）的元素
        y += b[::-1] 
        yaw += c[::-1]
        currentNodeIndex = currentNode.parentIndex
        currentNode = closedSet[currentNodeIndex]
    return x[::-1], y[::-1], yaw[::-1]

def run(s, g, mapParameters):

    # Compute Grid Index for start and Goal node
    sGridIndex = [round(s[0] / mapParameters.xyResolution), \
                  round(s[1] / mapParameters.xyResolution), \
                  round(s[2]/mapParameters.yawResolution)]
    gGridIndex = [round(g[0] / mapParameters.xyResolution), \
                  round(g[1] / mapParameters.xyResolution), \
                  round(g[2]/mapParameters.yawResolution)]
    print(f"起点网格索引{sGridIndex},终点网格索引{gGridIndex}")

    # Generate all Possible motion commands to car
    # 生成所有可能的小车转向运动指令
    motionCommand = motionCommands()

    # Create start and end Node
    startNode = Node(sGridIndex, [s], 0, 1, 0 , tuple(sGridIndex))
    goalNode = Node(gGridIndex, [g], 0, 1, 0, tuple(gGridIndex))

    # Find Holonomic Heuristric 
    # 启发函数 计算所有节点到终点的估计消耗
    holonomicHeuristics = holonomicCostsWithObstacles(goalNode, mapParameters)

    '''
    * 初始化open_set和close_set,及优先级队列;
    * 将起点加入open_set和优先级队列中,并设置优先级为0(优先级最高);
    * 如果open_set不为空,则从open_set中选取优先级最高的节点n:
        * 如果节点n为终点,则:
            * 从终点开始逐步追踪parent节点,一直达到起点;
            * 返回找到的结果路径,算法结束;
        * 如果节点n不是终点,则:
            * 将节点n从open_set中删除,并加入close_set中;
            * 遍历节点n所有的邻近节点:
                * 如果邻近节点m在close_set中,则:
                    * 跳过,选取下一个邻近节点
                * 如果邻近节点m也不在open_set中,则:
                    * 设置节点m的parent为节点n
                    * 计算节点m的优先级
                    * 将节点m加入open_set中
    '''

    # Add start node to open Set
    openSet = {index(startNode):startNode}
    closedSet = {}

    # Create a priority queue for acquiring nodes based on their cost's
    costQueue = heapdict()

    # Add start mode into priority queue
    costQueue[index(startNode)] = max(startNode.cost , Cost.hybridCost * holonomicHeuristics[startNode.gridIndex[0]][startNode.gridIndex[1]])
    counter = 0

    # Run loop while path is found or open set is empty
    while True:
        counter +=1
        # Check if openSet is empty, if empty no solution available
        if not openSet:
            print("没有可用路径！")
            return None

        # Get first node in the priority queue
        currentNodeIndex = costQueue.popitem()[0]
        currentNode = openSet[currentNodeIndex]

        # Revove currentNode from openSet and add it to closedSet
        openSet.pop(currentNodeIndex)
        closedSet[currentNodeIndex] = currentNode


        # Get Reed-Shepp Node if available
        rSNode = reedsSheppNode(currentNode, goalNode, mapParameters)

        # Id Reeds-Shepp Path is found exit
        if rSNode:
            print("找到了可直达终点的Reeds-Shepp路径！")
            closedSet[index(rSNode)] = rSNode
            break

        # USED ONLY WHEN WE DONT USE REEDS-SHEPP EXPANSION OR WHEN START = GOAL
        if currentNodeIndex == index(goalNode):
            print("Path Found")
            print(currentNode.traj[-1])
            break

        # Get all simulated Nodes from current node
        for i in range(len(motionCommand)):
            simulatedNode = kinematicSimulationNode(currentNode, motionCommand[i], mapParameters)

            # Check if path is within map bounds and is collision free
            if not simulatedNode:
                # print("当前节点没有可用的邻居节点")
                continue

            # Check if simulated node is already in closed set
            simulatedNodeIndex = index(simulatedNode)
            if simulatedNodeIndex not in closedSet: 

                # Check if simulated node is already in open set, if not add it open set as well as in priority queue
                if simulatedNodeIndex not in openSet:
                    openSet[simulatedNodeIndex] = simulatedNode
                    costQueue[simulatedNodeIndex] = max(simulatedNode.cost , Cost.hybridCost * holonomicHeuristics[simulatedNode.gridIndex[0]][simulatedNode.gridIndex[1]])
                else:
                    if simulatedNode.cost < openSet[simulatedNodeIndex].cost:
                        openSet[simulatedNodeIndex] = simulatedNode
                        costQueue[simulatedNodeIndex] = max(simulatedNode.cost , Cost.hybridCost * holonomicHeuristics[simulatedNode.gridIndex[0]][simulatedNode.gridIndex[1]])
    # Backtrack
    x, y, yaw = backtrack(startNode, goalNode, closedSet)

    return x, y, yaw

def drawCar(x, y, yaw, color='black'):
    # 车的左后方 右后方 右前方 左前方 左后方坐标点
    car = np.array([[-Car.axleToBack, -Car.axleToBack, Car.axleToFront, Car.axleToFront, -Car.axleToBack],
                    [Car.width / 2, -Car.width / 2, -Car.width / 2, Car.width / 2, Car.width / 2]])
    # 绕Z轴旋转矩阵
    rotationZ = np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])
    car = np.dot(rotationZ, car)
    car += np.array([[x], [y]])
    # 连接四个点绘制车体框架
    plt.plot(car[0, :], car[1, :], color)

def main():

    # Set Start, Goal x, y, theta
    # s = [15, 10, np.deg2rad(90)]
    # g = [45, 10, np.deg2rad(-90)]
    s = [10, 35, np.deg2rad(0)]
    g = [23, 28, np.deg2rad(0)]

    # Get Obstacle Map
    obstacleX, obstacleY = map()

    # Calculate map Paramaters
    mapParameters = calculateMapParameters(obstacleX, obstacleY, 4, np.deg2rad(5.0))

    # Run Hybrid A*
    x, y, yaw = run(s, g, mapParameters)
    
    # Draw Start, Goal Location Map and Path
    # plt.arrow(s[0], s[1], 1*math.cos(s[2]), 1*math.sin(s[2]), width=.1)
    # plt.arrow(g[0], g[1], 1*math.cos(g[2]), 1*math.sin(g[2]), width=.1)
    # plt.xlim(min(obstacleX), max(obstacleX)) 
    # plt.ylim(min(obstacleY), max(obstacleY))
    # plt.plot(obstacleX, obstacleY, "sk")
    # plt.plot(x, y, linewidth=2, color='r', zorder=0)
    # plt.title("Hybrid A*")


    # Draw Path, Map and Car Footprint
    plt.plot(x, y, linewidth=1.5, color='r', zorder=0)
    plt.plot(obstacleX, obstacleY, "sk")
    for k in np.arange(0, len(x), 2):
        plt.xlim(min(obstacleX), max(obstacleX)) 
        plt.ylim(min(obstacleY), max(obstacleY))
        drawCar(x[k], y[k], yaw[k])
        plt.arrow(x[k], y[k], 1*math.cos(yaw[k]), 1*math.sin(yaw[k]), width=.1)
        plt.title("Hybrid A*")

    # Draw Animated Car
    # for k in range(len(x)):
    #     plt.cla()
    #     plt.xlim(min(obstacleX), max(obstacleX)) 
    #     plt.ylim(min(obstacleY), max(obstacleY))
    #     plt.plot(obstacleX, obstacleY, "sk")
    #     plt.plot(x, y, linewidth=1.5, color='r', zorder=0)
    #     drawCar(x[k], y[k], yaw[k])
    #     # print(x[k], y[k], np.rad2deg(yaw[k]))
    #     plt.arrow(x[k], y[k], 1*math.cos(yaw[k]), 1*math.sin(yaw[k]), width=.1)
    #     plt.title("Hybrid A*")
    #     plt.pause(0.5)

    # k = 0
    # plt.cla()
    # plt.xlim(min(obstacleX), max(obstacleX)) 
    # plt.ylim(min(obstacleY), max(obstacleY))
    # plt.plot(obstacleX, obstacleY, "sk")
    # plt.plot(x, y, linewidth=1.5, color='r', zorder=0)
    # drawCar(x[k], y[k], yaw[k])
    # plt.arrow(x[k], y[k], 1*math.cos(yaw[k]), 1*math.sin(yaw[k]), width=.1)
    # plt.title("Hybrid A*")
    
    plt.show()

if __name__ == '__main__':
    main()
