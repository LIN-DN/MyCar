#!/usr/bin/env python3
'''
    障碍地图绘制
'''

def map():
    # Build Map
    obstacleX, obstacleY = [], []

    # # UAV MAP
    # for i in range(20*12):
    #     for j in range(12):
    #         obstacleX.append(i)
    #         obstacleY.append(0*12+j)

    # for i in range(9*12):
    #     for j in range(12):
    #         obstacleX.append(0*12+j)
    #         obstacleY.append(i)

    # for i in range(20*12):
    #     for j in range(12):
    #         obstacleX.append(i)
    #         obstacleY.append(8*12+j)

    # for i in range(9*12):
    #     for j in range(12):
    #         obstacleX.append(19*12+j)
    #         obstacleY.append(i)

    # for i in range(4*12,6*12):
    #     for j in range(12):
    #         obstacleX.append(5*12+j)
    #         obstacleY.append(i)

    # for i in range(1*12,2*12):
    #     for j in range(12):
    #         obstacleX.append(7*12+j)
    #         obstacleY.append(i)

    # for i in range(9*12,11*12):
    #     for j in range(12):
    #         obstacleX.append(i)
    #         obstacleY.append(3*12+j)

    # for i in range(9*12,11*12):
    #     for j in range(12):
    #         obstacleX.append(i)
    #         obstacleY.append(6*12+j)

    # for i in range(2*12,4*12):
    #     for j in range(12):
    #         obstacleX.append(15*12+j)
    #         obstacleY.append(i)

    # for i in range(6*12,7*12):
    #     for j in range(12):
    #         obstacleX.append(13*12+j)
    #         obstacleY.append(i)
    
    # for i in range(20*12):
    #     for j in range(12):
    #         obstacleX.append(i)
    #         obstacleY.append(0*12+j)

    # UWB 区域
    for i in range(7*12):
        obstacleX.append(i)
        obstacleY.append(0)

    for i in range(8*12):
        obstacleX.append(0)
        obstacleY.append(i)

    for i in range(7*12+1):
        obstacleX.append(i)
        obstacleY.append(8*12)

    for i in range(8*12+1):
        obstacleX.append(7*12)
        obstacleY.append(i)

    return obstacleX, obstacleY
