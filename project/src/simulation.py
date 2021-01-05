# repeat until simRemoteApi.start(19999,1300,false,true)~=-1
# %%
import math
import os

import time
import traceback
import matplotlib.pyplot as plt
import numpy as np
import scipy.ndimage as ndimage
import sim
from typing import *
NPArray = Any
#%%
clientID, leftWheel, rightWheel = None, None, None  # CoppeliaSim的用户ID以及左右轮句柄

mainSizeX, mainSizeY  = 80, 120  #^ 主视图大小
THRESHOLD = 127 #^二值化的阈值(像素值<127就当作黑色)
followLineX, followLineY = 480, 16 #^ 目标直线视图大小
bigX, bigY = 120, 240   #^ 大视图大小

baseVelocity = 3.0  # 基础前进速度
Integral = 0.0  # PID中积分项需要的误差累积
Kp, Ki, Kd = 0.02, 0.0, 0.0  # PID控制器的三个参数Kp/Ki/Kd
prevError = [0.0]  # 用于储存最近几次的误差，当作队列使用
bufferLen = 10
dt = 0.05
#%%
# 原图像480*640，左上角是(0,0)横向y纵向x，该函数用于裁剪原图像到指定大小(mainSizeX,mainSizeY)
# 裁剪框的位置在原图像的正下方中间
def crop(image, cropSizeX, cropSizeY):
    height, width = image.shape
    up = int(height - cropSizeX)
    down = int(height)
    left = int(width / 2 - cropSizeY / 2)
    right = int(width / 2 + cropSizeY / 2)

    cropImage = image[up:down, left:right]
    return cropImage

#%%
# 计算裁剪图像中黑色像素的平均位置
def calMeanPos(cropImage:NPArray)->Union[Tuple[float], Tuple[None]]:
    global THRESHOLD
    
    x_coordinates, y_coordinates = np.where(cropImage < THRESHOLD) #黑色0x00，白色0xff
    if len(x_coordinates)==0:
        #^ None值表示图像中没有检测到黑色像素点
        return None, None

    mean_x = np.mean(x_coordinates)
    mean_y = np.mean(y_coordinates)
    return mean_x, mean_y

def calMeanPosOfBlackLine(Line:NPArray)->Union[Tuple[float], Tuple[None]]:
    """取出目标线的黑色区域并计算其平均位置"""
    global THRESHOLD
    X, Y = Line.shape
    LOWER_CENTER_OF_LINE = (X - 1, Y // 2) #^ 目标直线视图中心位置
    if Line[LOWER_CENTER_OF_LINE] >= THRESHOLD:
        return None, None
    # 二值化，黑色像素标1，其他像素标0
    line_binary = np.where(Line < THRESHOLD, 1, 0)
    # 利用scipy中提供的标签方法，给每个不连通的黑色区域打上不同的标签
    line, num_area = ndimage.label(line_binary)
    # plt.imshow(line)
    # plt.show()
    value = line[LOWER_CENTER_OF_LINE]
    line = np.where(line==value, 0xff, 0x00) #^ 这里故意设反了，不知道为什么这样效果更好。
    return calMeanPos(line)

def fillCorner(image:NPArray)->NPArray:
    """填充图像角落的边缘黑色部分，实际测试显示暂时没啥用"""
    image_ = image.copy()
    X,Y = image.shape
    UPPER_RIGHT = (0, Y-1)
    UPPER_LEFT = (0, 0)
    LOWER_CENTER = (X - 1, Y // 2) #^ 视图中心位置
    # 二值化，黑色像素标1，其他像素标0
    image_binary = np.where(image < THRESHOLD, 1, 0)
    image_feats, num_area = ndimage.label(image_binary)

    upper_right_value = image_feats[UPPER_RIGHT]
    upper_left_value = image_feats[UPPER_LEFT]
    center_value = image_feats[LOWER_CENTER]

    #print(upper_left_value, upper_right_value, center_value)
    if upper_left_value!=0 and upper_left_value != center_value:
        image_[image_feats==upper_left_value] = 0xff #^填白色
    if upper_right_value!=0 and upper_right_value != center_value:
        image_[image_feats==upper_right_value] = 0xff #^填白色
    if False:
        plt.figure()
        plt.subplot(1,2,1)
        plt.imshow(image)
        plt.subplot(1,2,2)
        plt.imshow(image_)
        plt.show()
    return image_
#%%
def target_lost(mean_x:Any, mean_y:Any)->bool:
    """根据主视图x和y均值是否为空判断是否丢失目标"""
    return bool(mean_x==None and mean_y==None)

# 给小车赋予左右轮的速度
def motor(mean_x, mean_y, landscape_error, accelerationRate=1):
    global Integral, prevError, mainSizeX, mainSizeY, baseVelocity
    
    if target_lost(mean_x, mean_y): 
        # 若当前画面没有黑色像素，就根据最近几次的error决定当前向左旋转啊还是向右旋转
        #print(prevError)
        if np.mean(prevError) >= 0:
            leftVelocity = 0.67 * baseVelocity
            rightVelocity = -0.67 * baseVelocity
        else:
            leftVelocity = -0.67 * baseVelocity
            rightVelocity = 0.67 * baseVelocity
    else:
        # 分别计算PID的三个项（实际上只用到了一个P:)）
        velocity = baseVelocity * accelerationRate
        error = mean_y - mainSizeY / 2
        Integral += error
        differential = error - prevError[-1]
        adjustment = Kp * error + Ki * Integral + Kd * differential

        #^ 误差存储变成了存储大scale下的error，其余算法不变
        if landscape_error!=None:
            if len(prevError) >= bufferLen:
                del(prevError[0])
            prevError.append(landscape_error)
        #^ 优化打印的代码，现在看上去不是那么长了
        print(f'Err={error:.3f}, Adj={adjustment:.3f}')

        leftVelocity = velocity + adjustment
        rightVelocity = velocity - adjustment
    #^ 优化打印的代码，现在看上去不是那么长了
    print(f'[x{accelerationRate:.3f}] V_l={leftVelocity:.3f}, V_r={rightVelocity:.3f}')
    returnCode = sim.simxSetJointTargetVelocity(clientID, leftWheel, leftVelocity, sim.simx_opmode_oneshot)
    returnCode = sim.simxSetJointTargetVelocity(clientID, rightWheel, rightVelocity, sim.simx_opmode_oneshot)

    return leftVelocity, rightVelocity

#%%
if __name__ == "__main__":
    # initialize CoppeliaSim environment and get visionSensor handle
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim
    assert clientID!=-1, ConnectionError('Connection to server failed!')
    returnCode, visionSensorHandle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)

    # get wheel handle and initialize their parameters
    returnCode, leftWheel = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_blocking)
    returnCode, rightWheel = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_blocking)
    returnCode = sim.simxSetJointTargetVelocity(clientID, leftWheel, 0, sim.simx_opmode_oneshot)
    returnCode = sim.simxSetJointTargetVelocity(clientID, rightWheel, 0, sim.simx_opmode_oneshot)

    # start simulation and initialize vision sensor
    status = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
    #^ 改为不断查询直至获得图像
    returnCode = 1
    while returnCode != sim.simx_return_ok:
        returnCode, resolution, image = sim.simxGetVisionSensorImage(
            clientID, visionSensorHandle, 1, sim.simx_opmode_streaming)

    timeStart = time.time()
    last_time = sim.simxGetLastCmdTime(clientID)
    idx =0
    #^ 改为同步模式
    sim.simxSynchronous(clientID, 1)
    try:
        while True:
            returnCode, resolution, image = sim.simxGetVisionSensorImage(
                clientID, visionSensorHandle, 1, sim.simx_opmode_buffer)
            image = np.flipud(np.array(image, dtype=np.uint8).reshape([resolution[1], resolution[0]]))
            # if idx<=100:
            #     plt.imsave(f'full{idx}.jpg',np.stack([image]*3, axis=2))
            # idx+=1
            cropImage = crop(image, mainSizeX, mainSizeY)
            #^ 大scale就是原图像下半部分
            landscape = crop(image, bigX, bigY)
            #^ 增加一个图像followLine用于控制直线速度
            followLine = crop(image, followLineX, followLineY)

            mean_x, mean_y = calMeanPos(cropImage)
            #^ 计算大scale的平均黑色像素点位置，用于小车找不到方向时决定方向使用
            dir_x, dir_y = calMeanPos(landscape)
            #^ 增加一个函数calMeanPosOfBlackLine用于计算目标直线的平均黑点坐标
            acc_x, acc_y = calMeanPosOfBlackLine(followLine)
            # print(acc_x)
            #^ 加速速率设为对数增长
            accRate = max(math.log((followLineX - acc_x) / (followLineX / 5) + 1), 1) if acc_x else 1
            #^ 计算大尺度下的error
            landscape_error = dir_y - bigY / 2 if dir_y else None
            # print(landscape_error)
            leftVelocity, rightVelocity = motor(mean_x, mean_y, landscape_error, accRate)
            #^ 同步触发
            sim.simxSynchronousTrigger(clientID)

    #^ 暂且使用try-except来处理控制无限循环，具体逻辑懒得讲了，自己试试Ctrl+C是什么效果吧
    except KeyboardInterrupt:
        print('=============================================')
        print('Simulation stopped due to keyboard interrupt.')
        print('=============================================')
    except Exception as e:
        traceback.print_exc()
    # stop simulation and cleanup
    status = sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
#%%