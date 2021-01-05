import cv2

STRIDE = 100 # 控制检查直线是否穿过障碍物x,y的步长
EPS = 3 # 控制检查是否为终点时容忍的像素差

class Map(object):
    def __init__(self, path, init=None, goal=None):
        super().__init__()
        
        # 读取图片
        img = cv2.imread(path)
        # 转换成灰度图
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 灰度图二值化（0或255）
        ret, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        self.img = img
        self.mat = thresh
        self.init = init if init is not None else (0, 0)
        self.goal = goal if goal is not None else (self.mat.shape[0]-1, self.mat.shape[1]-1)
    
    def check_collision(self, pt1, pt2):
        # 检查两个点之间是否碰撞
        if self.mat[pt1[0]][pt1[1]] == 0 or self.mat[pt2[0]][pt2[1]] == 0: return False

        stepx = (pt2[0] - pt1[0]) / STRIDE
        stepy = (pt2[1] - pt1[1]) / STRIDE

        nowx, nowy = pt1[0], pt1[1]
        for i in range(STRIDE):
            nowx += stepx
            nowy += stepy

            if self.mat[round(nowx)][round(nowy)] == 0:
                return False
        return True
    
    def is_goal(self, pt):
        # 检查给定点是否到达终点
        diffx, diffy = pt[0] - self.goal[0], pt[1] - self.goal[1]
        diff = (diffx ** 2 + diffy ** 2) ** 0.5
        if diff < EPS: return True
        return False

    def draw_dot(self, pt, color=(255,0,0), thickness=1):
        # 在给定的点处画一个点
        self.img = cv2.circle(self.img, (pt[1], pt[0]), EPS, color, thickness=thickness)

    def draw_line(self, pt1, pt2, color=(255,0,0), thickness=1):
        # 在给定两点之间画一条直线
        self.img = cv2.line(self.img, (pt1[1], pt1[0]), (pt2[1], pt2[0]), color, thickness=thickness)
    
    def save_fig(self, path="result.jpg"):
        # 保存当前的地图
        cv2.imwrite(path, self.img)
