import numpy as np
import math

# 加入了pnp算法得出的角度

n = 2
X_k = np.zeros(((3 * n), 1)) # 状态函数的全零矩阵，（0，0，0）
A = np.ones((n, n), dtype= np.int) #A每次调用卡尔曼滤波后都会变
X_k_position = np.zeros((3*n, 1))
# 计算观测函数Z_k
def calacZ_k(X_k_position): #position
    count = -1
    Z_k = np.zeros((2 * n * (n - 1), 1))
    for i in range(n):
        for j in range(n):
            if j == i:
                continue
            if(A[i,j]==1):
                temp1 = X_k_position[3 * i + 0, 0]  # 观测机器人x
                temp2 = X_k_position[3 * i + 1, 0]  # 观测机器人y
                temp3 = X_k_position[3 * i + 2, 0]  # 观测机器人角度i
                temp4 = X_k_position[3 * j + 0, 0]  # 被观测机器人x
                temp5 = X_k_position[3 * j + 1, 0]  # 被观测机器人y
                temp6 = X_k_position[3 * j + 2, 0]  # 被观测机器人角度j
                count = count+1
                Z_k[count, 0] = ((temp1 - temp4) ** 2 + (temp2 - temp5) ** 2)**0.5
                count = count+1
                Z_k[count, 0] = temp3-temp6      # z为pnp输出的两机器人朝向的夹角的值，无法得出单独的朝向角度
    count = count+1
    Z_k = np.delete(Z_k, np.s_[count:], axis=0)
    return Z_k

#旋转z(一步预测)
def robot_get1(X_old,z):
    for i in range(n):
        X_old[3*i+2,0]=X_old[3*i+2,0]+z[i,0]
    return  X_old

#直走D(一步预测)
def robot_get2(X_old,D):
    for i in range(n):
        X_k[3 * i + 0, 0] = X_old[3 * i + 0, 0] + D[i,0]*math.cos(X_old[3*i+2,0] * math.pi / 180)
        X_k[3 * i + 1, 0] = X_old[3 * i + 1, 0] + D[i,0]*math.sin(X_old[3*i+2,0] * math.pi / 180)
        X_k[3 * i + 2, 0] = X_old[3 * i + 2, 0]
    return  X_k



def calcH_k(X_k_position): #position
    count = -1
    H_k = np.zeros((2 * n * (n - 1), 3 * n))
    for i in range(n):
        for j in range(n):
            if j == i:
                continue
            if(A[i,j]==1):
                temp1 = X_k_position[3 * i + 0,0]  #观测机器人x
                temp2 = X_k_position[3 * i + 1,0]  # 观测机器人y
                #temp3 = X_k_position[3 * i + 2,0]   观测机器人角度i
                temp4 = X_k_position[3 * j + 0,0]  # 被观测机器人x
                temp5 = X_k_position[3 * j + 1,0]  # 被观测机器人y
                #temp6 = X_k_position[3 * j + 2,0]   被观测机器人角度j
                #temp7 = ((temp1 - temp4) ** 2 + (temp2 - temp5) ** 2)
                temp8 = ((temp1 - temp4) ** 2 + (temp2 - temp5) ** 2)**0.5
                #print(temp8)
                count = count + 1
                H_k[count,3 * i + 0] = (temp1-temp4)/temp8
                H_k[count,3 * i + 1] = (temp2-temp5)/temp8
                H_k[count,3 * i + 2] = 0
                H_k[count,3 * j + 0] = (temp4-temp1)/temp8
                H_k[count,3 * j + 1] = (temp5-temp2)/temp8
                H_k[count,3 * j + 2] = 0
                count=count+1
                H_k[count,3 * i + 0] =0
                H_k[count,3 * i + 1] =0
                H_k[count,3 * i + 2] = 1 #-1
                H_k[count,3 * j + 0] =0
                H_k[count,3 * j + 1] =0
                H_k[count,3 * j + 2] = -1 # 0
    count=count+1
    H_k=np.delete(H_k, np.s_[count:], axis=0)
    return H_k

def calcP_now(P_old, Q):  # 3n*3n + 3n*3n（Q是W协方差，恒不变）p0=0
    P_now = P_old + Q
    return P_now

#R是v的协方差 2n*2n      R_k一样，都是统一值
def calcK_k1(P_now, R_k,X_k, H_k):
    # X_k = robot_get1(X_old, z)#旋转
    # H_k = calcH_k(X_k)
    HT = np.transpose(H_k).tolist()  # 转置函数
    temp = np.dot(H_k, P_now)
    a = np.dot(temp, HT) + R_k
    #a = H_k * P_now * HT + R_k  # a,b是temp
    b = np.linalg.inv(a)  # 矩阵求逆
    K_k = np.dot(np.dot(P_now, HT), b)
    return K_k

def calcK_k2(P_now, R_k,X_k,H_k):
    # X_k = robot_get2(X_old,D)#直走
    # H_k = calcH_k(X_k)
    HT = np.transpose(H_k).tolist()  # 转置函数
    temp = np.dot(H_k, P_now)
    a = np.dot(temp, HT) + R_k
    #a = H_k * P_now * HT + R_k  # a,b是temp
    b = np.linalg.inv(a)  # 矩阵求逆
    K_k = np.dot(np.dot(P_now, HT), b)
    return K_k

#Z_k是已知输入，不在此代码中
def calc_X_k(X_k, K_k, Z_k):#xk是一步预测
    h = calacZ_k(X_k)
    Temp_1 = Z_k - h
    Temp_2 = np.dot(K_k, Temp_1)
    return X_k + Temp_2

def calcP_k(K_k, H_k, P_now):
    Temp = np.eye(3 * n)
    P_k = np.dot((Temp - np.dot(K_k, H_k)), P_now)
    return P_k



def final_xz(X_k_position_xz,P_old_xz,Q_xz,R_k,X_old_xz,z):
    #先旋转的卡尔曼滤波
    Z_k_xz=calacZ_k(X_k_position_xz) # simulate observation

    X_k_xz = robot_get1(X_old_xz, z)
    P_now_xz=calcP_now(P_old_xz, Q_xz)
    H_k_xz = calcH_k(X_k_xz)
    # K_k_xz=calcK_k1(P_now_xz, R_k, X_k_xz, z)
    K_k_xz = calcK_k1(P_now_xz, R_k, X_k_xz, H_k_xz)

    # H_k_xz=calcH_k(X_k_position_xz)
    final_Xk_xz=calc_X_k(X_k_xz,K_k_xz,Z_k_xz)
    P_k_xz=calcP_k(K_k_xz, H_k_xz, P_now_xz)

    return final_Xk_xz,P_k_xz

def final_zz(R_k,X_k_position_zz, P_old_zz, Q_zz, X_old_zz, D,final_Xk_xz):
    #后直走的卡尔曼滤波
    Z_k_zz = calacZ_k(X_k_position_zz)

    X_k_zz = robot_get2(final_Xk_xz, D)
    P_now_zz= calcP_now(P_old_zz, Q_zz)
    H_k_zz = calcH_k(X_k_zz)

    # K_k_zz = calcK_k2(P_now_zz, R_k, X_k_zz, D)
    K_k_zz = calcK_k2(P_now_zz, R_k, X_k_zz, H_k_zz)

    final_Xk_zz=calc_X_k(X_k_zz, K_k_zz, Z_k_zz)
    P_k_zz=calcP_k(K_k_zz, H_k_zz, P_now_zz)

    return final_Xk_zz,P_k_zz