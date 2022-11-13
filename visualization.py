import matplotlib.pyplot as plt
#afterEKF
x_a = [0,0]    #x坐标数组，n个
y_a = [0,1]      #y坐标数组，n个
z_a = [0,0]      #角度数组，n个
#beforeEKF
x_b = [0,0]      #x坐标数组，n个
y_b = [0,1]      #y坐标数组，n个
z_b = [0,0]      #角度数组，n个
#true
x_t = []      #x坐标数组，n个
y_t = []      #y坐标数组，n个
z_t = []      #角度数组，n个

def put(IP_LIST,juzhen,l):       #juzhen为3n*1的矩阵，m为图像的marker,c为图像的color，l为图像的label
    num=len(IP_LIST) #机器人个数
    if l=='AfterEKF':
        i=0
        global x_a
        global y_a
        global z_a
        while i< num:   #i<机器人个数
            x_a.append(juzhen[3*i,0])
            y_a.append(juzhen[3*i+1,0])
            z_a.append(juzhen[3*i+2,0])
            i+=1

    if l=='BeforeEKF':
        i=0
        global x_b
        global y_b
        global z_b
        while i< num:   #i<机器人个数
            x_b.append(juzhen[3*i,0])
            y_b.append(juzhen[3*i+1,0])
            z_b.append(juzhen[3*i+2,0])
            i+=1


    if l=='True':
        i=0
        global x_t
        global y_t
        global z_t
        while i< num:   #i<机器人个数
            x_t.append(juzhen[3*i,0])
            y_t.append(juzhen[3*i+1,0])
            z_t.append(juzhen[3*i+2,0])
            i+=1

def showall(IP_LIST):
    global x_a
    global y_a
    global z_a
    global x_b
    global y_b
    global z_b
    global x_t
    global y_t
    global z_t
    j=0
    num=len(IP_LIST)
    while j < num:  # j<机器人个数
        plt.plot(x_a[j:len(x_a) - num + j+1:num], y_a[j:len(y_a) - num + j+1:num], marker='<', color='b',
                 label='AfterEKF' + 'robot' + str(j + 1))
        plt.plot(x_b[j:len(x_b) - num + j+1:num], y_b[j:len(y_b) - num + j+1:num], marker='o', color='r',
                 label='BeforeEKF' + 'robot' + str(j + 1))
        # plt.plot(x_t[j:len(x_t) - num + j+1:num], y_t[j:len(y_t) - num + j+1:num], marker='<', color='g',
        #          label='True' + 'robot' + str(j + 1))
        j += 1
    plt.legend()
    plt.xlim([-10, 10])
    plt.ylim([-10, 10])
    plt.xlabel('x/m')
    plt.ylabel('y/m')
    # for i in range(len(x)):
    #     plt.annotate(round(z[i],2),xy=(x[i],y[i]),xytext=(x[i],y[i]))    #数字化显示机器人朝向角度
    plt.show()

