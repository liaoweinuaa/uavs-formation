
import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd

N=20
sigma=0.5
#target_pos=[[5+4*np.cos(2*np.pi/20*i),5+4*np.sin(2*np.pi/20*i)] for i in range(N)]

target_pos=[[5+2*np.cos(0.5*np.pi+0.1*i*np.pi),6+2*np.sin(0.5*np.pi+0.1*i*np.pi)] for i in range(1,11)]+\
[[5+2*np.cos(-0.5*np.pi+0.1*i*np.pi),2+2*np.sin(-0.5*np.pi+0.1*i*np.pi)] for i in range(10)]

class UAV:
    def __init__(self):
        self.pos_x=np.random.uniform(0,10)
        self.pos_y=np.random.uniform(0,10)
        self.sigma=sigma


    def move(self,dx,dy):
        # l=math.sqrt(dx**2+dy**2)
        # dx=0.01*dx/l
        # dy=0.01*dy/l
        self.pos_x+=dx
        self.pos_y+=dy


    def probability(self,x,y):
        t_1=(-(x-self.pos_x)**2)/(self.sigma**2)
        t_2=(-(y-self.pos_y)**2)/(self.sigma**2)
        p=np.power(np.e,0.5*(t_1+t_2))/(2*np.pi*self.sigma**2)
        return p

    def dp_dmu(self,x,y):
        t_1=(-(x-self.pos_x)**2)/(self.sigma**2)
        t_2=(-(y-self.pos_y)**2)/(self.sigma**2)
        t_3=x-self.pos_x
        t_4=y-self.pos_y
        dp_dmu1=(np.power(np.e,0.5*(t_1+t_2))*t_3)/(2*np.pi*self.sigma**4)
        dp_dmu2=(np.power(np.e,0.5*(t_1+t_2))*t_4)/(2*np.pi*self.sigma**4)
        return dp_dmu1,dp_dmu2

class UAV_GROUP:
    def __init__(self,n,r):
        self.UAV_lst=[UAV() for i in range(n)]
        self.num_UAVs=len(self.UAV_lst)
        self.target_pos=target_pos
        self.sigma=self.UAV_lst[0].sigma
        self.samples=None
        self.r=r
        self.X_tra_data=[]
        self.Y_tra_data=[]


    def sample(self,n):
        res=[]
        for i in range(n):
            label=np.random.randint(0,self.num_UAVs)
            res.append(np.random.multivariate_normal(target_pos[label],[[self.sigma**2,0],[0,self.sigma**2]]))
        self.samples=np.array(res)

    def _sum_porb(self,x,y):
        s=0
        for uav in self.UAV_lst:
            p=uav.probability(x,y)
            s=s+p
        return s

    def _single_move(self,n):
        uav=self.UAV_lst[n]
        dx,dy=0,0
        for x,y in self.samples:
            dp_dmu1,dp_dmu2=uav.dp_dmu(x,y)
            s=self._sum_porb(x,y)
            dx+=(1/s)*dp_dmu1
            dy+=(1/s)*dp_dmu2
        l=np.sqrt( dx*dx+dy*dy )
        # dx=10*dx/len(self.samples)
        # dy=10*dy/len(self.samples)
        dx=0.02*dx/l
        dy=0.02*dy/l
        return dx,dy

    def move_all(self):
        move_lst=[]
        for n in range(self.num_UAVs):
            movation=self._single_move(n)
            move_lst.append(movation)

        for m in range(self.num_UAVs):
            self.UAV_lst[m].move(self.r*move_lst[m][0],self.r*move_lst[m][1])
            # self.UAV_lst[m].determinize()

        self.sigma=self.UAV_lst[0].sigma

    def show_pos(self):
        X=[uav.pos_x for uav in self.UAV_lst]
        Y=[uav.pos_y for uav in self.UAV_lst]
        #print(X,'\n',Y)
        # plt.scatter(X,Y)
        # plt.xlim((0,10))
        # plt.ylim((0,10))
        # plt.pause(0.01)
        self.X_tra_data.append(X)
        self.Y_tra_data.append(Y)

    # def show_pic(self):
    #     X=[uav.pos_x for uav in self.UAV_lst]
    #     Y=[uav.pos_y for uav in self.UAV_lst]
    #
    #     plt.scatter(X,Y)
    #     plt.pause(0.01)

uav_group=UAV_GROUP(20,1)
uav_group.show_pos()
# uav_group.sample(1024)
# for x,y in uav_group.samples:
#     print(x,y)

for i in range(800):
    print(i)
    uav_group.sample(2000)
    uav_group.move_all()
    uav_group.show_pos()
    # if i%20==0:
    #     uav_group.show_pic()
    # if (i+1)%100==0:
    #     uav_group.show_pos()

X_csv=pd.DataFrame(uav_group.X_tra_data)
Y_csv=pd.DataFrame(uav_group.Y_tra_data)

X_csv.to_csv('X_csv.csv')
Y_csv.to_csv('Y_csv.csv')
















































