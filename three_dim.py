
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd
fig = plt.figure()
# ax = Axes3D(fig)
N=20
sigma=0.45


target_pos=[[5+2*np.cos(2*np.pi*i/20),5+2*np.sin(2*np.pi*i/20),0.1*i] for i in range(20)]

class UAV:
    def __init__(self):
        self.pos_x=np.random.uniform(0,10)
        self.pos_y=np.random.uniform(0,10)
        self.pos_z=np.random.uniform(0,5)
        self.sigma=sigma


    def move(self,dx,dy,dz):
        # l=math.sqrt(dx**2+dy**2)
        # dx=0.01*dx/l
        # dy=0.01*dy/l
        self.pos_x+=dx
        self.pos_y+=dy
        self.pos_z+=dz

    def probability(self,x,y,z):
        t_1=(-(x-self.pos_x)**2)/(self.sigma**2)
        t_2=(-(y-self.pos_y)**2)/(self.sigma**2)
        t_3=(-(z-self.pos_z)**2)/(self.sigma**2)
        p=np.power(np.e,0.5*(t_1+t_2+t_3))/(15.8*self.sigma**3)
        return p

    def dp_dmu(self,x,y,z):
        t_1=(-(x-self.pos_x)**2)/(self.sigma**2)
        t_2=(-(y-self.pos_y)**2)/(self.sigma**2)
        t_5=(-(z-self.pos_z)**2)/(self.sigma**2)
        t_3=x-self.pos_x
        t_4=y-self.pos_y
        t_6=z-self.pos_z
        dp_dmu1=(np.power(np.e,0.5*(t_1+t_2+t_5))*t_3)/(2*np.pi*self.sigma**4)
        dp_dmu2=(np.power(np.e,0.5*(t_1+t_2+t_5))*t_4)/(2*np.pi*self.sigma**4)
        dp_dmu3=(np.power(np.e,0.5*(t_1+t_2+t_5))*t_6)/(2*np.pi*self.sigma**4)
        return dp_dmu1,dp_dmu2,dp_dmu3

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
        self.Z_tra_data=[]


    def sample(self,n):
        res=[]
        for i in range(n):
            label=np.random.randint(0,self.num_UAVs)
            res.append(np.random.multivariate_normal(target_pos[label],[[self.sigma**2,0,0],[0,self.sigma**2,0],[0,0,self.sigma**2]]))
        self.samples=np.array(res)

    def _sum_porb(self,x,y,z):
        s=0
        for uav in self.UAV_lst:
            p=uav.probability(x,y,z)
            s=s+p
        return s

    def _single_move(self,n):
        uav=self.UAV_lst[n]
        dx,dy,dz=0,0,0
        for x,y,z in self.samples:
            dp_dmu1,dp_dmu2,dp_dmu3=uav.dp_dmu(x,y,z)
            s=self._sum_porb(x,y,z)
            dx+=(1/s)*dp_dmu1
            dy+=(1/s)*dp_dmu2
            dz+=(1/s)*dp_dmu3
        l=np.sqrt(dx*dx+dy*dy+dz*dz)
        dx=0.02*dx/l
        dy=0.02*dy/l
        dz=0.02*dz/l
        return dx,dy,dz

    def move_all(self):
        move_lst=[]
        for n in range(self.num_UAVs):
            movation=self._single_move(n)
            move_lst.append(movation)

        for m in range(self.num_UAVs):
            self.UAV_lst[m].move(self.r*move_lst[m][0],self.r*move_lst[m][1],self.r*move_lst[m][2])


        self.sigma=self.UAV_lst[0].sigma

    def show_pos(self):
        X=[uav.pos_x for uav in self.UAV_lst]
        Y=[uav.pos_y for uav in self.UAV_lst]
        Z=[uav.pos_z for uav in self.UAV_lst]
        #print(X,'\n',Y)
        # plt.scatter(X,Y)
        # plt.pause(0.01)
        self.X_tra_data.append(X)
        self.Y_tra_data.append(Y)
        self.Z_tra_data.append(Z)
    # def show_pic(self):
    #     X=[uav.pos_x for uav in self.UAV_lst]
    #     Y=[uav.pos_y for uav in self.UAV_lst]
    #     plt.clf()
    #     plt.scatter(X,Y)
    #     plt.pause(0.01)


uav_group=UAV_GROUP(20,1)
uav_group.show_pos()
# uav_group.sample(1024)
# for x,y in uav_group.samples:
#     print(x,y)

for i in range(1500):
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
Z_csv=pd.DataFrame(uav_group.Z_tra_data)

X_csv.to_csv('X_csv3d.csv')
Y_csv.to_csv('Y_csv3d.csv')
Z_csv.to_csv('Z_csv3d.csv')















































