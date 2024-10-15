# Loader Planner
![](./resource/1.gif)
![](./resource/2.gif)

## 1. Prerequisites
### 1.1 Ubuntu and ROS
Ubuntu >= 18.04  
ROS >= Melodic. [ROS Installation ](https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97)  
ROS-rviz version >=5

### 1.2 Eigen3, Ipopt, CasADi, Boost

install.sh
```bash
#!/bin/bash
#!/bin/bash
#Boost
sudo apt update
sudo apt install -y libboost-dev

#glog
sudo apt-get install -y libgoogle-glog-dev

#eigen3.4
cd ~/Downloads
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
unzip eigen-3.4.0.zip &&cd eigen-3.4.0
mkdir build
cd build
cmake ..
make
sudo make install

#ipopt
sudo apt-get install -y --allow-unauthenticated gfortran pkg-config coinor-libipopt-dev liblapack-dev libmetis-dev
cd ~/Downloads
mkdir Ipopt_pkg
cd Ipopt_pkg
git clone https://github.com/coin-or-tools/ThirdParty-ASL.git  #ASL
cd ThirdParty-ASL
./get.ASL
./configure
make
sudo make install
#https://github.com/coin-or-tools/ThirdParty-HSL.git  #HSL(no install)
cd ~/Downloads/Ipopt_pkg
git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git #Mumps
cd ThirdParty-Mumps
./get.Mumps
./configure
make
sudo make install
cd ~/Downloads/Ipopt_pkg #ipopt
git clone https://github.com/coin-or/Ipopt.git
cd Ipopt 
mkdir build
cd build
../configure
make 
sudo make install
cd /usr/local/include
sudo cp coin-or coin -r
sudo ln -s /usr/local/lib/libcoinmumps.so.3 /usr/lib/libcoinmumps.so.3
sudo ln -s /usr/local/lib/libcoinhsl.so.2 /usr/lib/libcoinhsl.so.2
sudo ln -s /usr/local/lib/libipopt.so.3 /usr/lib/libipopt.so.3

#CasADi
cd ~/Downloads
git clone --recursive https://github.com/casadi/casadi.git
cd casadi && mkdir build 
cd build 
cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DWITH_IPOPT=true
make
sudo make install
```

## 2. Build from source
Clone the respository, and build it. If the ros package cannot be found during compilation, use `sudo apt install -y ros-{your-ros}-{package}` to install it.
```bash
git clone https://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@github.com/Photin1a/LoaderPlanner.git
cd LoaderPlanner
catkin build
source devel/setup.bash
```
If you can't clone due to permissions, you can [download](https://github.com/Photin1a/LoaderPlanner) it manually.
## 3. Run
```bash
roslaunch simple_move_base simple_move_base.launch
```

## others
If you are using to develop, you can use `compile_commands.sh` to configure the path of the header file after completing the compilation.
```bash
chmod 777 ./compile_commands.sh && ./compile_commands.sh
```

## Reference
>[hybrid A*](https://github.com/zm0612/Hybrid_A_Star)  
>[decompROS](https://github.com/sikang/DecompROS.git)


---
# 更新日志
## Version-1
...
## Version-2
问题涉及到的约束有：  
x,y,theta,gamma：`走廊约束`   
gamma、omega、v、a、jerk的`边界约束`   
dt时间的`正则约束`dt>0，必须严格大于0   
### ~2024.5.13
使用f=e^x把走廊（x,y）不等式约束变变为代价添加到代价函数中   
求解时间在20m在500ms左右，长距离50m可能在1~3s不等甚至更多。总体不会超过4s   

### 2024.5.15
1) 将dt时间使用微分同胚映射到 (-inf,inf),从而消除`时间约束`  
2) 走廊Ax < b,使用f=e^x障碍函数添加到代价中，消除`走廊约束`  
3) gamma、omega、v、a、jerk,f=e^x障碍函数添加到代价中，消除`边界约束` 
==>求解时间50m在1~2s，一般50m以下求解时间最多一秒。20m在100ms~300ms之间，如果要求比较复杂求解时间可能更长。   

> 最后这是一个只有两点边界约束问题，IPOPT求解

## Version-dev 【维护中】
### 2024.xx.xx~
1) 解析推导梯度、Hessian
2) 改进罚函数和微分同胚变换
2) 代码优化
2) 求解
* LBFGS求解【测试优化中】
* casadi自动微分+ipopt求解
    ==>求解时间50m在300ms左右，20m在100ms以下，如果要求比较复杂求解时间可能略长。 

## 提上日程
1) 针对铰接模型设计粗解求解器  ==》 减轻后端优化的计算负载
2) 轨迹Minco曲线参数化  ==》曲线参数化加强求解稳定性、加速求解

