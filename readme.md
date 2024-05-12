# Loader Planner

## 1. Prerequisites
### 1.1 Ubuntu and ROS
Ubuntu >= 18.04  
ROS >= Melodic. [ROS Installation ](https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97)  
ROS-rviz version >=5

### 1.2 Eigen3, Ipopt, CasADi, Boost
```bash
#Boost
sudo apt update
sudo apt install -y libboost-dev
```

```bash
#eigen3.4
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
unzip eigen-3.4.0.zip &&cd eigen-3.4.0
mkdir build
cd build
cmake ..
make
sudo make install
```

```bash
#ipopt
sudo apt-get install -y gfortran pkg-config coinor-libipopt-dev
git clone https://github.com/coin-or/Ipopt.git
cd Ipopt && mkdir build
cd build
./configure
make 
sudo make install
```

```bash
#CasADi
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
mkdir loader_ws 
cd loader_ws
git clone https://github.com/Photin1a/LoaderPlanner.git
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