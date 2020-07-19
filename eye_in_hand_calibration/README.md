### This is an eye in hand calibration project to calibrate a Robot and a camera.

### This is adjusted from LI KAI's project.

### Below is the original README.md file.

### To be continued...

```
sudo apt install ros-kinetic-universal-robot
```

image collector jaka is for other robots. robot.txt is joint angle, read using infilestream(same as ur), move using move joint joint angle(differ from ur). controlle using '9'
更改机器人消息为ur -->> jaka_pose
由于parameter.yml中使用了相对路径原因，需要在本仓库根目录运行代码，否则读不到机器人关节数据。
 遇见msg没有转化为头文件，只需要重复编译几次，即可。
# 机器人手眼标定

#### Author：LI KAI

#### Date： 2017.11.14整理

本程序用于UR机器人手眼标定、结果验证。
### 1、平台、框架与第三方依赖
#### opencv
建议使用opencv2.4.8及以上版本
#### ROS
ROS indigo(ubuntu 14.04)
### 2、构建与运行方法
#### 构建
clone本项目至ros catkin_ws/src目录下，catkin_make即可。
#### 运行
将标定图片保存至src/parameters/image文件夹内，设置好parameters内的各个参数文件。
保存标定图像时，先开启相机节点，广播彩色图topic，随后运行 ur_controller.py  image_collector，按2移动机器人至    
指定位置，按t保存图片。机器人位置、彩色图topic等参数，存储在parameters/parameter.yml内。
~~~shell
cd src/eye_hand_calibration/src
python ur_controller.py2
rosrun eye_hand_calibration image_collector parameters/parameter.yml
~~~
输入参数如下：<br />
**imageList.xml**    &nbsp;&nbsp;  保存标定图片名称<br />
**robot_pose.txt**   &nbsp;&nbsp;  保存机器人位姿，p(x,y,z,rx,ry,rz)，xyz->单位m，(rx,ry,rz)->旋转矢量<br />
**parameters.yml**   &nbsp;&nbsp;  各类输入参数<br />
命令行目录移至改项目src文件夹下
~~~shell
cd src/eye_hand_calibration/src
rosrun eye_hand_calibration calibration parameters/parameter.yml
~~~
读取标定图片时，在图像上，单击鼠标左键选择标定板坐标系原点，右键确认。<br />
如遇到提取角点失败情形，单击鼠标右键跳过。

标定输出参数如下：<br />
**out_camera_data.yml**    &nbsp;&nbsp;  相机内外参数<br />
**EyeHandParameter.yml**   &nbsp;&nbsp;  手眼矩阵标定结果。p(x,y,z,rx,ry,rz)，xyz->单位mm，(rx,ry,rz)->旋转矢量<br />
**VarifyPose.yml**   &nbsp;&nbsp;  根据输入的相机验证位姿，求得的机器人位姿。验证程序据此控制机器人移动，位姿定义同机器人位姿定义<br />

#### 手眼标定结果验证
开启相机节点，广播彩色图topic。<br />
开启ur_controller，读取机器人数据，等待控制指令。
~~~shell
cd src/eye_hand_calibration/src
python ur_controller.py
~~~
~~~shell
cd src/eye_hand_calibration/src
rosrun eye_hand_calibration cross parameters/parameter.yml
~~~
验证程序开启后，在图像窗口下按空格，机器人会按参数定义移动。观察画面准星，判断标定效果。

