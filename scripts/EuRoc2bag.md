[TOC]

# EuRoc2bag

EuRoc数据集的图像和惯性数据做成.bag形式文件，需要采集图像后再利用bagcreater.py工具去转化成.bag包。

## 1.数据输出

数据输出格式包括

```shell
- mav0
 - cam0
  - data dir
  - data.csv
  - sensor.yaml
 - cam1
  - data dir
  - data.csv
  - sensor.yaml
 - imu0
  - data.csv
  - sensor.yaml
 - state_groundtruth_estimate0
  - data.csv
  - sensor.yaml
 - body.yaml
```

groundtruth输出格式为：

18位时间戳timestamp

p代表position，指的是MAV的空间3D坐标，RS代表这个坐标是在R坐标系的值，也就是LEICA位姿跟踪系统坐标系下测到的值，S指的是原来的值是从Sensor坐标系下得到的，后来又变换到了R坐标系。R代表LEICA坐标系，x代表这是3D位置的x轴方向上的真值，单位是米。

```shell
p_RS_R_x [m] p_RS_R_y [m] p_RS_R_z [m]
```

q代表quaternion四元数，表达了MAV的朝向信息，RS代表是在R坐标系下测到的朝向信息，实际上最开始是在Sensor坐标系下的朝向，后来被变换到了R坐标系下，w为四元数的实部，xyz为虚部。

```shell
q_RS_w [] q_RS_x [] q_RS_y [] q_RS_z []
```

v代表这是MAV的速度信息，而且是在R坐标系下的速度信息，单位m/s。

```shell
v_RS_R_x [m s^-1] v_RS_R_y [m s^-1] v_RS_R_z [m s^-1]
```

w代表这是MAV在R坐标系下的角速度信息，单位rad/s。

```shell
b_w_RS_S_x [rad s^-1] b_w_RS_S_y [rad s^-1] b_w_RS_S_z [rad s^-1]
```

a代表这是MAV在R坐标系下的线加速度信息，单位m/s^2。

```shell
b_a_RS_S_x [m s^-2] b_a_RS_S_y [m s^-2] b_a_RS_S_z [m s^-2]
```

## 2.bagcreater打包方法

```python
#!/usr/bin/env python
print "importing libraries"
 
import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
#import ImageFile
#import PIL.ImageFile as ImageFile
import time, sys, os
import argparse
import cv2
import numpy as np
import csv
 
#structure
# dataset/cam0/data/TIMESTAMP.png
# dataset/camN/data/TIMESTAMP.png
# dataset/imu0/data.csv
# dataset/imuN/data.csv
# dataset/leica0/data.csv
 
#setup the argument list
parser = argparse.ArgumentParser(description='Create a ROS bag using the images and imu data.')
parser.add_argument('--folder', metavar='folder', nargs='?', help='Data folder')
parser.add_argument('--output-bag', metavar='output_bag', default="output.bag", help='ROS bag file %(default)s')
 
#print help if no argument is specified
if len(sys.argv)<2:
 parser.print_help()
 sys.exit(0)
 
#parse the args
parsed = parser.parse_args()
 
def getImageFilesFromDir(dir):
 '''Generates a list of files from the directory'''
 image_files = list()
 timestamps = list()
 if os.path.exists(dir):
 for path, names, files in os.walk(dir):
 for f in files:
 if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
 image_files.append( os.path.join( path, f ) )
 timestamps.append(os.path.splitext(f)[0]) 
 #sort by timestamp
 sort_list = sorted(zip(timestamps, image_files))
 image_files = [file[1] for file in sort_list]
 return image_files
 
def getCamFoldersFromDir(dir):
 '''Generates a list of all folders that start with cam e.g. cam0'''
 cam_folders = list()
 if os.path.exists(dir):
 for path, folders, files in os.walk(dir):
  for folder in folders: 
 if folder[0:3] == "cam":
 cam_folders.append(folder)
 return cam_folders
 
def getImuFoldersFromDir(dir):
 '''Generates a list of all folders that start with imu e.g. cam0'''
 imu_folders = list()
 if os.path.exists(dir):
 for path, folders, files in os.walk(dir):
 for folder in folders: 
 if folder[0:3] == "imu":
 imu_folders.append(folder)
 return imu_folders
 
def getImuCsvFiles(dir):
 '''Generates a list of all csv files that start with imu'''
 imu_files = list()
 if os.path.exists(dir):
 for path, folders, files in os.walk(dir):
 for file in files:
  if file[0:3] == 'imu' and os.path.splitext(file)[1] == ".csv":
 imu_files.append( os.path.join( path, file ) )
 
 return imu_files
 
def loadImageToRosMsg(filename):
 image_np = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
  
 timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
 timestamp = rospy.Time( secs=int(timestamp_nsecs[0:-9]), nsecs=int(timestamp_nsecs[-9:]) )
 
 rosimage = Image()
 rosimage.header.stamp = timestamp
 rosimage.height = image_np.shape[0]
 rosimage.width = image_np.shape[1]
 rosimage.step = rosimage.width #only with mono8! (step = width * byteperpixel * numChannels)
 rosimage.encoding = "mono8"
 rosimage.data = image_np.tostring()
 
 return rosimage, timestamp
 
def createImuMessge(timestamp_int, omega, alpha):
 timestamp_nsecs = str(timestamp_int)
 timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
 
 rosimu = Imu()
 rosimu.header.stamp = timestamp
 rosimu.angular_velocity.x = float(omega[0])
 rosimu.angular_velocity.y = float(omega[1])
 rosimu.angular_velocity.z = float(omega[2])
 rosimu.linear_acceleration.x = float(alpha[0])
 rosimu.linear_acceleration.y = float(alpha[1])
 rosimu.linear_acceleration.z = float(alpha[2])
 
 return rosimu, timestamp
 
#create the bag
try:
 bag = rosbag.Bag(parsed.output_bag, 'w')
 
 #write images
 camfolders = getCamFoldersFromDir(parsed.folder)
 for camfolder in camfolders:
 camdir = parsed.folder + "/{0}".format(camfolder) + "/data"
 image_files = getImageFilesFromDir(camdir)
 for image_filename in image_files:
 image_msg, timestamp = loadImageToRosMsg(image_filename)
 bag.write("/{0}/image_raw".format(camfolder), image_msg, timestamp)
 
 #write imu data
 imufolders = getImuFoldersFromDir(parsed.folder)
 for imufolder in imufolders:
 imufile = parsed.folder + "/" + imufolder + "/data.csv"
 topic = os.path.splitext(os.path.basename(imufolder))[0]
 with open(imufile, 'rb') as csvfile:
 reader = csv.reader(csvfile, delimiter=',')
 headers = next(reader, None)
 for row in reader:
 imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
 bag.write("/{0}".format(topic), imumsg, timestamp)
 
finally:
 bag.close()
```

EuRoc的數據包的打包方法：在包含mav文件夾和上述腳本文件的文件夾內，運行上述腳本：

```shell
python bagcreater.py --folder mav --output-bag mav.bag
```

如果遇到L、uint32字样的错误，可能是由于csv时间戳中出现空格导致的。

可以得到bag文件，但是这个方法会出现某些错误，例如文件目录找不到，或强制要求EuRoc数据集的LEICA数据，因此改用其原版kalibr程序进行打包。

## 3.Kalibr打包方法

首先需要安装依赖项：

```shell
sudo apt-get install python-igraph
```

新建kalibr和其中的src文件夹

在kalibr里

```shell
source/opt/ros/kinetic/setup.bash
```

安装和初始化catkin-tools

```shell
#具體ros版本根據實際變化
sudo apt-get install python-catkin-tools
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

然后下载Kalibr

```shell
cd~/kalibr_workspace/src
git clone [https://github.com/ethz-asl/Kalibr.git]
cd~/kalibr_workspace
```

开线程进行编译

```shell
catkin build -DCMAKE_BUILD_TYPE=Release -j2
```

途中遇到出错没有libv4l2，可以安装libv4l-dev。

关于catkin_make过程中下载suitesparse过久甚至失败的问题：

假设已经下载好kalibr包到~/catkin_ws/src中，在github中下载相应版本https://github.com/jluttine/suitesparse/releases (假设下载suitesparse-4.2.1.tar.gz)；

修改～/catkin_ws/src/kalibr/suitesparse中的CMakeLists.txt；

注释掉下载的命令、tar -xzf后改为保存上述下载suitesparse的路径、将原文件名SuiteSparse改为suitesparse-4.2.1，如下所示：

```shell
DOWNLOAD_COMMAND rm -f SuiteSparse-${VERSION}.tar.gz #&& wget https://github.com/jluttine/suitesparse/archive/v4.2.1.tar.gz
PATCH_COMMAND tar -xzf /home/lucious/Downloads/suitesparse-${VERSION}.tar.gz && rm -rf ../suitesparse_src-build/SuiteSparse && sed -i.bu "s/\\/usr\\/local\\/lib/..\\/lib/g" suitesparse-4.2.1/SuiteSparse_config/SuiteSparse_config.mk && sed -i.bu "s/\\/usr\\/local\\/include/..\\/include/g" suitesparse-4.2.1/SuiteSparse_config/SuiteSparse_config.mk && mv suitesparse-4.2.1 ../suitesparse_src-build/
 CONFIGURE_COMMAND ""
 BUILD_COMMAND cd suitesparse-4.2.1 && make library -j8 -l8
 INSTALL_COMMAND cd suitesparse-4.2.1 && mkdir -p lib 
```

最后重新在~/catkin_ws中重新执行catkin_make即可。

```shell
source~/kalibr_workspace/devel/setup.bash
```

打包方法为：

```shell
kalibr_bagcreater--folder dataset-dir --output-bag 1.bag
```

