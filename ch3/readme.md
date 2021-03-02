# 主要有三个文件：

1,vio_data_simulation-master

在该程序中，将IMU仿真代码中的欧拉积分替换成中值积分。替换为中值积分后，精度应该有所提升。

2,vio_data_simulation-ros_version

用于生成静态的IMU数据，imu.bag格式。

3,使用allan方差工具Kalibr_allan-master工具标定生成Allen方差标定曲线

# 利用2生成的imu.bag进行阿伦方差分析
（1）利用bagconvert转换成mat形式

（2）使用脚本对mat文件进行分析（耗时）

（3）使用脚本画allan方差曲线
