## 作业：

![image-20210508153616897](../images/image-20210508153616897.png)

![image-20210508154754881](../images/image-20210508154754881.png)

![image-20210508154836213](../images/image-20210508154836213.png)

## *1，VIO文献阅读：*

视觉与IMU融合之后有何优势：

​         通过将视觉与IMU组合，构成一个VIO，即惯性视觉里程计。通常，视觉里程计利用相机可以有效采集周围图像信息，并根据图像中的特征进行位姿估计，但是如果载体运动较快，会由于提到的特征点数量较少，导致定位精度较低。而IMU作为传统定位传感器，频率较高，短期运动预测精度较高，可以弥补纯视觉运动估计中存在的运动模糊以及特征缺失等不足。对于长时间运动，纯IMU的惯导解算会累积误差，而视觉又可以有效的估计其位姿参数，修正IMU测量数据中的漂移误差。两者有效结合，可以实现优势互补。有哪些常见的视觉+IMU融合方案？有没有工业界应用的例子：

​          VIO框架主要分为两大类：松耦合以及紧耦合。松耦合，即IMU和相机分别进行自身的运动估计，再对其位姿估计结果进行融合。紧耦合，即构建一个包含IMU状态以及相机状态的状态方程，共同构建运动方程以及观测方程，进行状态估计。目前主要的开源算法都是紧耦合方案，并分为滤波以及优化两种。滤波最具有代表性的就是MSCKF，如Google Project Tango中就有运用。优化具有代表性的就是VINS-MONO，大疆无人机也有运用。在学术界，VIO研究有哪些新进展，有没有将学习方法运用到VIO中的例子：

​          VIO新进展，主要是有视觉引入了语义信息，在大部分场景下有更强的鲁棒性，速度更快。可以称为真正的SLAM，定位建图一条龙。此外，还有引入GPU前端加速。另外，多传感器融合也是一大方案，适合于更多场景，并在更长时间运行条件下保持足够定位精度。学习方法运用到VIO例子：半监督，无监督学习的VIO。

## *2，四元数与李代数更新：*

运行结果：

![image-20210508154909697](../images/image-20210508154909697.png)

## *3，其他导数：*

(1)

![image-20210508154931609](../images/image-20210508154931609.png)

(2)

![image-20210508155011790](../images/image-20210508155011790.png)