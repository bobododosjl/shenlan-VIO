大作业为两个代码作业

# 1.更优的优化策略

a.选用更优的LM策略，使得VINS-Mono在MH-05数据集上收敛速度更快或更高。

b.实现dog-leg算法替换LM算法，并测试替换后的VINS—Mono在MH-05上算法精度。

详细的实验报告，包括：对迭代时间和精度进行评估，其中精度评估可以采用evo工具对轨迹精度进行评估，轨迹真实值在zip中已经给出。

# 2.更快的makehessian矩阵

可以采用任何一种或多种加速方式（如多线程，如sse指令集等）对信息矩阵的拼接函数加速，并给出详细的实验对比报告。
