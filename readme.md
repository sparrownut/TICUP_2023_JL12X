# 电赛2023 吉林12*组E题方案部分代码
## 巡线部分算法本机一键测试
    在某个文件夹打开CMD
    执行
    git clone https://github.com/sparrownut/TICUP_2023_JL12X.git
    cd TICUP_2023_JL12X
    pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
    python blackEdge.py
    即可运行,需要连接摄像头，建议在本机电脑上初次尝试，若出现图像则成功
## OPENMV视觉部分
    在openmv文件夹下
## 树莓派视觉部分
    opencv实现双边缘检测和中线识别,在主目录下blackEdge.py  
    若在树莓派上运行，需要将/dev/ttyAMA0硬件串口启用，并给予相应权限  
    若运行成功，需要根据摄像头来设置像素，使摄像头屏幕区域完全和白板屏幕重合  
### 树莓派调整摄像头区域快捷键
    WASD - 屏幕左上角点移动  
    TFGH - 屏幕宽高设置  
    ER - 屏幕旋转角度 
    N - 循迹单次执行快捷键  
    C - 慢速循迹模式切换按键  
    Q - 退出