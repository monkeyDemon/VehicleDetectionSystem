//声明用来存储普通系统参数的结构体
struct Common{
    char *DetectVideoPath;       //算法读取的视频路径
    int camera_index;            //摄像头设备号，如果使用视频不使用摄像头，则需要赋值为-1
    int GPU_INDEX;               //使用的GPU硬件设备标号，不使用GPU赋值-1
    float MaxCarArea;            //用于检测结果筛选，大于原图面积的MaxCarArea的候选框将会被剔除
    float NMSthresh;             //非极大指抑制阈值
    int InterestedArea_X;
    int InterestedArea_Y;
    int InterestedArea_Width;
    int InterestedArea_Height;
    int TolerateDistance;      //检测框被允许距离感兴趣区域边缘的最小距离
};


//声明用来存储UI参数的结构体
struct UI{
    int MainWindowWidth;
    int MainWindowHeight;
    int StartLocationX;
    int StartLocationY;
};


//声明用来存储KCF算法参数的结构体
struct KCF{
    bool HOG;
    bool FIXEDWINDOW;
    bool MULTISCALE;
    bool SILENT;
    bool LAB;
}; 

//声明用来存储YOLO算法参数的结构体
struct YOLO{
    float Thresh;
    float Hier_thresh;
    char *Data_cfg;              //数据集的配置文件
    char *Cfg;                    //YOLO框架Darknet的配置文件
    char *weights;               //卷积神经网络的权重文件
};

