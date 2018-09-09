#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "demo.h"
#include <sys/time.h>
#include <vector>
#include "kcftracker.hpp"
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp> 
//#include <dlfcn.h>  //调用so文件需要引用
//#include "CWrap.h"
//#include "SysParameter.h"

#define FRAMES 3
#define TrackerNUmber 500

using namespace cv;  

#ifdef OPENCV

KCFTracker tracker[TrackerNUmber];
Rect rect;
int nFrames = 0;  //已经处理的帧数
int Framecount=0;  //记录已经处理的帧数
int CarCounts=0;   //车辆计数
int flags[TrackerNUmber]={0};//标识每个跟踪器是否在跟踪一辆车
int DetectKcfFlag[TrackerNUmber]={0};//标志跟踪框是否被检测到，检测到置为１,Steve
int UndetectKcfCount[TrackerNUmber]={0};//记录为检测到追踪框的次数，Steve
int flagcount[TrackerNUmber]={0};
Rect members[TrackerNUmber];
Point rook_points[TrackerNUmber-1][1000];
double t=0;
Rect result1[TrackerNUmber];//记录前8帧追踪框
int record[TrackerNUmber]={0};//判断追踪框刷新次数
bool selectflag=false;//jzh
bool selectObject = false;//jzh
bool stopflag=false;//jzh
Point origin;//jzh
Mat myMat;//jzh
int Sz=50;


//区域
int ImageWidth,ImageHeight;//图像大小
CvRect InterestedArea;//感兴趣区域
Rect selection;//jzh  
static CvVideoWriter *mVideoWriter;//jzh
Mat frame;//jzh

static char **demo_names;
static image **demo_alphabet;
static int demo_classes;

static float **probs;
static box *boxes;
static bool *AllBoxFlag;
static network net;
static image in   ;
static image in_s ;
static image det  ;
static image det_s;
IplImage* Image_for_kcf;
static image disp = {0};
static CvCapture * cap;
static float fps = 0;
static float demo_thresh = 0;
static float demo_hier_thresh = .5;
static float demo_MaxCarArea=0;
static float demo_NMSthresh;
static float demo_TolerateDistance=20;

static float *predictions[FRAMES];
static int demo_index = 0;
static image images[FRAMES];
static float *avg;



static void onMouse(int event, int x, int y, int, void*)//jzh
{
		if (selectObject)
		{
			selection.x= x < origin.x ? x : origin.x ;
			selection.y= y < origin.y ? y : origin.y ;
			selection.width = std::abs(x - origin.x);
			selection.height = std::abs(y - origin.y);
		}

		switch (event)
		{
		case CV_EVENT_LBUTTONDOWN:
			origin = Point(x, y);
			selection = Rect(x, y, 0, 0);
			selectObject = true;//标识开始选择物体
			break;
		case CV_EVENT_LBUTTONUP:
			selectObject = false;
			stopflag=false;
			if (selection.width > 0 && selection.height > 0){
				InterestedArea.x= selection.x < 0 ? 0 : selection.x ;
				InterestedArea.y= selection.y < 0 ? 0 : selection.y ;
				InterestedArea.width = selection.width > ImageWidth ? ImageWidth : selection.width ;
				InterestedArea.height = selection.height > ImageHeight ? ImageHeight : selection.height ;
			}
			break;
		}
}


void *KCF_track(vector<Rect> rects,Mat img){
	//重新跟踪一遍（如果第k个缓冲区有跟踪车辆，需要更新跟踪框的位置）
	//printf("ddd%d\n",Rects.size());
	for(int k=0;k<TrackerNUmber;k++)
	{	
		if(flags[k]==1)//第k个缓冲区有跟踪车辆
		{
			//printf("find %d buffer has car\n",k);
			DetectKcfFlag[k]=0;//初始化车辆跟踪框未被检测到(待定)
			//更新跟踪框的位置
			Rect result = tracker[k].update(img);
			members[k]=result;
			record[k]++;//第k个检测框已经跟踪的帧数+1
			rook_points[k][flagcount[k]]=Point(result.x+result.width/2,result.y+result.height/2);//更新第k个检测框的中心
			flagcount[k]++;//？？？？

			//如果已经追踪了16帧，但是追踪框没有移动，认为该车辆处于停车状态
			if( record[k]%16==0 && abs(result.y-result1[k].y)<result.width/32 )
			{		
				printf("\nThe Program find a Car anchor at the %d frame !!!\n",Framecount);
				printf("This car is located at ( %d , %d , %d , %d ) \n\n",result.x,result.y,result.width,result.height);
				//putText(img, "anchor!!!", Point(result.x, result.y), 1.5, 1, Scalar(0, 0, 255)); 
				//停车时间有点过分了，不再跟踪
				if(record[k]>5000)
					flags[k]=0;
			}
			//如果跟踪框超出了感兴趣区域，不再跟踪

			//小注一下，暂时

			if (result.y+result.height>=(InterestedArea.y+InterestedArea.height)||result.x<=InterestedArea.x||result.x+result.width>=InterestedArea.x+InterestedArea.width||result.y<=InterestedArea.y)
//if (result.y+result.height>=(InterestedArea.y+40*InterestedArea.height/41)||result.x-InterestedArea.x<=InterestedArea.width/21||result.x+result.width>=InterestedArea.x+20*InterestedArea.width/21||InterestedArea.y-result.y<=InterestedArea.height/41)
			{
				CarCounts++;
				flags[k]=0;
				flagcount[k]=0;
				//printf("free %d buffer \n",k);
			}
			//result1用于与record比较检测框是否移动，每16帧更新一次取值
			result1[k]=record[k]%16==0?result:result1[k];
			
		}
	}	
	
	//处理这一帧检测算法得到的目标框（判断得到目标框中是否有新的车辆）
	vector<Rect>::const_iterator r;
	for(r = rects.begin(); r != rects.end(); r++)  
	{
		//rect=Rect(InterestedArea.x+r->x,InterestedArea.y+r->y,r->width,r->height);
		rect=Rect(r->x,r->y,r->width,r->height);
		int  m=0;
		int index=-1;
                
		for(int k=0;k<TrackerNUmber;k++)
		{
			if(flags[k]==1)//如果第k个缓冲区有跟踪车辆
			{
				
				Rect result = members[k];
				if(CHK_IND(result,rect,demo_NMSthresh))
				{
				    DetectKcfFlag[k]=1;//将检测到的跟踪框的标志计为１(待定)
				    //tracker[k].init(rect, img);
				    //members[k]=rect;
				    
				    //rook_points[k][flagcount[k]-1]=Point(rect.x+rect.width/2,rect.y+rect.height/2);
				    m++;
				}	
			}
			else
				index=k;
		}
		if(index==-1)
		{
			cout<<"车辆数量超出所能追踪的车辆数";
			exit(0);
		}
		
		//当前检测框如果没有与任何一个已跟踪框重合，且没有超过感兴趣区域。认为是新的检测框
		if(m==0&&(((rect.y+rect.height)<(InterestedArea.y+InterestedArea.height-demo_TolerateDistance))&&rect.x-InterestedArea.x>demo_TolerateDistance && InterestedArea.x+InterestedArea.width-rect.x-rect.width>demo_TolerateDistance &&rect.y-InterestedArea.y>demo_TolerateDistance ))
		//if(m==0)
		{	
			//printf("check init \n");
			tracker[index].init(rect, img);//Rect(xMin, yMin, width, height)
/*
			printf("init %d \n",rect.x);
			printf("init %d \n",rect.y);
			printf("init %d \n",rect.width);
			printf("init %d \n",rect.height);
*/
			rook_points[index][flagcount[index]]=Point(rect.x+rect.width/2,rect.y+rect.height/2);//初始化当前检测框的中心
			flags[index]=1;
			flagcount[index]++;
			//printf("flags[index]= %d \n",index);
			/*笙歌代码
			Rect result = tracker[index].update(img);
			members[index]=result;
			record[index]=0;
			result1[index]=result;
			*/
			//myth
			members[index]=rect;
			record[index]=0;
			result1[index]=rect;
			//
		}
	}/*
	//如果追踪框未被检测到并且追踪框离边界较近则将其消除,Steve
        for(int i=0;i<TrackerNUmber;i++)
	{
		Rect result=members[i];
		//达到消除追踪框的要求一次
		if (flags[i]==1&&DetectKcfFlag[i]==0&&((result.y+result.height>=InterestedArea.y+InterestedArea.height-InterestedArea.height/30)||result.x<=InterestedArea.x||(result.x+result.width>=InterestedArea.x+InterestedArea.width)||result.y<=InterestedArea.y))
		{
			UndetectKcfCount[i]++;
			if(UndetectKcfCount[i]>20)//要求达到3次才消除
			{
			    printf("UndetectKcfCount%d \n",UndetectKcfCount[i]);
			    CarCounts++;
			    flags[i]=0;
			    flagcount[i]=0;
			}
		}
	}*/
	nFrames++;
	cout<<"Current Count cars number:"<<CarCounts<<endl;
    //绘制追踪框轨迹(待定)
/*方法一，描点
    for (int k=0;k<TrackerNUmber;k++)
    {
	int r=255;
	int g=0;
	int b=0;
        if(flags[k]==1)
        {
	    for(int j=0;j<flagcount[k];j++)
	    {
		Point p=rook_points[k][j];
		int x=p.x;
		int y=p.y;
		det.data[x + y*det.w + 0*det.w*det.h] = r;
        	det.data[x + y*det.w + 1*det.w*det.h] = g;
        	det.data[x + y*det.w + 2*det.w*det.h] = b;
	    }
        }
    }
*/
//方法二
/*for(int i=0;i<TrackerNUmber;i++)
{
    if(flags[i]==1)
    {
	for(int j=0;j<flagcount[i]-1;j++)
	{
	    Point p1=rook_points[i][j];
	    Point p2=rook_points[i][j+1];
	    line(det,p1, p2,Scalar(0, 0, 255),1,8,0);
	}
    }
}*/
}


//获取一帧图像
void *fetch_in_thread(void *ptr)
{
    in = get_image_from_stream2(cap,Image_for_kcf);
    if(!in.data){
        error("Stream closed.");
    }
    in_s = resize_image(in, net.w, net.h);
    return 0;
}

//检测
void *detect_in_thread(void *ptr)
{
    layer l = net.layers[net.n-1];
    float *X = det_s.data;
    float *prediction = network_predict(net, X);

    memcpy(predictions[demo_index], prediction, l.outputs*sizeof(float));
    mean_arrays(predictions, FRAMES, l.outputs, avg);
    l.output = avg;
    free_image(det_s);

    //================================得到初步检测框===========================
    //结果存于probs, boxes中
    if(l.type == DETECTION){
        get_detection_boxes(l, 1, 1, demo_thresh, probs, boxes, 0);
    } else if (l.type == REGION){
        get_region_boxes(l, 1, 1, demo_thresh, probs, boxes, 0, 0, demo_hier_thresh);
    } else {
        error("Last layer must produce detections\n");
    }

    images[demo_index] = det;
    det = images[(demo_index + FRAMES/2 + 1)%FRAMES];
    demo_index = (demo_index + 1)%FRAMES;

    //=================================得到筛选过的候选框========================
    //进行的筛选包括：NMS(non maximum suppression)非极大值抑制,去掉面积过大的框，长宽比例异常的框，置信度过低的框等
    vector<Rect> rects;
    //do_nms(boxes, probs, l.w*l.h*l.n, l.classes,0.9);
    //这里要改。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。
    rects = do_nms2(boxes, probs, l.w*l.h*l.n, l.classes,demo_thresh, demo_NMSthresh,demo_MaxCarArea,InterestedArea,det.w,det.h);


    //===============================KCF追踪======================================

    //Mat m(det.w,det.h, CV_32FC3, det.data);
    Mat m = cvarrToMat(Image_for_kcf);
    //Mat m(Image_for_kcf);
    //cvShowImage("Test",Image_for_kcf);
    //imshow("Test",m);
    //cvWaitKey(3000);

    //IplImage *src;
    //src=&IplImage(m);
    //cvShowImage("Test",(CvArr*)m);
    //cvWaitKey(20000);
    //IplImage *src=&IplImage（img）;
    //Mat img=ipl_to_image(IplImage* src);
    
    if(rects.empty()==false){
	//printf("\nvextorSize%d\n",rects.size());
	KCF_track(rects,m);
    }

    //===================================绘图=====================================
    //绘制检测到的车辆
    draw_detections2(det, members,flags,TrackerNUmber, demo_alphabet);
    
    //绘制感兴趣区域
    int left  = InterestedArea.x;
    int right = InterestedArea.x+InterestedArea.width;
    int top   = InterestedArea.y;
    int bot   = InterestedArea.y+InterestedArea.height;
    draw_box_width(det, left, top, right, bot, 5, 255, 255, 255);

    /*
for (int i=0;i<TrackerNUmber;i++)
		    {
			const Point* pt[1] = { rook_points[i] };
		        int npt[1] = { flagcount[i] };
		        polylines(myMat, pt, npt, 1, 0, Scalar(0, 0, 255), 1);
		    }
    */
    //draw_TrackLines(det,TrackerNUmber,rook_points,flagcount);
    return 0;
}

double get_wall_time()
{
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

void demo( Common Common_p, UI UI_p,KCF KCF_p,YOLO YOLO_p,char **names, int classes)
{
    //==========================================变量赋值==================================================
    int cam_index=Common_p.camera_index;
    const char *filename=Common_p.DetectVideoPath;
    demo_MaxCarArea=Common_p.MaxCarArea;
    demo_NMSthresh=Common_p.NMSthresh;
    demo_TolerateDistance=Common_p.TolerateDistance;

    //感兴趣区域
    InterestedArea=cvRect(Common_p.InterestedArea_X,Common_p.InterestedArea_Y,Common_p.InterestedArea_Width,Common_p.InterestedArea_Height);
    
    demo_thresh=YOLO_p.Thresh;                    //检测阈值 
    demo_hier_thresh=YOLO_p.Hier_thresh;       //？？？
    char *cfgfile=YOLO_p.Cfg;
    char *weightfile=YOLO_p.weights;

    image **alphabet = load_alphabet();
    demo_names = names;                        //类别名
    demo_alphabet = alphabet;
    demo_classes = classes;                    //类别数
    //==========================================变量赋值完毕==================================================
    


    //-----------------------------------------初始化KCF追踪器--------------------------------------------------
    // Create KCFTracker object
    //KCFTracker tracker[TrackerNUmber];
    for (int i = 0;i < TrackerNUmber;i++)
    {
	tracker[i]=KCFTracker(KCF_p.HOG, KCF_p.FIXEDWINDOW, KCF_p.MULTISCALE, KCF_p.LAB);
    }
    //----------------------------------------初始化KCF追踪器完毕-----------------------------------------------



    //==========================================初始化YOLO==================================================
    //解析配置文件
    net = parse_network_cfg(cfgfile);
    //加载权重文件
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 1);
    srand(2222222);
    cvWaitKey(1000);
    //==========================================初始化YOLO完毕==================================================

    //判断读取视频还是摄像头
    if(cam_index==-1){
        printf("video file: %s\n", filename);
        cap = cvCaptureFromFile(filename);
         if(cap){
             int mfps = cvGetCaptureProperty(cap,CV_CAP_PROP_FPS);
             mVideoWriter=cvCreateVideoWriter("Output.avi",CV_FOURCC('M','J','P','G'),mfps,cvSize(cvGetCaptureProperty(cap,CV_CAP_PROP_FRAME_WIDTH),cvGetCaptureProperty(cap,CV_CAP_PROP_FRAME_HEIGHT)),1);//jzh
         }
    }else{
        cap = cvCaptureFromCAM(cam_index);
    }
    if(!cap) error("Couldn't connect to webcam.\n");

    printf("\nTraffic Detection Starting...\n");
    
    IplImage* IPL_image=cvQueryFrame(cap);
    ImageWidth=IPL_image->width;
    ImageHeight=IPL_image->height;

    layer l = net.layers[net.n-1];
    int j;
    avg = (float *) calloc(l.outputs, sizeof(float));
    for(j = 0; j < FRAMES; ++j) predictions[j] = (float *) calloc(l.outputs, sizeof(float));
    for(j = 0; j < FRAMES; ++j) images[j] = make_image(1,1,3);
    boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
    //AllBoxFlag=(bool *)calloc(l.w*l.h*l.n, sizeof(bool));
    probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
    for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = (float *)calloc(l.classes, sizeof(float));

    pthread_t fetch_thread;    //获取线程
    pthread_t detect_thread;   //检测线程

    fetch_in_thread(0);        //从视频流中获取一帧，存于in和in_s
    det = in;
    det_s = in_s;

    fetch_in_thread(0);
    detect_in_thread(0);
    disp = det;
    det = in;
    det_s = in_s;

    for(j = 0; j < FRAMES/2; ++j){
        fetch_in_thread(0);
        detect_in_thread(0);
        disp = det;
        det = in;
        det_s = in_s;
    }

    Framecount = 0;

    cvNamedWindow("Traffic Detection", CV_WINDOW_NORMAL); 
    cvMoveWindow("Traffic Detection", 0, 0);
    cvResizeWindow("Traffic Detection", 1352, 1013);
    //namedWindow("Test"); 

    double before = get_wall_time();

    while(1){
        ++Framecount;
        if(1){

	    if(stopflag==false){
		    if(pthread_create(&fetch_thread, 0, fetch_in_thread, 0)) error("Thread creation failed");
		    if(pthread_create(&detect_thread, 0, detect_in_thread, 0)) error("Thread creation failed");
		    //show_image(disp, "Traffic Detection");
		    
		    //show_image(disp, "Traffic Detection");
		    myMat=iplImage2Mat(disp);
		    //Steve

		    for (int i=0;i<TrackerNUmber;i++)
		    {
			const Point* pt[1] = { rook_points[i] };
		        int npt[1] = { flagcount[i] };
		        polylines(myMat, pt, npt, 1, 0, Scalar(0, 0, 255), 2);
		    }
		    imshow("Traffic Detection",myMat);
		    
		    //save_video(disp,mVideoWriter); //jzh


		    pthread_join(fetch_thread, 0);
		    pthread_join(detect_thread, 0);
		    
		    free_image(disp);
		    disp = det;
		    det   = in;
		    det_s = in_s;
            }
            char c = (char)cvWaitKey(1);//每帧停留的毫秒数
            if (c == 27){
		//暂不做任何处理
            }
	    switch (c)
	    {
              case 'p':
              stopflag=!stopflag;            
              break;
            }
	    if(stopflag==true)//jzh
	    {
		setMouseCallback("Traffic Detection", onMouse, 0);
	    }
	}


        double after = get_wall_time();
        float curr = 1./(after - before);
        fps = curr;
        before = after;
    }
}
#else
void demo(Common Common_p, UI UI_p,KCF KCF_p,YOLO YOLO_p, char **names, int classes)
{
    fprintf(stderr, "Demo needs OpenCV for webcam images.\n");
}
#endif

