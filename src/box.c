#include "box.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include "utils.h"
#include <memory.h>

box float_to_box(float *f, int stride)
{
    box b;
    b.x = f[0];
    b.y = f[1*stride];
    b.w = f[2*stride];
    b.h = f[3*stride];
    return b;
}

dbox derivative(box a, box b)
{
    dbox d;
    d.dx = 0;
    d.dw = 0;
    float l1 = a.x - a.w/2;
    float l2 = b.x - b.w/2;
    if (l1 > l2){
        d.dx -= 1;
        d.dw += .5;
    }
    float r1 = a.x + a.w/2;
    float r2 = b.x + b.w/2;
    if(r1 < r2){
        d.dx += 1;
        d.dw += .5;
    }
    if (l1 > r2) {
        d.dx = -1;
        d.dw = 0;
    }
    if (r1 < l2){
        d.dx = 1;
        d.dw = 0;
    }

    d.dy = 0;
    d.dh = 0;
    float t1 = a.y - a.h/2;
    float t2 = b.y - b.h/2;
    if (t1 > t2){
        d.dy -= 1;
        d.dh += .5;
    }
    float b1 = a.y + a.h/2;
    float b2 = b.y + b.h/2;
    if(b1 < b2){
        d.dy += 1;
        d.dh += .5;
    }
    if (t1 > b2) {
        d.dy = -1;
        d.dh = 0;
    }
    if (b1 < t2){
        d.dy = 1;
        d.dh = 0;
    }
    return d;
}

float overlap(float x1, float w1, float x2, float w2)
{
    float l1 = x1 - w1/2;
    float l2 = x2 - w2/2;
    float left = l1 > l2 ? l1 : l2;
    float r1 = x1 + w1/2;
    float r2 = x2 + w2/2;
    float right = r1 < r2 ? r1 : r2;
    return right - left;
}


//两个矩形的交
float box_intersection(box a, box b)
{
    float w = overlap(a.x, a.w, b.x, b.w);
    float h = overlap(a.y, a.h, b.y, b.h);
    if(w < 0 || h < 0) return 0;
    float area = w*h;
    return area;
}
//两个矩形的并
float box_union(box a, box b)
{
    float i = box_intersection(a, b);
    float u = a.w*a.h + b.w*b.h - i;
    return u;
}

//新加函数
float box_minArea(box a, box b)
{
    float area1=a.w*a.h;
    float area2=b.w*b.h;
    return area1<area2?area1:area2;
}
//新加函数
float box_IsIn(box a, box b)
{
    return box_intersection(a, b)/box_minArea(a, b);
}

//新增针对Rect的非极大值抑制------------------------------------
//两个矩形的交
float Intersection(Rect a, Rect b)
{
    float w = 0;
    if( a.x<=b.x && b.x<=a.x+a.width ){
	w=a.x+a.width-b.x;
    }else if( b.x<=a.x && a.x<=b.x+b.width ){
	w=b.x+b.width-a.x;
    }else{
	w=0;
    }
    float h = 0;
    if( a.y<=b.y && b.y<=a.y+a.height ){
	h=a.y+a.height-b.y;
    }else if( b.y<=a.y && a.y<=b.y+b.height ){
	h=b.y+b.height-a.y;
    }else{
	h=0;
    }
    float area = w*h;
    return area;
}
//两个矩形的并
float Union(Rect a, Rect b)
{
    float i = Intersection(a, b);
    float u = a.width*a.height + b.width*b.height - i;
    return u;
}

float IOU(Rect a ,Rect b)
{
    return Intersection(a, b)/Union(a, b);
}
//新增针对Rect的非极大值抑制------------------------------------


float box_iou(box a, box b)
{
    return box_intersection(a, b)/box_union(a, b);
}

float box_rmse(box a, box b)
{
    return sqrt(pow(a.x-b.x, 2) + 
                pow(a.y-b.y, 2) + 
                pow(a.w-b.w, 2) + 
                pow(a.h-b.h, 2));
}

dbox dintersect(box a, box b)
{
    float w = overlap(a.x, a.w, b.x, b.w);
    float h = overlap(a.y, a.h, b.y, b.h);
    dbox dover = derivative(a, b);
    dbox di;

    di.dw = dover.dw*h;
    di.dx = dover.dx*h;
    di.dh = dover.dh*w;
    di.dy = dover.dy*w;

    return di;
}

dbox dunion(box a, box b)
{
    dbox du;

    dbox di = dintersect(a, b);
    du.dw = a.h - di.dw;
    du.dh = a.w - di.dh;
    du.dx = -di.dx;
    du.dy = -di.dy;

    return du;
}



bool CHK_IND(Rect p,Rect q,float demo_NMSthresh) {
        /*int Sz=200;
	return abs((p).x-(q).x)<=Sz&&abs((p).width-(q).width)<=Sz&&abs((p).y-(q).y)<=Sz&&abs((p).height-(q).height)<=Sz;*/
	
	bool result =false;

	/*
	box pp,qq;
	pp.x=p.x;
	pp.y=p.y;
	pp.w=p.width;
	pp.h=p.height;
	qq.x=q.x;
	qq.y=q.y;
	qq.w=q.width;
	qq.h=q.height;
	

	if (box_iou(pp, qq) > demo_NMSthresh){
		//非极大值抑制
		result=true;
	}else{ 
	    //检查一个框是否嵌入在另一个框中
	    //在有交集的前提下进行
	    float inter=box_intersection(pp, qq);
	    if(inter>0){
		float area1=pp.w*pp.h;//目标框i的面积
		float area2=qq.w*qq.h;//目标框j的面积
		float areamin=area1>area2?area2:area1;
		//如果较小的框的面积与交的面积大致相等，认为嵌套在其中
		if( areamin<inter*1.05 && areamin<inter*0.95 ){
			result=true;
		}
	    }
	}
	*/

	if (IOU(p, q) > demo_NMSthresh){
		//非极大值抑制
		result=true;
	}
	return result;
}



void test_dunion()
{
    box a = {0, 0, 1, 1};
    box dxa= {0+.0001, 0, 1, 1};
    box dya= {0, 0+.0001, 1, 1};
    box dwa= {0, 0, 1+.0001, 1};
    box dha= {0, 0, 1, 1+.0001};

    box b = {.5, .5, .2, .2};
    dbox di = dunion(a,b);
    printf("Union: %f %f %f %f\n", di.dx, di.dy, di.dw, di.dh);
    float inter =  box_union(a, b);
    float xinter = box_union(dxa, b);
    float yinter = box_union(dya, b);
    float winter = box_union(dwa, b);
    float hinter = box_union(dha, b);
    xinter = (xinter - inter)/(.0001);
    yinter = (yinter - inter)/(.0001);
    winter = (winter - inter)/(.0001);
    hinter = (hinter - inter)/(.0001);
    printf("Union Manual %f %f %f %f\n", xinter, yinter, winter, hinter);
}
void test_dintersect()
{
    box a = {0, 0, 1, 1};
    box dxa= {0+.0001, 0, 1, 1};
    box dya= {0, 0+.0001, 1, 1};
    box dwa= {0, 0, 1+.0001, 1};
    box dha= {0, 0, 1, 1+.0001};

    box b = {.5, .5, .2, .2};
    dbox di = dintersect(a,b);
    printf("Inter: %f %f %f %f\n", di.dx, di.dy, di.dw, di.dh);
    float inter =  box_intersection(a, b);
    float xinter = box_intersection(dxa, b);
    float yinter = box_intersection(dya, b);
    float winter = box_intersection(dwa, b);
    float hinter = box_intersection(dha, b);
    xinter = (xinter - inter)/(.0001);
    yinter = (yinter - inter)/(.0001);
    winter = (winter - inter)/(.0001);
    hinter = (hinter - inter)/(.0001);
    printf("Inter Manual %f %f %f %f\n", xinter, yinter, winter, hinter);
}

void test_box()
{
    test_dintersect();
    test_dunion();
    box a = {0, 0, 1, 1};
    box dxa= {0+.00001, 0, 1, 1};
    box dya= {0, 0+.00001, 1, 1};
    box dwa= {0, 0, 1+.00001, 1};
    box dha= {0, 0, 1, 1+.00001};

    box b = {.5, 0, .2, .2};

    float iou = box_iou(a,b);
    iou = (1-iou)*(1-iou);
    printf("%f\n", iou);
    dbox d = diou(a, b);
    printf("%f %f %f %f\n", d.dx, d.dy, d.dw, d.dh);

    float xiou = box_iou(dxa, b);
    float yiou = box_iou(dya, b);
    float wiou = box_iou(dwa, b);
    float hiou = box_iou(dha, b);
    xiou = ((1-xiou)*(1-xiou) - iou)/(.00001);
    yiou = ((1-yiou)*(1-yiou) - iou)/(.00001);
    wiou = ((1-wiou)*(1-wiou) - iou)/(.00001);
    hiou = ((1-hiou)*(1-hiou) - iou)/(.00001);
    printf("manual %f %f %f %f\n", xiou, yiou, wiou, hiou);
}

dbox diou(box a, box b)
{
    float u = box_union(a,b);
    float i = box_intersection(a,b);
    dbox di = dintersect(a,b);
    dbox du = dunion(a,b);
    dbox dd = {0,0,0,0};

    if(i <= 0 || 1) {
        dd.dx = b.x - a.x;
        dd.dy = b.y - a.y;
        dd.dw = b.w - a.w;
        dd.dh = b.h - a.h;
        return dd;
    }

    dd.dx = 2*pow((1-(i/u)),1)*(di.dx*u - du.dx*i)/(u*u);
    dd.dy = 2*pow((1-(i/u)),1)*(di.dy*u - du.dy*i)/(u*u);
    dd.dw = 2*pow((1-(i/u)),1)*(di.dw*u - du.dw*i)/(u*u);
    dd.dh = 2*pow((1-(i/u)),1)*(di.dh*u - du.dh*i)/(u*u);
    return dd;
}

typedef struct{
    int index;
    int classs;
    float **probs;
} sortable_bbox;

int nms_comparator(const void *pa, const void *pb)
{
    sortable_bbox a = *(sortable_bbox *)pa;
    sortable_bbox b = *(sortable_bbox *)pb;
    float diff = a.probs[a.index][b.classs] - b.probs[b.index][b.classs];
    if(diff < 0) return 1;
    else if(diff > 0) return -1;
    return 0;
}

void do_nms_obj(box *boxes, float **probs, int total, int classes, float thresh)
{
    int i, j, k;
    sortable_bbox *s = (sortable_bbox *)calloc(total, sizeof(sortable_bbox));

    for(i = 0; i < total; ++i){
        s[i].index = i;       
        s[i].classs = classes;
        s[i].probs = probs;
    }

    qsort(s, total, sizeof(sortable_bbox), nms_comparator);
    for(i = 0; i < total; ++i){
        if(probs[s[i].index][classes] == 0) continue;
        box a = boxes[s[i].index];
        for(j = i+1; j < total; ++j){
            box b = boxes[s[j].index];
            if (box_iou(a, b) > thresh){
                for(k = 0; k < classes+1; ++k){
                    probs[s[j].index][k] = 0;
                }
            }
        }
    }
    free(s);
}


void do_nms_sort(box *boxes, float **probs, int total, int classes, float thresh)
{
    int i, j, k;
    sortable_bbox *s = (sortable_bbox *)calloc(total, sizeof(sortable_bbox));

    for(i = 0; i < total; ++i){
        s[i].index = i;       
        s[i].classs = 0;
        s[i].probs = probs;
    }

    for(k = 0; k < classes; ++k){
        for(i = 0; i < total; ++i){
            s[i].classs = k;
        }
        qsort(s, total, sizeof(sortable_bbox), nms_comparator);
        for(i = 0; i < total; ++i){
            if(probs[s[i].index][k] == 0) continue;
            box a = boxes[s[i].index];
            for(j = i+1; j < total; ++j){
                box b = boxes[s[j].index];
                if (box_iou(a, b) > thresh){
                    probs[s[j].index][k] = 0;
                }
            }
        }
    }
    free(s);
}

void do_nms(box *boxes, float **probs, int total, int classes, float thresh)
{
    int i, j, k;
    for(i = 0; i < total; ++i){
        int any = 0;
        for(k = 0; k < classes; ++k) any = any || (probs[i][k] > 0);
        if(!any) {
            continue;
        }
        for(j = i+1; j < total; ++j){
            if (box_iou(boxes[i], boxes[j]) > thresh){
		//两个框重叠比例过高，则会按照对应来别一一进行抑制
                for(k = 0; k < classes; ++k){
                    if (probs[i][k] < probs[j][k]) probs[i][k] = 0;
                    else probs[j][k] = 0;
                }
            }
        }
    }
}

float maxx=-1;

//加入除非极大值抑制以外的其他筛选条件
vector<Rect> do_nms2(box *boxes, float **probs, int total, int classes, float thresh,float NMSthresh ,float MaxCarArea ,CvRect INRect,int ImageW,int ImageH)
{
    int i, j, k;
    float left;
    float right;
    float top;
    float bot;


    bool * flag = new bool [total];  
    memset(flag,true,total); 

    /*
    //初步筛选
    for(i=0;i<total;++i){
	int classs = max_index(probs[i], classes);
	//判断是否为车
	if((classs==2 ||classs==5||classs==7)){
	    //判断第i个框概率最大的类别是否超过阈值
	    float prob = probs[i][classs];
	    if(prob>thresh){
		float area=boxes[i].w*boxes[i].h;//目标框i的面积
		float bili=boxes[i].w/boxes[i].h;
		
		//判断第i个框面积是否超过阈值
		//&& boxes[i].w<MaxCarArea && boxes[i].h<MaxCarArea
		if(area<MaxCarArea && bili<1.5 && bili>0.66){
		    printf("aaaaa%f\n",bili);
		    left  = (boxes[i].x-boxes[i].w/2.)*ImageW;
        	    right = (boxes[i].x+boxes[i].w/2.)*ImageW;
        	    top   = (boxes[i].y-boxes[i].h/2.)*ImageH;
        	    bot   = (boxes[i].y+boxes[i].h/2.)*ImageH;
		    //float bili=(right-left)/(bot-top);
		    
		    //判断第i个框是否超过了预设范围
		   // if(left<INRect.x || top<INRect.y || right>INRect.x+INRect.width || bot>INRect.y+INRect.height || bili>2 || bili< 0.5){
		    //|| bili>1 || bili<0.7 
		    if( (right-left)<80 || (bot-top)< 70 ){
			//printf("aaaaa%f\n",bili);
			for(k = 0; k < classes; ++k){
			    probs[i][k] = 0;
		    	}
		    	flag[i]=false;
		    }else{
			
			//能到达这里是通过了层层筛选的目标框^_^
		    }
		}else{
		    for(k = 0; k < classes; ++k){
			probs[i][k] = 0;
		    }
		    flag[i]=false;
		    //printf("%s\n","框太大");
		}
	    }else{
		for(k = 0; k < classes; ++k){
		    probs[i][k] = 0;
		}
		flag[i]=false;
		//printf("%s\n","置信度低");
	    }
	}else{
	    for(k = 0; k < classes; ++k){
		probs[i][k] = 0;
	    }
	    flag[i]=false;
	    //printf("%s\n","类别不对");
	    //printf("%d\n",classs);
	}
    }

    
    for(i = 0; i < total; ++i){
	if (flag[i]==true){
	    int classs = max_index(probs[i], classes);
	    float prob1 = probs[i][classs];
	    for(j = i+1; j < total; ++j){
		//首先要保证有必要考虑当前目标框j
		if(flag[j]==false)
		    continue;
		//判断是否需要进行 非极大值抑制
		if (box_iou(boxes[i], boxes[j]) > NMSthresh){
		    //两个框重叠比例过高，进行非极大值抑制
		    classs = max_index(probs[j], classes);
		    float prob2 = probs[j][classs];
		    if (prob1>prob2){
			for(k = 0; k < classes; ++k){
			    probs[j][k] = 0;
		        }
			flag[j]=false;
		    }else{
			if(flag[i]==true){
			    for(k = 0; k < classes; ++k){
			        probs[i][k] = 0;
		            }
			    flag[i]=false;
			}
		    }
		}else if(box_intersection(boxes[i], boxes[j])>0){
		    //没有产生非极大值抑制的情况，筛选出嵌入在其他框中的小框
		    //在有交集的前提下进行
		    float area1=boxes[i].w*boxes[i].h;//目标框i的面积
		    float area2=boxes[j].w*boxes[j].h;//目标框j的面积
		    float UnionArea=box_union(boxes[i], boxes[j]);
		    if(area1<area2){
	   		if(UnionArea<1.05*area2){
			    //认为目标框i在目标框j里面
			    //剔除目标框i
			    if(flag[i]==true){
			    	for(k = 0; k < classes; ++k){
			            probs[i][k] = 0;
		                }
			        flag[i]=false;
			    }
			}
		    }else{
			if(UnionArea<1.05*area1){
			    //认为目标框j在目标框i里面
			    for(k = 0; k < classes; ++k){
				probs[j][k] = 0;
			    }
			    flag[j]=false;
			}
		    }
		}
	    }
	}
    }
    */
    

    //初步筛选
    for(i=0;i<total;++i){
	int classs = max_index(probs[i], classes);
	//判断是否为车
	if((classs==2 ||classs==5||classs==7)){
	    //判断第i个框概率最大的类别是否超过阈值
	    float prob = probs[i][classs];
	    if(prob>thresh){
		left  = (boxes[i].x-boxes[i].w/2.)*ImageW;
            	right = (boxes[i].x+boxes[i].w/2.)*ImageW;
            	top   = (boxes[i].y-boxes[i].h/2.)*ImageH;
            	bot   = (boxes[i].y+boxes[i].h/2.)*ImageH;
	    	if(left < 0) left = 0;
            	if(right > ImageW-1) right = ImageW-1;
            	if(top < 0) top = 0;
            	if(bot > ImageH-1) bot = ImageH-1;

		float width_bili=(float)(right-left)/ImageW;//目标框i的比例	

		float height_bili=(float)(bot-top)/(ImageH);//目标框i的比例	

		float bili=(float)(right-left)/(bot-top);//目标框i面积占整附图的比例
//printf("bili0 %f\n",bili);
		//判断第i个框长或宽是否超过阈值（不能太大）
		if( width_bili<MaxCarArea && height_bili<MaxCarArea && bili<2 && bili>0.5){
			//printf("bili1 %f\n",bili);  
			//能到达这里是通过了层层筛选的目标框^_^2 0.5
		}else{
		    for(k = 0; k < classes; ++k){
			probs[i][k] = 0;
		    }
		    flag[i]=false;
		    //printf("%s\n","框太大");
		}
	    }else{
		for(k = 0; k < classes; ++k){
		    probs[i][k] = 0;
		}
		flag[i]=false;
		//printf("%s\n","置信度低");
	    }
	}else{
	    for(k = 0; k < classes; ++k){
		probs[i][k] = 0;
	    }
	    flag[i]=false;
	    //printf("%s\n","类别不对");
	}
    }

    for(i = 0; i < total; ++i){
	if (flag[i]==true){
	    int classs = max_index(probs[i], classes);
	    float prob1 = probs[i][classs];
	    
	    left  = (boxes[i].x-boxes[i].w/2.)*ImageW;
            right = (boxes[i].x+boxes[i].w/2.)*ImageW;
            top   = (boxes[i].y-boxes[i].h/2.)*ImageH;
            bot   = (boxes[i].y+boxes[i].h/2.)*ImageH;
	    //printf("boxes[i].x%f\n",boxes[i].x);
	    //printf("boxes[i].w%f\n",boxes[i].w);
		//printf("left%f\n",left);
	    if(left < 0) left = 0;
            if(right > ImageW-1) right = ImageW-1;
            if(top < 0) top = 0;
            if(bot > ImageH-1) bot = ImageH-1;
	    Rect boxI=Rect(left,top,right-left,bot-top);

	    for(j = i+1; j < total; ++j){
		//首先要保证有必要考虑当前目标框j
		if(flag[j]==false)
		    continue;

		left  = (boxes[j].x-boxes[j].w/2.)*ImageW;
                right = (boxes[j].x+boxes[j].w/2.)*ImageW;
                top   = (boxes[j].y-boxes[j].h/2.)*ImageH;
                bot   = (boxes[j].y+boxes[j].h/2.)*ImageH;
	        if(left < 0) left = 0;
                if(right > ImageW-1) right = ImageW-1;
                if(top < 0) top = 0;
                if(bot > ImageH-1) bot = ImageH-1;
	        Rect boxJ=Rect((int)left,(int)top,(int)(right-left),(int)(bot-top));
		
		//判断是否需要进行 非极大值抑制
		if (IOU(boxI, boxJ) > NMSthresh){
		    //两个框重叠比例过高，进行非极大值抑制
		    classs = max_index(probs[j], classes);
		    float prob2 = probs[j][classs];
		    if (prob1>prob2){
			for(k = 0; k < classes; ++k){
			    probs[j][k] = 0;
		        }
			flag[j]=false;
		    }else{
			if(flag[i]==true){
			    for(k = 0; k < classes; ++k){
			        probs[i][k] = 0;
		            }
			    flag[i]=false;
			}
		    }
		}
	    }
	}
    }

    //YOLO提供的目标检测候选框是冗余的
    //重新对框进行赋值，完全去掉被筛选掉的框
    vector<Rect> ResultRects;
    //int inter=0;
    for(i = 0; i < total; ++i){
	//目标框i的置信度
	if(flag[i]==true){
	    left  = (boxes[i].x-boxes[i].w/2.)*ImageW;
            right = (boxes[i].x+boxes[i].w/2.)*ImageW;
            top   = (boxes[i].y-boxes[i].h/2.)*ImageH;
            bot   = (boxes[i].y+boxes[i].h/2.)*ImageH;

	    if(left < 0) left = 0;
            if(right > ImageW-1) right = ImageW-1;
            if(top < 0) top = 0;
            if(bot > ImageH-1) bot = ImageH-1;

	    //float bili=((float)(right-left))/(bot-top);
		    //printf("aaaaa%f\n",bili);
	    //Rect tempRect=new Rect(left,top,right-left,bot-top);
	    //ResultRects.push_back(inter,Rect(left,top,right-left,bot-top));
	    ResultRects.push_back(Rect(left,top,right-left,bot-top));
	    
	    //maxx=bili>maxx?bili:maxx;
		    	//printf("aaaaa%f\n",maxx);
	    //inter++;
	}
    }
    //printf("ddd%d\n",ResultRects.size());
    return ResultRects;
}



box encode_box(box b, box anchor)
{
    box encode;
    encode.x = (b.x - anchor.x) / anchor.w;
    encode.y = (b.y - anchor.y) / anchor.h;
    encode.w = log2(b.w / anchor.w);
    encode.h = log2(b.h / anchor.h);
    return encode;
}

box decode_box(box b, box anchor)
{
    box decode;
    decode.x = b.x * anchor.w + anchor.x;
    decode.y = b.y * anchor.h + anchor.y;
    decode.w = pow(2., b.w) * anchor.w;
    decode.h = pow(2., b.h) * anchor.h;
    return decode;
}

