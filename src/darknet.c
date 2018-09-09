#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include "parser.h"
#include "utils.h"
#include "cuda.h"
#include "blas.h"
#include "connected_layer.h"


//    
//                         _oo0oo_  
//                        o8888888o  
//                        88" . "88  
//                        (| -_- |)  
//                        0\  =  /0  
//                      ___/`---'\___  
//                    .' \\|     |// '.  
//                   / \\|||  :  |||// \  
//                  / _||||| -:- |||||- \  
//                 |   | \\\  -  /// |   |  
//                 | \_|  ''\---/''  |_/ |  
//                 \  .-\__  '-'  ___/-. /  
//               ___'. .'  /--.--\  `. .'___  
//            ."" '<  `.___\_<|>_/___.' >' "".  
//           | | :  `- \`.;`\ _ /`;.`/ - ` : | |  
//           \  \ `_.   \_ __\ /__ _/   .-` /  /  
//       =====`-.____`.___ \_____/___.-`___.-'=====  
//                         `=---='  
//    
//    
//       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
//  
//                 佛祖保佑         永无BUG  
//  
//


extern void run_detector(int argc, char **argv);
//extern void run_cifar(int argc, char **argv);



int main(int argc, char **argv)
{
    // ./darknet detector demo cfg/coco.data cfg/yolo.cfg yolo.weights data/test1.mp4 -thresh 0.25

    /*
    gpu_index = find_int_arg(argc, argv, "-i", 0);
    if(find_arg(argc, argv, "-nogpu")) {
        gpu_index = -1;
    }

#ifndef GPU
    gpu_index = -1;
#else
    if(gpu_index >= 0){
        cuda_set_device(gpu_index);
    }
#endif
    */

    run_detector(argc, argv);
    
    return 0;
}
