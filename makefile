GPU=1
CUDNN=0
OPENCV=1
DEBUG=0

ARCH= -gencode arch=compute_20,code=[sm_20,sm_21] \
      -gencode arch=compute_30,code=sm_30 \
      -gencode arch=compute_35,code=sm_35 \
      -gencode arch=compute_50,code=[sm_50,compute_50] \
      -gencode arch=compute_52,code=[sm_52,compute_52]

# This is what I use, uncomment if you know your arch and want to specify
ARCH=  -gencode arch=compute_52,code=compute_52

VPATH=./src/  #特殊变量VPATH，make回到该路径下寻找依赖文件和目标文件
EXEC=carDetect
OBJDIR=./obj/

CC=g++
NVCC=nvcc #nvcc是CUDA的编译工具
OPTS=-Ofast
LDFLAGS= -lm -pthread 
COMMON= 
CFLAGS=-Wall -Wfatal-errors 

ifeq ($(DEBUG), 1) 
OPTS=-O0 -g
endif

CFLAGS+=$(OPTS)

ifeq ($(OPENCV), 1) 
COMMON+= -DOPENCV
CFLAGS+= -DOPENCV
LDFLAGS+= `pkg-config --libs opencv` 
COMMON+= `pkg-config --cflags opencv` 
endif

ifeq ($(GPU), 1) 
COMMON+= -DGPU -I/usr/local/cuda/include/
CFLAGS+= -DGPU
LDFLAGS+= -L/usr/local/cuda/lib64 -lcuda -lcudart -lcublas -lcurand
endif

ifeq ($(CUDNN), 1) 
COMMON+= -DCUDNN 
CFLAGS+= -DCUDNN
LDFLAGS+= -lcudnn
endif

OBJ=gemm.o utils.o cuda.o deconvolutional_layer.o convolutional_layer.o list.o image.o activations.o im2col.o col2im.o blas.o crop_layer.o dropout_layer.o maxpool_layer.o softmax_layer.o data.o matrix.o network.o connected_layer.o cost_layer.o parser.o option_list.o darknet.o detection_layer.o captcha.o route_layer.o writing.o box.o nightmare.o normalization_layer.o avgpool_layer.o coco.o dice.o yolo.o detector.o layer.o compare.o regressor.o classifier.o local_layer.o swag.o shortcut_layer.o activation_layer.o rnn_layer.o gru_layer.o rnn.o rnn_vid.o crnn_layer.o demo.o tag.o cifar.o go.o  batchnorm_layer.o art.o region_layer.o reorg_layer.o lsd.o super.o voxel.o tree.o 
ifeq ($(GPU), 1) 
LDFLAGS+= -lstdc++ 
OBJ+=convolutional_kernels.o deconvolutional_kernels.o activation_kernels.o im2col_kernels.o col2im_kernels.o blas_kernels.o crop_layer_kernels.o dropout_layer_kernels.o maxpool_layer_kernels.o network_kernels.o avgpool_layer_kernels.o
endif

OBJS = $(addprefix $(OBJDIR), $(OBJ))
DEPS = $(wildcard src/*.h) makefile

all: obj $(EXEC)

$(EXEC): $(OBJS) ./obj/fhog.o ./obj/kcftracker.o
	$(CC) $(COMMON) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# $^  所有的依赖目标的集合。以空格分隔。如果在依赖目标中有多个重复的,
#     那个这个变量会去除重复的依赖目标,只保留一份。
# $@  表示规则中的目标文件集
# $<  第一个依赖文件

$(OBJDIR)%.o: %.c $(DEPS)
	$(CC) $(COMMON) $(CFLAGS) -c $< -g  -o $@

$(OBJDIR)%.o: %.cu $(DEPS)
	$(NVCC) $(ARCH) $(COMMON) --compiler-options "$(CFLAGS)" -c $< -g  -o $@

./obj/fhog.o:./src/fhog.cpp ./src/fhog.hpp
	g++ -o ./obj/fhog.o -c ./src/fhog.cpp -g  $(COMMON) $(CFLAGS)

./obj/kcftracker.o:./src/kcftracker.cpp ./src/kcftracker.hpp ./src/ffttools.hpp ./src/fhog.hpp ./src/labdata.hpp ./src/recttools.hpp ./src/tracker.h
	g++ -o ./obj/kcftracker.o -c ./src/kcftracker.cpp -g  $(COMMON) $(CFLAGS)

obj:
	mkdir -p obj

.PHONY: clean

clean:
	rm -rf $(OBJS) $(EXEC) ./obj/fhog.o ./obj/kcftracker.o

