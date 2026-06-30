# trtx/ — 第三方依赖说明 (vendored)

本目录的 GPU YOLO 基于第三方 **tensorrtx**(wang-xinyu) 的 `yolov5-v5.0` 标签。
按「只提交我的代码」原则, **上游文件不入库**, 仓库里只保留我方文件:

| 文件 | 归属 | 说明 |
|---|---|---|
| `yolo_follow.cpp`   | 我写 | 读 grabber 左目帧 → yolov5s 检 person → 采深度 → 写 `runtime/target.json` |
| `CMakeLists.txt`    | 我改 | x86→aarch64 路径 + tegra 链接(nvdla/nvmedia) + Xavier `sm_72` + `yolo_follow` 目标 |
| `run_yolo_follow.sh`| 我写 | 启动脚本(设好 LD_LIBRARY_PATH) |

## 还原成可编译目录(容器内 `/apollo/follow_data/trtx/`)

1. 取上游(不入库的那些文件):
   ```
   git clone -b yolov5-v5.0 --depth 1 https://github.com/wang-xinyu/tensorrtx
   cp tensorrtx/yolov5/{yolov5.cpp,yololayer.cu,yololayer.h,common.hpp,utils.h,calibrator.cpp,calibrator.h,logging.h,macros.h,cuda_utils.h} ./
   ```
2. 用本仓库的 `CMakeLists.txt` 覆盖上游同名文件。
3. 打 **calibrator.cpp 补丁**(OpenCV 2.4 无 dnn 模块, 否则编不过):
   - 删掉 `#include <opencv2/dnn/dnn.hpp>`
   - 把 `cv::dnn::blobFromImages(...)` 那一行换成手写 NCHW float 填充(scale `1/255`, swapRB=true)。
4. 权重 `.wts`: 在 4090 上 `yolov5s.pt`(v5.0 release) → tensorrtx `gen_wts` → `yolov5s.wts`(torch≥2.6 需 `weights_only=False` + `PYTHONPATH=<yolov5 v5.0 repo>`)。
5. 编译 + 建引擎(容器内, TRT6/CUDA10):
   ```
   mkdir -p build && cd build && cmake .. && make
   LD_LIBRARY_PATH=$PWD:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda-10.0/lib64 \
     ./yolov5 -s yolov5s.wts yolov5s.engine s        # 出 yolov5s.engine
   ```
6. 跑跟随检测: `bash ../run_yolo_follow.sh`

> 环境: apollo_dev_nvidia 容器(TensorRT 6.0.1 + libnvonnxparser 6.0.1 + CUDA 10.0 + nvcc10 + cmake3.5 + OpenCV C++ 2.4.8)。
> 实测: yolov5s FP16 单帧 ~16ms ≈ 62fps; bus.jpg 正确检出 3 人。
