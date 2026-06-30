// yolo_follow.cpp — GPU 人体检测(tensorrtx yolov5s engine) → 写 runtime/target.json 喂 follow_controller.py
//
// 文件式管线(与现有架构一致, 不走 ROS):
//   zkhy_grabber 写 grab/left_latest.ppm + grab/depth_latest.pgm(16bit,mm) + grab/camera_status.json
//   本程序读这些 → yolov5s 检 person → 挑最大框 → 采框中心深度 → 写 runtime/target.json
//   follow_controller.py 读 target.json → 3~4m 控制律 → CAN
//
// target.json 契约(follow_controller 消费):
//   有深度: {"ts","valid":true,"source":"yolo_trt","dist_m","lateral_m"(有focus才给,右为正),"off_x","box_h_norm","conf","n_persons","depth":true}
//   无深度: {... 无 dist_m/lateral_m, 只 off_x+box_h_norm, "depth":false}  ← 控制器退框高代理
//   无人:   {"ts","valid":false,"source":"yolo_trt","n_persons":0}
//
// 用法: ./yolo_follow --engine yolov5s.engine --grab-dir <grab> --runtime <runtime> --hz 10 [--focus <px>] [--once]
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdio>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <cuda_runtime_api.h>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "yololayer.h"
#include "utils.h"

using namespace nvinfer1;

#define DEVICE 0
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;
const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";
static const int PERSON_CLASS = 0;   // COCO: 0 = person
static Logger gLogger;

static void doInference(IExecutionContext& context, cudaStream_t& stream, void** buffers,
                        float* input, float* output) {
    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, 3 * INPUT_H * INPUT_W * sizeof(float),
                               cudaMemcpyHostToDevice, stream));
    context.enqueue(1, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], OUTPUT_SIZE * sizeof(float),
                               cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

static std::string argval(int argc, char** argv, const std::string& k, const std::string& d) {
    for (int i = 1; i < argc - 1; i++) if (k == argv[i]) return argv[i + 1];
    return d;
}
static bool hasarg(int argc, char** argv, const std::string& k) {
    for (int i = 1; i < argc; i++) if (k == argv[i]) return true;
    return false;
}
static double now_s() {
    return std::chrono::duration_cast<std::chrono::duration<double> >(
               std::chrono::system_clock::now().time_since_epoch()).count();
}
static double file_mtime(const std::string& p) {
    struct stat st;
    if (stat(p.c_str(), &st) != 0) return -1;
    return (double)st.st_mtime;
}
// 从 camera_status.json 抠出 "focus": 数值(grabber 会写, 见下方对 grabber 的增补); 没有返回 -1
static double read_focus(const std::string& path) {
    std::ifstream f(path.c_str());
    if (!f.good()) return -1;
    std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    size_t p = s.find("\"focus\"");
    if (p == std::string::npos) return -1;
    p = s.find(':', p);
    if (p == std::string::npos) return -1;
    return atof(s.c_str() + p + 1);
}
// 框中心区域(中间 50%)的有效深度中位数(mm); 没有有效像素返回 0
static double sample_depth_mm(const cv::Mat& depth, const cv::Rect& box) {
    if (depth.empty()) return 0;
    cv::Rect roi(box.x + box.width / 4, box.y + box.height / 4,
                 std::max(1, box.width / 2), std::max(1, box.height / 2));
    roi &= cv::Rect(0, 0, depth.cols, depth.rows);
    if (roi.width <= 0 || roi.height <= 0) return 0;
    std::vector<unsigned short> vals;
    for (int y = roi.y; y < roi.y + roi.height; y++) {
        const unsigned short* row = depth.ptr<unsigned short>(y);
        for (int x = roi.x; x < roi.x + roi.width; x++)
            if (row[x] > 0) vals.push_back(row[x]);
    }
    if (vals.empty()) return 0;
    std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
    return (double)vals[vals.size() / 2];
}
static void write_atomic(const std::string& runtime, const std::string& json) {
    std::string tmp = runtime + "/target.json.tmp";
    std::ofstream o(tmp.c_str());
    o << json;
    o.close();
    std::rename(tmp.c_str(), (runtime + "/target.json").c_str());
}

int main(int argc, char** argv) {
    std::string engine_name = argval(argc, argv, "--engine", "yolov5s.engine");
    std::string grab_dir    = argval(argc, argv, "--grab-dir", "/apollo/follow_data/runtime/grab");
    std::string runtime     = argval(argc, argv, "--runtime", "/apollo/follow_data/runtime");
    double hz          = atof(argval(argc, argv, "--hz", "10").c_str());
    double conf        = atof(argval(argc, argv, "--conf", "0.5").c_str());
    double focus_cli   = atof(argval(argc, argv, "--focus", "-1").c_str());
    double max_age     = atof(argval(argc, argv, "--max-age", "1.0").c_str());
    double lateral_sign= atof(argval(argc, argv, "--lateral-sign", "1.0").c_str());
    bool once = hasarg(argc, argv, "--once");

    cudaSetDevice(DEVICE);
    std::system(("mkdir -p '" + runtime + "'").c_str());

    // ---- 反序列化 engine ----
    std::ifstream file(engine_name.c_str(), std::ios::binary);
    if (!file.good()) { std::cerr << "open engine fail: " << engine_name << std::endl; return 1; }
    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0, file.beg);
    std::vector<char> buf(size);
    file.read(buf.data(), size);
    file.close();

    IRuntime* rt = createInferRuntime(gLogger);
    ICudaEngine* engine = rt->deserializeCudaEngine(buf.data(), size);
    if (!engine) { std::cerr << "deserialize engine fail" << std::endl; return 1; }
    IExecutionContext* context = engine->createExecutionContext();
    void* buffers[2];
    int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], 3 * INPUT_H * INPUT_W * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], OUTPUT_SIZE * sizeof(float)));
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));
    static float data[3 * INPUT_H * INPUT_W];
    static float prob[OUTPUT_SIZE];

    const std::string left   = grab_dir + "/left_latest.ppm";
    const std::string depthp = grab_dir + "/depth_latest.pgm";
    const std::string status = grab_dir + "/camera_status.json";
    double period = 1.0 / (hz > 0 ? hz : 10.0);

    std::cerr << "[yolo_follow] engine=" << engine_name << " grab=" << grab_dir
              << " -> " << runtime << "/target.json @ " << hz << "Hz (conf=" << conf << ")" << std::endl;

    while (true) {
        double t0 = now_s();
        std::ostringstream js;
        js.setf(std::ios::fixed);

        double mt = file_mtime(left);
        bool fresh = (mt > 0) && (now_s() - mt <= max_age);
        cv::Mat img;
        if (fresh) img = cv::imread(left);   // 默认彩色, PPM 的 RGB 自动转 BGR

        if (img.empty()) {
            js << std::setprecision(3)
               << "{\"ts\":" << now_s() << ",\"valid\":false,\"source\":\"yolo_trt\",\"reason\":\"no_frame\"}";
        } else {
            int W = img.cols, H = img.rows;
            cv::Mat pr = preprocess_img(img, INPUT_W, INPUT_H);
            int i = 0;
            for (int row = 0; row < INPUT_H; ++row) {
                uchar* uc = pr.data + row * pr.step;
                for (int col = 0; col < INPUT_W; ++col) {
                    data[i]                       = uc[2] / 255.0f;
                    data[i + INPUT_H * INPUT_W]   = uc[1] / 255.0f;
                    data[i + 2 * INPUT_H * INPUT_W] = uc[0] / 255.0f;
                    uc += 3; ++i;
                }
            }
            doInference(*context, stream, buffers, data, prob);
            std::vector<Yolo::Detection> res;
            nms(res, prob, (float)conf, 0.4f);

            std::vector<Yolo::Detection*> persons;
            for (size_t k = 0; k < res.size(); k++)
                if ((int)res[k].class_id == PERSON_CLASS) persons.push_back(&res[k]);

            if (persons.empty()) {
                js << std::setprecision(3)
                   << "{\"ts\":" << now_s() << ",\"valid\":false,\"source\":\"yolo_trt\",\"n_persons\":0}";
            } else {
                // 挑最大框 = 目标(最显著的人, 与 Phase2 仿真一致)
                Yolo::Detection* best = NULL;
                cv::Rect bestr;
                double bestArea = -1;
                for (size_t k = 0; k < persons.size(); k++) {
                    cv::Rect r = get_rect(img, persons[k]->bbox);
                    double a = (double)r.width * r.height;
                    if (a > bestArea) { bestArea = a; best = persons[k]; bestr = r; }
                }
                bestr &= cv::Rect(0, 0, W, H);
                double u = bestr.x + bestr.width / 2.0;
                double off_x = (u - W / 2.0) / W;          // -0.5..0.5, 右为正
                double box_h_norm = (double)bestr.height / H;

                cv::Mat depth;
                double dmt = file_mtime(depthp);
                if (dmt > 0 && now_s() - dmt <= max_age)
                    depth = cv::imread(depthp, CV_LOAD_IMAGE_ANYDEPTH);  // CV_16U, mm
                double dmm = sample_depth_mm(depth, bestr);
                double focus = focus_cli > 0 ? focus_cli : read_focus(status);

                js << std::setprecision(3) << "{\"ts\":" << now_s()
                   << ",\"valid\":true,\"source\":\"yolo_trt\""
                   << std::setprecision(4)
                   << ",\"off_x\":" << off_x << ",\"box_h_norm\":" << box_h_norm
                   << ",\"conf\":" << best->conf
                   << ",\"n_persons\":" << (int)persons.size()
                   << ",\"bbox\":[" << bestr.x << "," << bestr.y << "," << bestr.width << "," << bestr.height << "]"
                   << ",\"img_w\":" << W << ",\"img_h\":" << H;
                if (dmm > 0) {
                    double dist_m = dmm / 1000.0;
                    js << std::setprecision(3) << ",\"dist_m\":" << dist_m;
                    if (focus > 0) {
                        double lateral_m = lateral_sign * (u - W / 2.0) * dist_m / focus;
                        js << ",\"lateral_m\":" << lateral_m;
                    }
                    js << ",\"depth\":true";
                } else {
                    js << ",\"depth\":false";
                }
                js << "}";
            }
        }

        write_atomic(runtime, js.str());
        if (once) { std::cout << js.str() << std::endl; break; }
        double dt = now_s() - t0;
        if (dt < period) std::this_thread::sleep_for(std::chrono::duration<double>(period - dt));
    }

    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(buffers[inputIndex]));
    CUDA_CHECK(cudaFree(buffers[outputIndex]));
    context->destroy();
    engine->destroy();
    rt->destroy();
    return 0;
}
