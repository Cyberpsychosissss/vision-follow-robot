// zkhy_grabber.cpp — 我们自己的 ZKHY 立体相机抓取器 (基于 fr07 验证过的版本)。
// 不含 ROS (这台车 roscpp C++ advertise 会崩)。用厂商 SDK 直读, 写文件供下游 Python 读。
// 相对 fr07 的增量: 启动加载立体标定参数, 在 Disparity 分支算【每像素真实 Z 距离(米/mm)】,
//   写 depth_latest.pgm (16bit, 单位 mm)。其余输出(left/right ppm, obstacles_latest.json,
//   disparity_latest.pgm 归一化, camera_status.json) 与 fr07 兼容。
//
// 编译(在 apollo_dev_nvidia 容器内, 见 fr07 README 配方):
//   g++ -std=c++11 -I/apollo/modules/drivers/zkhy/src/inc \
//     -L/apollo/follow_data/lib -L/apollo/modules/drivers/zkhy/src/Bin \
//     -Wl,-rpath,/apollo/follow_data/lib -Wl,-rpath,/apollo/modules/drivers/zkhy/src/Bin \
//     -Wl,-rpath-link,/apollo/follow_data/lib -Wl,-rpath-link,/apollo/modules/drivers/zkhy/src/Bin \
//     -o zkhy_grabber zkhy_grabber.cpp \
//     -lStereoCamera -lImageUtils -lboost_system -pthread /apollo/follow_data/lib/libstdc++.so.6
// 运行(相机需空, 现被 fr07 grabber 占):
//   export LD_LIBRARY_PATH=/apollo/follow_data/lib:/apollo/modules/drivers/zkhy/src/Bin:$LD_LIBRARY_PATH
//   ./zkhy_grabber --ip 192.168.1.251 --out-dir /apollo/follow_data/runtime/grab --duration 0 --write-fps 5
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "camerahandler.h"
#include "calibrationparams.h"
#include "disparityconvertor.h"
#include "frameid.h"
#include "frameformat.h"
#include "obstacleData.h"
#include "satpext.h"
#include "stereocamera.h"
#include "taskiddef.h"

namespace {

int clamp_int(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }

void yuyv_to_rgb(const uint8_t* src, int width, int height, uint8_t* dst) {
  const int pixels = width * height;
  for (int i = 0, j = 0; i + 3 < pixels * 2; i += 4) {
    const int y0 = src[i + 0], u = src[i + 1] - 128, y1 = src[i + 2], v = src[i + 3] - 128;
    dst[j++] = clamp_int(y0 + (int)(1.402 * v));
    dst[j++] = clamp_int(y0 - (int)(0.344136 * u + 0.714136 * v));
    dst[j++] = clamp_int(y0 + (int)(1.772 * u));
    dst[j++] = clamp_int(y1 + (int)(1.402 * v));
    dst[j++] = clamp_int(y1 - (int)(0.344136 * u + 0.714136 * v));
    dst[j++] = clamp_int(y1 + (int)(1.772 * u));
  }
}

bool write_all(FILE* fp, const void* data, size_t bytes) {
  return fwrite(data, 1, bytes, fp) == bytes;
}

bool finish_atomic(FILE* fp, bool ok, const std::string& tmp, const std::string& path) {
  fclose(fp);
  if (!ok) { std::remove(tmp.c_str()); return false; }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) { std::remove(tmp.c_str()); return false; }
  return true;
}

bool write_ppm_atomic(const std::string& path, const uint8_t* rgb, int w, int h) {
  const std::string tmp = path + ".tmp";
  FILE* fp = fopen(tmp.c_str(), "wb");
  if (!fp) return false;
  char hdr[128];
  int n = snprintf(hdr, sizeof(hdr), "P6\n%d %d\n255\n", w, h);
  bool ok = write_all(fp, hdr, n) && write_all(fp, rgb, (size_t)w * h * 3);
  return finish_atomic(fp, ok, tmp, path);
}

bool write_pgm_atomic(const std::string& path, const uint8_t* gray, int w, int h) {
  const std::string tmp = path + ".tmp";
  FILE* fp = fopen(tmp.c_str(), "wb");
  if (!fp) return false;
  char hdr[128];
  int n = snprintf(hdr, sizeof(hdr), "P5\n%d %d\n255\n", w, h);
  bool ok = write_all(fp, hdr, n) && write_all(fp, gray, (size_t)w * h);
  return finish_atomic(fp, ok, tmp, path);
}

// ++ 16bit PGM (P5, maxval 65535, 大端2字节/像素) —— 存真实深度(mm)
bool write_pgm16_atomic(const std::string& path, const uint16_t* gray, int w, int h) {
  const std::string tmp = path + ".tmp";
  FILE* fp = fopen(tmp.c_str(), "wb");
  if (!fp) return false;
  char hdr[128];
  int n = snprintf(hdr, sizeof(hdr), "P5\n%d %d\n65535\n", w, h);
  std::vector<uint8_t> buf((size_t)w * h * 2);
  for (size_t i = 0; i < (size_t)w * h; ++i) { buf[2 * i] = gray[i] >> 8; buf[2 * i + 1] = gray[i] & 0xFF; }
  bool ok = write_all(fp, hdr, n) && write_all(fp, buf.data(), buf.size());
  return finish_atomic(fp, ok, tmp, path);
}

const char* typeName(int t) {
  switch (t) {
    case INVALID: return "INVALID"; case VEHICLE: return "VEHICLE"; case PEDESTRIAN: return "PEDESTRIAN";
    case CHILD: return "CHILD"; case BICYCLE: return "BICYCLE"; case MOTO: return "MOTO";
    case TRUCK: return "TRUCK"; case BUS: return "BUS"; case OTHERS: return "OTHERS";
    case ESTIMATED: return "ESTIMATED"; case CONTINUOUS: return "CONTINUOUS"; default: return "UNKNOWN";
  }
}

class DumpHandler : public CameraHandler {
 public:
  DumpHandler(std::string out_dir, double write_fps)
      : out_dir_(std::move(out_dir)),
        min_interval_ms_(write_fps > 0.0 ? (int64_t)(1000.0 / write_fps) : 0),
        started_(std::chrono::steady_clock::now()),
        last_left_(started_), last_disp_(started_) {}

  // ++ 加载标定参数 -> 使能真实米深度
  void setCalib(const StereoCalibrationParameters& p) { calib_ = p; calib_ready_ = true; }

  void handleRawFrame(const RawImageFrame* f) override {
    if (!f || f->width <= 0 || f->height <= 0) return;
    const uint8_t* img = (const uint8_t*)f + sizeof(RawImageFrame);
    const auto now = std::chrono::steady_clock::now();
    if (f->frameId == FrameId::Obstacle) { handleObstacle(img, f->dataSize, now); return; }
    if (f->frameId == FrameId::LeftCamera) { handleLeft(img, f->width, f->height, f->format, now); return; }
    if (f->frameId == FrameId::Disparity) { handleDisparity(img, f->width, f->height, f->format, now); return; }
  }

  void handleLeft(const uint8_t* image, int w, int h, int fmt,
                  std::chrono::steady_clock::time_point now) {
    left_seen_++;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_left_).count();
    if (min_interval_ms_ > 0 && ms < min_interval_ms_) return;
    last_left_ = now;
    rgb_.resize((size_t)w * h * 3);
    yuyv_to_rgb(image, w, h, rgb_.data());
    if (write_ppm_atomic(out_dir_ + "/left_latest.ppm", rgb_.data(), w, h)) {
      write_ppm_atomic(out_dir_ + "/latest.ppm", rgb_.data(), w, h);
      left_written_++;
    }
    last_w_ = w; last_h_ = h; last_fmt_ = fmt;
    write_status(now);
  }

  void handleDisparity(const uint8_t* image, int w, int h, int fmt,
                       std::chrono::steady_clock::time_point now) {
    disp_seen_++;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_disp_).count();
    if (min_interval_ms_ > 0 && ms < min_interval_ms_) return;
    last_disp_ = now;
    const int bitNum = DisparityConvertor::getDisparityBitNum(fmt);

    // (a) 归一化 8bit 视差灰度图 (同 fr07, 便于可视化)
    gray_.assign((size_t)w * h, 0);
    if (bitNum >= 0) {
      dispf_.resize((size_t)w * h);
      DisparityConvertor::convertDisparity2FloatFormat(image, w, h, bitNum, dispf_.data());
      float mx = 1.0f;
      for (float v : dispf_) if (v > mx && v < 1000.0f) mx = v;
      for (size_t i = 0; i < dispf_.size(); ++i) {
        float v = dispf_[i];
        if (v < 0.0f || v > 1000.0f) v = 0.0f;
        gray_[i] = (uint8_t)clamp_int((int)(255.0f * v / mx));
      }
    } else {
      for (size_t i = 0; i < (size_t)w * h; ++i) gray_[i] = image[i];
    }
    write_pgm_atomic(out_dir_ + "/disparity_latest.pgm", gray_.data(), w, h);

    // ++ (b) 真实米深度: 直接 Z = focus*baseline/disparity, 绕开 SDK 查找表(它对 focus 不响应).
    //    dispf_ 上面已算好(浮点视差, 单位 px)。focus(px)*Tx(mm)/d(px) = Z(mm)。
    //    focus*baseline 这个乘积由 disp_calib.py 用相机自报距离经验标定。
    if (calib_ready_ && bitNum >= 0) {
      const double fb = (double)calib_.focus * (double)calib_.Tx;   // px*mm
      depth16_.resize((size_t)w * h);
      for (size_t i = 0; i < (size_t)w * h; ++i) {
        float d = dispf_[i];                                        // px (上面 convertDisparity2FloatFormat 得到)
        float z = (d > 0.1f && d < 1000.0f) ? (float)(fb / d) : 0.0f;  // mm
        depth16_[i] = (z > 0.0f && z < 65535.0f) ? (uint16_t)z : 0;
      }
      write_pgm16_atomic(out_dir_ + "/depth_latest.pgm", depth16_.data(), w, h);
      depth_written_++;
    }
    write_status(now);
  }

  void handleObstacle(const uint8_t* image, uint32_t data_size,
                      std::chrono::steady_clock::time_point now) {
    obs_seen_++;
    if (data_size < 8) return;
    const int block_num = ((const int*)image)[0];
    const OutputObstacles* obs = (const OutputObstacles*)((const int*)image + 2);
    const int max_blocks = (int)((data_size - 8) / sizeof(OutputObstacles));
    const int count = block_num < max_blocks ? block_num : max_blocks;
    const std::string tmp = out_dir_ + "/obstacles_latest.json.tmp";
    FILE* fp = fopen(tmp.c_str(), "w");
    if (!fp) return;
    const double el = std::chrono::duration<double>(now - started_).count();
    fprintf(fp, "{\n  \"timestamp_s\": %.3f,\n  \"count\": %d,\n  \"obstacles\": [\n", el, count);
    for (int i = 0; i < count; ++i) {
      const OutputObstacles& o = obs[i];
      const int x1 = std::min(std::min<int>(o.firstPointX, o.secondPointX), std::min<int>(o.thirdPointX, o.fourthPointX));
      const int y1 = std::min(std::min<int>(o.firstPointY, o.secondPointY), std::min<int>(o.thirdPointY, o.fourthPointY));
      const int x2 = std::max(std::max<int>(o.firstPointX, o.secondPointX), std::max<int>(o.thirdPointX, o.fourthPointX));
      const int y2 = std::max(std::max<int>(o.firstPointY, o.secondPointY), std::max<int>(o.thirdPointY, o.fourthPointY));
      fprintf(fp, "    {\"track_id\": %u, \"type\": \"%s\", \"type_id\": %d, \"class_label\": %u, "
                  "\"state_label\": %u, \"distance_m\": %.3f, \"near_distance_m\": %.3f, "
                  "\"far_distance_m\": %.3f, \"center_x_m\": %.3f, \"bbox\": [%d, %d, %d, %d], "
                  "\"frame_rate\": %.3f}%s\n",
              (unsigned)o.trackId, typeName(o.obstacleType), (int)o.obstacleType, (unsigned)o.classLabel,
              (unsigned)o.stateLabel, o.avgDistanceZ, o.nearDistanceZ, o.farDistanceZ, o.real3DCenterX,
              x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1), o.frameRate, i + 1 == count ? "" : ",");
    }
    fprintf(fp, "  ]\n}\n");
    fclose(fp);
    if (std::rename(tmp.c_str(), (out_dir_ + "/obstacles_latest.json").c_str()) == 0) obs_written_++;
    write_status(now);
  }

  void write_status(std::chrono::steady_clock::time_point now) {
    const std::string tmp = out_dir_ + "/camera_status.json.tmp";
    FILE* fp = fopen(tmp.c_str(), "w");
    if (!fp) return;
    const double el = std::chrono::duration<double>(now - started_).count();
    auto fps = [&](uint64_t n) { return el > 0 ? n / el : 0.0; };
    fprintf(fp, "{\n  \"left_frames_written\": %llu,\n  \"disparity_frames_written\": %llu,\n"
                "  \"depth_frames_written\": %llu,\n  \"obstacle_frames_written\": %llu,\n"
                "  \"width\": %d,\n  \"height\": %d,\n  \"calib_ready\": %s,\n"
                "  \"focus\": %.4f,\n  \"baseline_mm\": %.4f,\n  \"elapsed_s\": %.3f,\n"
                "  \"left_fps\": %.3f,\n  \"disparity_fps\": %.3f,\n  \"depth_fps\": %.3f,\n  \"obstacle_fps\": %.3f\n}\n",
            (unsigned long long)left_written_.load(), (unsigned long long)disp_seen_.load(),
            (unsigned long long)depth_written_.load(), (unsigned long long)obs_written_.load(),
            last_w_, last_h_, calib_ready_ ? "true" : "false",
            (calib_ready_ ? (double)calib_.focus : 0.0), (calib_ready_ ? (double)calib_.Tx : 0.0), el,
            fps(left_seen_.load()), fps(disp_seen_.load()), fps(depth_written_.load()), fps(obs_seen_.load()));
    fclose(fp);
    std::rename(tmp.c_str(), (out_dir_ + "/camera_status.json").c_str());
  }

  uint64_t left_written() const { return left_written_.load(); }
  uint64_t depth_written() const { return depth_written_.load(); }
  uint64_t obs_written() const { return obs_written_.load(); }

 private:
  std::string out_dir_;
  int64_t min_interval_ms_;
  std::chrono::steady_clock::time_point started_, last_left_, last_disp_;
  std::atomic<uint64_t> left_seen_{0}, left_written_{0}, disp_seen_{0}, depth_written_{0}, obs_seen_{0}, obs_written_{0};
  int last_w_{0}, last_h_{0}, last_fmt_{0};
  std::vector<uint8_t> rgb_, gray_;
  std::vector<float> dispf_, depthf_, lutZ_;
  std::vector<uint16_t> depth16_;
  StereoCalibrationParameters calib_;
  bool calib_ready_{false}, lut_gen_{false};
  static const int kDispCount_ = 81;
};

std::string arg_value(int argc, char** argv, const std::string& name, const std::string& def) {
  for (int i = 1; i + 1 < argc; ++i) if (argv[i] == name) return argv[i + 1];
  return def;
}
double arg_double(int argc, char** argv, const std::string& name, double def) {
  std::string v = arg_value(argc, argv, name, ""); return v.empty() ? def : std::atof(v.c_str());
}
int arg_int(int argc, char** argv, const std::string& name, int def) {
  std::string v = arg_value(argc, argv, name, ""); return v.empty() ? def : std::atoi(v.c_str());
}

}  // namespace

int main(int argc, char** argv) {
  const std::string ip = arg_value(argc, argv, "--ip", "192.168.1.251");
  const std::string out_dir = arg_value(argc, argv, "--out-dir", "/apollo/follow_data/runtime/grab");
  const int duration_s = arg_int(argc, argv, "--duration", 0);
  const double write_fps = arg_double(argc, argv, "--write-fps", 5.0);
  // 经验标定默认值: focus=1065px/baseline=120mm, 由 disp_calib.py 从相机自报障碍物距离反推
  // (SDK 标定接口在本相机失败; 深度只看 focus*baseline 乘积; 重标定见 bin/disp_calib.py)。
  const double cli_focus = arg_double(argc, argv, "--focus", 1065.0);
  const double cli_baseline = arg_double(argc, argv, "--baseline", 120.0);
  std::system(("mkdir -p '" + out_dir + "'").c_str());

  std::cout << "connecting camera " << ip << std::endl;
  StereoCamera* camera = StereoCamera::connect(ip.c_str());
  if (!camera) { std::cerr << "connect failed" << std::endl; return 2; }

  DumpHandler handler(out_dir, write_fps);
  camera->enableTasks(TaskId::ObstacleTask | TaskId::DisplayTask);
  camera->requestFrame(&handler, FrameId::LeftCamera | FrameId::Disparity | FrameId::Obstacle);

  // ++ 标定 -> 真实米深度. 本相机 SDK 不吐标定参数, 故优先用手动 --focus/--baseline
  //    (深度 Z = focus*baseline/disparity, 只有 f*b 乘积决定深度尺度, 用已知距离经验标定即可);
  //    没给手动值则尝试 SDK 并在循环里重试.
  bool calib_done = false;
  if (cli_focus > 0 && cli_baseline > 0) {
    StereoCalibrationParameters params{};
    params.focus = cli_focus;
    params.Tx = cli_baseline;
    handler.setCalib(params);
    calib_done = true;
    std::cout << "manual calib: focus=" << cli_focus << "px baseline(Tx)=" << cli_baseline
              << "mm (f*b=" << (cli_focus * cli_baseline) << ") -> metric depth from disparity" << std::endl;
  } else {
    StereoCalibrationParameters params{};
    if (camera->requestStereoCameraParameters(params)) {
      handler.setCalib(params);
      calib_done = true;
      std::cout << "calib loaded(SDK): focus=" << params.focus << " Tx=" << params.Tx
                << " -> metric depth enabled" << std::endl;
    } else {
      std::cout << "SDK calib not ready -> retry in loop (or pass --focus/--baseline)" << std::endl;
    }
  }

  const auto t0 = std::chrono::steady_clock::now();
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!calib_done) {
      StereoCalibrationParameters params;
      if (camera->requestStereoCameraParameters(params)) {
        handler.setCalib(params);
        calib_done = true;
        std::cout << "calib loaded(retry): focus=" << params.focus << " Tx=" << params.Tx
                  << " -> metric depth enabled" << std::endl;
      }
    }
    const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "elapsed=" << el << " left=" << handler.left_written()
              << " depth=" << handler.depth_written() << " obs=" << handler.obs_written() << std::endl;
    if (duration_s > 0 && el >= duration_s) break;
  }
  const int rc = handler.left_written() > 0 ? 0 : 3;
  std::cout << "done left=" << handler.left_written() << " depth=" << handler.depth_written()
            << " obs=" << handler.obs_written() << " rc=" << rc << std::endl;
  std::cout.flush();
  std::_Exit(rc);   // SDK 断连会崩, 数据已落盘, 直接退出
}
