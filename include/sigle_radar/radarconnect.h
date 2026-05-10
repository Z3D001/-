#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include <thread>
#include <mutex>
#include <chrono>
#include <cmath>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>

struct data_format_t {
    uint32_t DataFlag = 0;
    uint8_t WaveType = 0;
    uint8_t WaveType2 = 0;
    uint8_t AngleResType = 0;
};

struct ladar_data_t {
    int valid = 0;
    float angle_min = 0.0f;
    float angle_max = 0.0f;
    float angle_increment = 0.0f;
    float scan_time = 0.0f;
    float time_increment = 0.0f;
    float range_min = 0.0f;
    float range_max = 0.0f;
    uint8_t wave_count = 1;
    std::vector<float> ranges;
    std::vector<float> intensities;
};

struct frame_list_t
{
    frame_list_t* next;
    const uint8_t* frame;
    int length;
};

struct ladar_t
{
    uint64_t timer_;

    bool dataMerged_;
    const uint8_t* dataFrame_;
    int32_t dataFrameLength_;
    int32_t dataFrameCount_;

    uint32_t param_flags_;
    data_format_t data_format_;
    float radius_;
    float ticks_;
    float angle_start_;
    float angle_stop_;
    float angle_step_;
    int32_t wave_count_;
    int32_t shift_count_;
    int32_t point_count_;

    frame_list_t* frameHead;
    frame_list_t frames[1024];
    int32_t frameCount;

    ladar_data_t data_;

    const uint8_t* begin_;
    int32_t offset_;
    int32_t length_;
    uint8_t buffer_[4];

    int status;
};

struct QPointF {
    float x = 0.0f;
    float y = 0.0f;
    QPointF(float x_ = 0.0f, float y_ = 0.0f) : x(x_), y(y_) {}
};

constexpr uint32_t dsParam   = 1 << 0;
constexpr uint32_t dsDataFmt = 1 << 1;
constexpr uint32_t dsLogin   = 1 << 2;

class RadarConnect {
public:
    using RevRadarCallback = std::function<void(const std::string&, void*, int64_t,
                                            const std::vector<float>& ranges,
                                            const std::vector<float>& ys,
                                            const std::vector<float>& intensities)>;

    RadarConnect(const std::string& mark, const std::string& addr, int port,
                 void* priv, RevRadarCallback callback);

    ~RadarConnect();

    void startConnect();
    #define DIFF_TIME_CACHE_NUM 50
private:
    void receiverLoop();
    void parseIncomingFrames();
    void ladarHandleFrame(const uint8_t *frame, int32_t frameLength);

    void ladarHandleBaseParam(const uint8_t* frame, int32_t frameLength);
    void ladarHandleBaseParamHS(const uint8_t *frame, int32_t frameLength);
    void ladarHandleDataFormat(const uint8_t *frame, int32_t frameLength);

    static inline float mdeg_to_rad(double x);
    static int ladarGetFormatWaveCount(data_format_t format);

    int sendLoginCmd();
    int sendHeartBeatCmd();
    int sendGetInfoCmd();
    int sendGetDataFormatCmd();

    int getDiffTime(int diffTime);

    int ladarParseData(const uint8_t *frame, int frameLength);

    int sock_fd_ = -1;
    std::string mMark, mAddr;
    int mPort;
    void* mPriv;
    RevRadarCallback revCallback;

    bool running_ = true;
    std::thread receiver_thread_;
    std::vector<uint8_t> rev_cache_;

    ladar_t ladar_{};
    ladar_data_t data_{};
    std::vector<QPointF> points_;
    std::vector<int> diffTimeCache_;
    int diffTimeCacheIndex_ = 0;
    int64_t revtimeStamp_ = 0;

    float mHeight_ = 0.0f;
    bool isEnableLimitX_ = false;
    float startX_ = 0.0f, endX_ = 0.0f;
    bool isEnableReadFile_ = false;

    std::thread heartbeat_thread_;
    bool heartbeat_running_ = true;
};
