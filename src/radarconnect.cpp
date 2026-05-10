#include "sigle_radar/radarconnect.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

static const int LadarFrameMinLength = 40;
static const int LadarFrameMaxLength = 64 * 1024;
static const double pi = 3.14159265358979323846;

RadarConnect::RadarConnect(const std::string& mark, const std::string& addr, int port,
                           void* priv, RevRadarCallback callback)
    : mMark(mark), mAddr(addr), mPort(port), mPriv(priv), revCallback(callback)
{
    diffTimeCache_.resize(DIFF_TIME_CACHE_NUM, 0);
    ladar_.dataMerged_ = false;
    ladar_.param_flags_ = 0;
    ladar_.wave_count_ = 1;
}

RadarConnect::~RadarConnect() {
    heartbeat_running_ = false;
    if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
    running_ = false;
    if (receiver_thread_.joinable()) receiver_thread_.join();
    if (sock_fd_ > 0) close(sock_fd_);
}

void RadarConnect::startConnect() {
    if (sock_fd_ > 0) close(sock_fd_);

    sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
        std::cerr << "[Radar] 创建 socket 失败" << std::endl;
        return;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(mPort);
    inet_pton(AF_INET, mAddr.c_str(), &server_addr.sin_addr);

    if (connect(sock_fd_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "[Radar] 连接失败 " << mAddr << ":" << mPort << std::endl;
        close(sock_fd_);
        sock_fd_ = -1;
        return;
    }

    std::cout << "[Radar] 成功连接 " << mAddr << ":" << mPort << std::endl;
    sendLoginCmd();

    if (!receiver_thread_.joinable()) {
        receiver_thread_ = std::thread(&RadarConnect::receiverLoop, this);
    }

    // 心跳线程（每8秒发送一次心跳）
    if (!heartbeat_thread_.joinable()) {
        heartbeat_thread_ = std::thread([this]() {
            while (heartbeat_running_) {
                std::this_thread::sleep_for(std::chrono::seconds(8));
                if (sock_fd_ > 0) {
                    sendHeartBeatCmd();
                }
            }
        });
    }
}

void RadarConnect::receiverLoop() {
    std::vector<uint8_t> buffer(65536);
    while (running_) {
        if (sock_fd_ <= 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            startConnect();
            continue;
        }
        int n = recv(sock_fd_, buffer.data(), buffer.size(), 0);
        if (n <= 0) {
            std::cerr << "[Radar] 连接断开" << std::endl;
            close(sock_fd_);
            sock_fd_ = -1;
            continue;
        }
        rev_cache_.insert(rev_cache_.end(), buffer.begin(), buffer.begin() + n);
        parseIncomingFrames();
    }
}

void RadarConnect::parseIncomingFrames() {
    if (rev_cache_.size() < LadarFrameMinLength) return;

    const uint8_t* begin = rev_cache_.data();
    const uint8_t* end = begin + rev_cache_.size();
    const uint8_t* last = end - LadarFrameMinLength;
    const uint8_t* frame = begin;

    while (frame <= last) {
        if (*reinterpret_cast<const uint16_t*>(frame) == 0x49ad) {
            int32_t frameLength = *reinterpret_cast<const int32_t*>(frame + 16);
            if (frameLength >= LadarFrameMinLength && frameLength <= LadarFrameMaxLength &&
                frame + frameLength <= end) {
                ladarHandleFrame(frame, frameLength);
                frame += frameLength;
                continue;
            }
            break;
        }
        frame++;
    }
    int dealt = frame - begin;
    rev_cache_.erase(rev_cache_.begin(), rev_cache_.begin() + dealt);
}

inline float RadarConnect::mdeg_to_rad(double x)
{
    return static_cast<float>(x * pi / 180000.0);
}

int RadarConnect::ladarGetFormatWaveCount(data_format_t format)
{
    switch (format.WaveType)
    {
    case 0: return 1;
    case 1: return 1;
    case 2: return 2;
    case 3:
        switch (format.WaveType2)
        {
        case 0: return 1;
        case 1: return 2;
        case 2: return 2;
        default: break;
        }
        break;
    default: break;
    }
    return 0;
}

int RadarConnect::sendLoginCmd() {
    static uint8_t login_frame[] = {
        0xAD, 0x49, 0xEA, 0x0B, 0x00, 0x2F, 0x01, 0x00, 0x0E, 0x2C, 0x00, 0x00, 0x05, 0x00, 0x6D, 0xF3,
        0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB7, 0xF2, 0x57, 0x5B, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x67, 0x6A, 0x64, 0x74, 0x31, 0x64, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x67, 0x63, 0x33, 0x34, 0x33, 0x6A, 0x34, 0x36,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    return (sock_fd_ > 0) ? send(sock_fd_, login_frame, sizeof(login_frame), 0) : -1;
}

int RadarConnect::sendGetInfoCmd() {
    static uint8_t frames[] = {
        0xAD, 0x49, 0xE1, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x35, 0x00, 0x00, 0x00, 0x05, 0x00, 0xF6, 0xE3,
        0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    return (sock_fd_ > 0) ? send(sock_fd_, frames, sizeof(frames), 0) : -1;
}

int RadarConnect::sendGetDataFormatCmd() {
    static uint8_t frames[]={
        0xAD, 0x49, 0xD9, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x05, 0x00, 0xFE, 0xE3,
        0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC9, 0x71, 0xCC, 0x5E, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    return (sock_fd_ > 0) ? send(sock_fd_, frames, sizeof(frames), 0) : -1;
}

int RadarConnect::sendHeartBeatCmd() {
    static uint8_t heart_beta_frame[] = {
        0xAD, 0x49, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x05, 0x00, 0xD6, 0xFF,
        0x28, 0x00, 0x00, 0x00,	0x00, 0x00, 0x00, 0x00, 0x59, 0xD3, 0x2D, 0x5C, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    return (sock_fd_ > 0) ? send(sock_fd_, heart_beta_frame, sizeof(heart_beta_frame), 0) : -1;
}

void RadarConnect::ladarHandleBaseParam(const uint8_t* frame, int32_t frameLength)
{
    if (ladar_.param_flags_ & dsParam){
        return;
    }

#pragma pack(push, 1)
    struct BaseParamTag1
    {
        uint16_t LadarMode;
        uint16_t Range;
        uint16_t Esav;
        uint16_t AngleRes;
        int32_t AngleStart;
        int32_t AngleStop;
        uint8_t LineNum;
    };

    struct BaseParamTag2 : public BaseParamTag1
    {
        uint8_t AngleResType;
    };
#pragma pack(pop)

    uint32_t resType = 0;
    switch (frameLength)
    {
    case 40 + sizeof(BaseParamTag1):
        resType = 0;
        break;

    case 40 + sizeof(BaseParamTag2):
        resType = ((BaseParamTag2*)(frame + 40))->AngleResType;
        break;

    default:
        RCLCPP_WARN(rclcpp::get_logger("Radar"), "[Radar] BaseParam 帧长度不支持: %d", frameLength);
        return;
    }

    BaseParamTag1& param = *((BaseParamTag1*)(frame + 40));
    ladar_.radius_ = param.Range;
    ladar_.ticks_ = 360.f / param.Esav;
    ladar_.angle_start_ = param.AngleStart / 1000.f;
    ladar_.angle_stop_ = param.AngleStop / 1000.f;

    RCLCPP_INFO(rclcpp::get_logger("Radar"), "[Radar] BaseParam: Range=%.1f, Esav=%u, AngleRes=%u, resType=%u",
                ladar_.radius_, param.Esav, param.AngleRes, resType);

    switch (resType)
    {
    case 0:
        ladar_.angle_step_ = mdeg_to_rad(param.AngleRes);
        ladar_.point_count_ = 360000 / param.AngleRes;
        break;

    case 1:
        ladar_.angle_step_ = mdeg_to_rad(param.AngleRes) * 0.01f;
        ladar_.point_count_ = 36000000 / param.AngleRes;
        break;

    case 3:
        ladar_.angle_step_ = (float)(pi * 2.0 / param.AngleRes);
        ladar_.point_count_ = param.AngleRes;
        break;

    default:
        ladar_.angle_step_ = 0.f;
        ladar_.point_count_ = 0;
        RCLCPP_ERROR(rclcpp::get_logger("Radar"), "[Radar] BaseParam 不支持的分辨率类型: %u", resType);
        return;
    }

    ladar_.param_flags_ |= dsParam;
    RCLCPP_INFO(rclcpp::get_logger("Radar"), "[Radar] BaseParam 初始化完成，param_flags_ = 0x%08X", ladar_.param_flags_);
}

void RadarConnect::ladarHandleBaseParamHS( const uint8_t* frame, int32_t frameLength)
{
    if (ladar_.param_flags_ & dsParam)
        return;

#pragma pack(push, 1)
    struct BaseParamHSTag
    {
        uint16_t LadarMode;
        uint16_t Range;
        uint32_t Esav;
        uint16_t Reserved;
        uint16_t AngleRes;
        int32_t AngleStart;
        int32_t AngleStop;
        uint8_t LineNum;
        uint8_t AngleResType;
    };
#pragma pack(pop)

    BaseParamHSTag& param = *((BaseParamHSTag*)(frame + 40));
    ladar_.radius_ = param.Range;
    ladar_.ticks_ = 360.f / param.Esav;
    ladar_.angle_start_ = param.AngleStart / 1000.f;
    ladar_.angle_stop_ = param.AngleStop / 1000.f;
    switch (param.AngleResType)
    {
    case 0:
        ladar_.angle_step_ = mdeg_to_rad(param.AngleRes);
        ladar_.point_count_ = 360000 / param.AngleRes;
        break;

    case 1:
        ladar_.angle_step_ = mdeg_to_rad(param.AngleRes) * 0.01f;
        ladar_.point_count_ = 36000000 / param.AngleRes;
        break;

    case 3:
        ladar_.angle_step_ = (float)(pi * 2.0 / param.AngleRes);
        ladar_.point_count_ = param.AngleRes;
        break;

    default:
        ladar_.angle_step_ = 0.f;
        ladar_.point_count_ = 0;
        return;
    }

    ladar_.param_flags_ |= dsParam;
}

void RadarConnect::ladarHandleDataFormat(const uint8_t* frame, int32_t frameLength)
{
    if (ladar_.param_flags_ & dsDataFmt) {
        return;
    }

#pragma pack(push, 1)
    struct DataFmt
    {
        data_format_t DataFormat;
        int32_t AngleStart;
        int32_t AngleStop;
        uint16_t AngleRes;
        uint16_t LadarMode;
        uint8_t LineNum;
        uint8_t PackSum;
        uint8_t PackNo;
        uint8_t ShiftIndex : 4;
        uint8_t ShiftCount : 4;
    };
#pragma pack(pop)

    DataFmt& fmt = *((DataFmt*)(frame + 40));

    RCLCPP_INFO(rclcpp::get_logger("Radar"), "[Radar] DataFormat: AngleResType=%d, AngleRes=%u, WaveType=%d",
                fmt.DataFormat.AngleResType, fmt.AngleRes, fmt.DataFormat.WaveType);

    ladar_.data_format_.DataFlag = fmt.DataFormat.DataFlag;
    ladar_.wave_count_ = ladarGetFormatWaveCount(fmt.DataFormat);
    ladar_.shift_count_ = fmt.ShiftCount & 0x0F;
    ladar_.angle_start_ = mdeg_to_rad(fmt.AngleStart);
    ladar_.angle_stop_ = mdeg_to_rad(fmt.AngleStop);

    switch (fmt.DataFormat.AngleResType)
    {
    case 0:
        ladar_.angle_step_ = mdeg_to_rad(fmt.AngleRes);
        ladar_.point_count_ = 360000 / fmt.AngleRes;
        break;
    case 1:
        ladar_.angle_step_ = mdeg_to_rad(fmt.AngleRes * 100) * 0.01f;
        ladar_.point_count_ = 36000000 / fmt.AngleRes;
        break;
    case 3:
        ladar_.angle_step_ = (float)(pi * 2.0 / fmt.AngleRes);
        ladar_.point_count_ = fmt.AngleRes;
        break;
    default:
        RCLCPP_WARN(rclcpp::get_logger("Radar"), "[Radar] DataFormat 角度分辨率类型不支持: %d，使用默认值", fmt.DataFormat.AngleResType);
        // 使用默认值而不是返回
        ladar_.angle_step_ = mdeg_to_rad(100.0f); // 默认0.1度分辨率
        ladar_.point_count_ = 3600;
    }

    if ((*(uint32_t*)(frame + 42)) & 0x00400000) {
        ladar_.dataMerged_ = false;
    }

    ladar_.param_flags_ |= dsDataFmt;
    RCLCPP_INFO(rclcpp::get_logger("Radar"), "[Radar] DataFormat 初始化完成，param_flags_ = 0x%08X", ladar_.param_flags_);
}

int RadarConnect::getDiffTime(int diffTime) {
    diffTimeCache_[diffTimeCacheIndex_] = diffTime;
    diffTimeCacheIndex_ = (diffTimeCacheIndex_ + 1) % DIFF_TIME_CACHE_NUM;
    int minDiffTime = diffTime;
    for (int t : diffTimeCache_) {
        if (t != 0 && t < minDiffTime) minDiffTime = t;
    }
    return minDiffTime;
}

int RadarConnect::ladarParseData(const uint8_t* frame, int frameLength)
{
#define Frame_DataFormat(frame) (*(uint32_t*)((frame) + 42))
#define Frame_DataOffset(frame) (*(uint16_t*)((frame) + 36))
#define Frame_DataCount(frame) (*(uint16_t*)((frame) + 38))
#define Frame_DataFmtSize(frame) (*(uint16_t*)((frame) + 40))
#define Frame_Total(frame) ((uint32_t)((frame)[59]))
#define Frame_Index(frame) ((uint32_t)((frame)[60]))
#define Frame_WaveCount(frame) ((uint32_t)((frame)[33]))
#define Frame_ShiftInfo(frame) ((uint32_t*)((frame)[42 + 19]))
#define Frame_AngleStart(frame) (*(int32_t*)((frame) + 46))
#define Frame_AngleStop(frame) (*(int32_t*)((frame) + 50))
#define Frame_AngleStep(frame) (*(uint16_t*)((frame) + 54))

    static const int32_t scales[] = { 10, 1, 0, 0, 0, 0, 10, 1 };
    static const int32_t sizes[] = { 2, 2, 0, 0, 0, 0, 4, 4 };

    const uint32_t unitIndex = frame[32];
    if (unitIndex >= 8) return -1;

    const int32_t datascale = scales[unitIndex];
    const int32_t datasize = sizes[unitIndex];
    if (datascale == 0) return -1;

    int waveCount;
    switch (Frame_WaveCount(frame))
    {
    case 0: case 1: case 4: waveCount = 1; break;
    case 2: case 5: case 6: waveCount = 2; break;
    default: return -1;
    }

    const int dataOffset = Frame_DataOffset(frame);
    const int dataCount   = Frame_DataCount(frame);
    const int has_intensity = (Frame_DataFormat(frame) & 0x00200000) ? 1 : 0;
    const int datastrip = datasize + has_intensity;

    if (42 + dataOffset + datastrip * dataCount != frameLength)
        return -1;

    const float angle_start = mdeg_to_rad((float)Frame_AngleStart(frame));
    const float angle_stop  = mdeg_to_rad((float)Frame_AngleStop(frame));
    const uint32_t angleResType = ((Frame_DataFormat(frame) >> 14) & 3);

    float angle_step = (float)Frame_AngleStep(frame);
    switch (angleResType)
    {
    case 0: angle_step = mdeg_to_rad(angle_step); break;
    case 1: angle_step = mdeg_to_rad(angle_step / 100); break;
    case 3: angle_step = mdeg_to_rad(360000.f / angle_step); break;
    default: return -1;
    }

    points_.clear();
    if (points_.capacity() < (size_t)dataCount)
        points_.reserve(dataCount);

    const float scale = datascale * 0.001f;

    int64_t ts_sec  = *((uint32_t*)(frame + 40 + dataOffset - 8));
    int64_t ts_usec = *((uint32_t*)(frame + 40 + dataOffset - 4));
    auto now = std::chrono::system_clock::now().time_since_epoch();
    int64_t currentTime = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    int64_t revTime = ts_sec * 1000000LL + ts_usec;
    int32_t diffMs = static_cast<int32_t>((currentTime - revTime) / 1000);
    int32_t newDiffMs = getDiffTime(diffMs);
    revtimeStamp_ = revTime / 1000 + newDiffMs;

    uint8_t* rdata = (uint8_t*)(frame + 42 + dataOffset);
    float a = angle_start;

    std::vector<float> xs, ys, intensities;
    xs.reserve(dataCount);
    ys.reserve(dataCount);
    intensities.reserve(dataCount);

    for (int i = 0; i < dataCount; ++i)
    {
        double distance = (datasize == 2 ? *(uint16_t*)(rdata) : *(uint32_t*)(rdata)) * scale;

        double x = cosf(a) * distance;
        double y = sinf(a) * distance;

        if (mHeight_ > 0.0f) y = mHeight_ - y;
        if (isEnableLimitX_) {
            if (x < startX_ || x > endX_) {
                x = 0.0; y = 0.0;
            }
        }

        float intensity_value = 0.0f;
        if (has_intensity) {
            intensity_value = static_cast<float>(*(rdata + datasize));
        }

        xs.push_back(static_cast<float>(x));
        ys.push_back(static_cast<float>(y));
        intensities.push_back(intensity_value);

        rdata += datastrip * waveCount;
        a += angle_step;
    }

    if (Frame_Total(frame) - Frame_Index(frame) == 1)
    {
        if (revCallback) {
            revCallback(mMark, mPriv, revtimeStamp_, xs, ys, intensities);
        }
        return 1;
    }

    return 0;

#undef Frame_DataFormat
#undef Frame_DataOffset
#undef Frame_DataCount
#undef Frame_DataFmtSize
#undef Frame_Total
#undef Frame_Index
#undef Frame_WaveCount
#undef Frame_ShiftInfo
#undef Frame_AngleStart
#undef Frame_AngleStop
#undef Frame_AngleStep
}

void RadarConnect::ladarHandleFrame( const uint8_t* frame, int32_t frameLength)
{
    const uint32_t lsDirtyFlag = 0x01;
    const uint32_t lsFaultFlag = 0x08;
    const uint32_t cmdWord = *(uint16_t*)(frame + 2);

    switch (cmdWord)
    {
    case 7138:
        ladarHandleBaseParam( frame, frameLength);
        sendGetDataFormatCmd();
        break;

    case 6201:
        ladarHandleBaseParamHS(frame, frameLength);
        break;

    case 7130:
        ladarHandleDataFormat( frame, frameLength);
        break;

    case 3051:
        RCLCPP_INFO(rclcpp::get_logger("Radar"), "[Radar] 登录成功 (3051)，请求参数信息");
        ladar_.param_flags_ |= dsLogin;
        sendGetInfoCmd();
        break;

    case 110:
        RCLCPP_DEBUG(rclcpp::get_logger("Radar"), "[Radar] 收到数据帧，param_flags_ = 0x%08X", ladar_.param_flags_);
		if ((ladar_.param_flags_ & (dsParam | dsDataFmt)) == (dsParam | dsDataFmt))
        {
            int result = ladarParseData(frame, frameLength);

            if (result == 1 && revCallback)
            {
                // ladarParseData 已经在内部调用了回调，这里不需要重复调用
            }
        }
        else
        {
            static int warn_count = 0;
            if (++warn_count % 100 == 0) {  // 每100帧打印一次
                RCLCPP_WARN(rclcpp::get_logger("Radar"), "[Radar] 参数未完全初始化 (param_flags_ = 0x%08X)，跳过数据帧", ladar_.param_flags_);
            }
        }
        break;

    case 7105:
        ladar_.status |= lsDirtyFlag;
        break;
    case 7106:
        ladar_.status &= (~lsDirtyFlag);
        break;
    case 7109:
        ladar_.status |= lsFaultFlag;
        break;
    case 7110:
        ladar_.status &= (~lsFaultFlag);
        break;
    default:
        break;
    }
}
