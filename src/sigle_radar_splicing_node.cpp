#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <deque>
#include <condition_variable>
#include <thread>
#include <atomic>

#include <zmq.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cstdlib>

#include "sigle_radar/radarconnect.h"

namespace fs = std::filesystem;

class SingleRadarSplicingNode : public rclcpp::Node {
public:
    SingleRadarSplicingNode() : Node("sigle_radar_splicing_node") {
        this->declare_parameter("lidar_ip", "192.168.2.83");
        this->declare_parameter("lidar_port", 1112);
        this->declare_parameter("segment_time_sec", 0.35);
        this->declare_parameter("odom_threshold_m", 0.025);

        // ==================== 新增：ZeroMQ 参数 ====================
        this->declare_parameter("receiver_ip", "192.168.2.152");   // Host B IP
        this->declare_parameter("zmq_port", 5555);
        this->declare_parameter("pcd_save_dir", "/tmp/radar_pcd");

        radar_ip_ = this->get_parameter("lidar_ip").as_string();
        radar_port_ = this->get_parameter("lidar_port").as_int();
        segment_time_sec_ = this->get_parameter("segment_time_sec").as_double();
        odom_threshold_m_ = this->get_parameter("odom_threshold_m").as_double();

        receiver_ip_ = this->get_parameter("receiver_ip").as_string();
        zmq_port_ = this->get_parameter("zmq_port").as_int();
        pcd_save_dir_ = this->get_parameter("pcd_save_dir").as_string();

        // 创建临时目录
        if (!fs::exists(pcd_save_dir_)) {
            fs::create_directories(pcd_save_dir_);
        }

        // 初始化 ZeroMQ Publisher
        zmq_context_ = std::make_unique<zmq::context_t>(1);
        zmq_publisher_ = std::make_unique<zmq::socket_t>(*zmq_context_, zmq::socket_type::pub);
        std::string bind_addr = "tcp://*:" + std::to_string(zmq_port_);
        zmq_publisher_->bind(bind_addr);

        zmq_publisher_->set(zmq::sockopt::sndhwm, 50);
        zmq_publisher_->set(zmq::sockopt::sndbuf, 64 * 1024 * 1024); // 64MB

        RCLCPP_INFO(this->get_logger(), "ZeroMQ PUB 已绑定 %s | 目标接收端: %s", 
                    bind_addr.c_str(), receiver_ip_.c_str());
        // =======================================================

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/SingleRadar/pointcloud", 10);  // 可保留或后面删除

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SingleRadarSplicingNode::OdomCallback, this, std::placeholders::_1));

        control_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/SingleRadar/Start_or_Stop", 10, std::bind(&SingleRadarSplicingNode::ControlCallback, this, std::placeholders::_1));

        current_buffer_.reserve(500000);

        radar_connect_ = new RadarConnect("GLK20", radar_ip_, radar_port_, this,
            std::bind(&SingleRadarSplicingNode::OnLadarFrameCallback, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4,
                      std::placeholders::_5, std::placeholders::_6));

        radar_connect_->startConnect();

        publish_thread_ = std::thread(&SingleRadarSplicingNode::PublishWorker, this);

        RCLCPP_INFO(this->get_logger(), "节点启动 | 周期 %.2f 秒 | 静止阈值 %.3f 米", 
                    segment_time_sec_, odom_threshold_m_);
    }

    ~SingleRadarSplicingNode() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            running_ = false;
            queue_cv_.notify_one();
        }
        if (publish_thread_.joinable()) publish_thread_.join();
        if (radar_connect_) delete radar_connect_;
    }

private:
    // ==================== 新增成员变量 ====================
    std::unique_ptr<zmq::context_t> zmq_context_;
    std::unique_ptr<zmq::socket_t> zmq_publisher_;
    std::string receiver_ip_;
    int zmq_port_ = 5555;
    std::string pcd_save_dir_;
    int zip_package_id_ = 0;
    // ===================================================

    RadarConnect* radar_connect_ = nullptr;
    std::string radar_ip_;
    int radar_port_;
    double segment_time_sec_ = 0.35;
    double odom_threshold_m_ = 0.025;

    struct PointData {
        float x, y, z, intensity;
        int32_t package_id = 0;
        int32_t frame_id = 0;
    };

    int global_frame_id_ = 0;
    int package_id_ = 0;
    bool is_active_ = false;
    double latest_odom_x_ = 0.0;
    double reference_odom_x_ = 0.0;
    double last_publish_odom_x_ = 0.0;
    bool has_odom_ = false;

    std::vector<PointData> current_buffer_;
    std::deque<std::vector<PointData>> ready_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> running_{true};
    std::thread publish_thread_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr control_sub_;

    // ====================== 回调函数（保持不变） ======================
    void OnLadarFrameCallback(const std::string& /*mark*/, void* /*priv*/,
                              int64_t /*timestamp*/,
                              const std::vector<float>& xs,
                              const std::vector<float>& ys,
                              const std::vector<float>& intensities) {
        if (!is_active_) return;
        global_frame_id_++;
        std::lock_guard<std::mutex> lock(queue_mutex_);

        double current_z = (latest_odom_x_ - reference_odom_x_) * 1000.0;

        for (size_t i = 0; i < xs.size(); ++i) {
            current_buffer_.push_back({
                xs[i] * -1000.0f,
                ys[i] * 1000.0f,
                static_cast<float>(current_z),
                intensities[i],
                0,
                global_frame_id_
            });
        }
    }

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        latest_odom_x_ = msg->pose.pose.position.x;
        has_odom_ = true;
    }

    void ControlCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (msg->data <= 0.0f) {
            if (is_active_) {
                is_active_ = false;
                if (!current_buffer_.empty()) {
                    std::vector<PointData> last_package;
                    last_package.swap(current_buffer_);
                    PublishOnePackage(last_package);        // 处理剩余点云
                }
                RCLCPP_INFO(this->get_logger(), "拼接已停止");
            }
            return;
        }

        if (!has_odom_) {
            RCLCPP_WARN(this->get_logger(), "尚未收到 /odom，无法开始拼接！");
            return;
        }

        reference_odom_x_ = latest_odom_x_;
        last_publish_odom_x_ = latest_odom_x_;
        current_buffer_.clear();
        ready_queue_.clear();
        is_active_ = true;
        segment_time_sec_ = msg->data;
        global_frame_id_ = 0;
        package_id_ = 0;

        RCLCPP_INFO(this->get_logger(), "开始拼接 | 周期 %.2f 秒", msg->data);
    }

    void TimerPublishCallback() { /* 保持你原来的实现 */ 
        // ...（此处保持你原来的 TimerPublishCallback 代码不变）
        // 只需确保最后调用 PublishOnePackage
    }

    void PublishWorker() { /* 保持你原来的实现 */ }

    // ====================== 新核心函数 ======================
    void PublishOnePackage(std::vector<PointData>& points_to_pub) {
        if (points_to_pub.empty()) return;

        zip_package_id_++;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->reserve(points_to_pub.size());
        for (const auto& p : points_to_pub) {
            pcl::PointXYZI pt;
            pt.x = p.x / 1000.0f;
            pt.y = p.y / 1000.0f;
            pt.z = p.z / 1000.0f;
            pt.intensity = p.intensity;
            cloud->push_back(pt);
        }

        std::string pcd_path = pcd_save_dir_ + "/temp_" + std::to_string(zip_package_id_) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_path, *cloud);

        std::string zip_path = pcd_save_dir_ + "/scan_" + std::to_string(zip_package_id_) + ".zip";
        std::string cmd = "zip -q -j " + zip_path + " " + pcd_path;
        std::system(cmd.c_str());

        std::ifstream ifs(zip_path, std::ios::binary | std::ios::ate);
        if (!ifs) {
            RCLCPP_ERROR(this->get_logger(), "读取 zip 失败: %s", zip_path.c_str());
            return;
        }
        std::streamsize size = ifs.tellg();
        ifs.seekg(0, std::ios::beg);
        std::vector<char> data(size);
        ifs.read(data.data(), size);

        zmq::message_t msg(data.data(), size);
        zmq_publisher_->send(msg, zmq::send_flags::none);

        RCLCPP_INFO(this->get_logger(), 
            "✅ 已发送第 %d 个 zip 包 | 点数:%zu | 大小:%.2f MB", 
            zip_package_id_, points_to_pub.size(), size / (1024.0 * 1024.0));

        fs::remove(pcd_path);
        // fs::remove(zip_path);   // 如需保留 zip 可取消注释
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleRadarSplicingNode>());
    rclcpp::shutdown();
    return 0;
}