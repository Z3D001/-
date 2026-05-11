#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <deque>
#include <condition_variable>
#include <thread>
#include <atomic>

#include <zmq.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <cstdlib>

#include "sigle_radar/radarconnect.h"

namespace fs = std::filesystem;

class SingleRadarSplicingNode : public rclcpp::Node {
public:
    SingleRadarSplicingNode() : Node("sigle_radar_splicing_node") {
        this->declare_parameter("lidar_ip", "192.168.2.83");  // 雷达IP
        this->declare_parameter("lidar_port", 1112);
        this->declare_parameter("segment_time_sec", 0.35);   // 默认拼接周期   
        this->declare_parameter("odom_threshold_m", 0.025);  // 检测小车移动距离
        this->declare_parameter("receiver_ip", "192.168.2.152"); // 接收端主机ip
        this->declare_parameter("zmq_port", 5555);              // 自定义端口号
        this->declare_parameter("pcd_save_dir", "/tmp/radar_pcd");

        radar_ip_ = this->get_parameter("lidar_ip").as_string();
        radar_port_ = this->get_parameter("lidar_port").as_int();
        segment_time_sec_ = this->get_parameter("segment_time_sec").as_double();
        odom_threshold_m_ = this->get_parameter("odom_threshold_m").as_double();
        receiver_ip_ = this->get_parameter("receiver_ip").as_string();
        zmq_port_ = this->get_parameter("zmq_port").as_int();
        pcd_save_dir_ = this->get_parameter("pcd_save_dir").as_string();

        fs::create_directories(pcd_save_dir_);

        // ZeroMQ 初始化
        zmq_context_ = std::make_unique<zmq::context_t>(1);
        zmq_publisher_ = std::make_unique<zmq::socket_t>(*zmq_context_, zmq::socket_type::pub);
        zmq_publisher_->bind("tcp://*:" + std::to_string(zmq_port_));

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

        rclcpp::on_shutdown([this]() {
            running_ = false;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                queue_cv_.notify_all();
            }
        });

        RCLCPP_INFO(this->get_logger(), "节点启动 | 周期 %.2f 秒 | 静止阈值 %.3f 米", 
                    segment_time_sec_, odom_threshold_m_);
    }

    ~SingleRadarSplicingNode() {
        running_ = false;
        
        // 强制唤醒线程
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            queue_cv_.notify_all();      // ← 改成 notify_all
        }
        
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }
        
        if (publish_timer_) {
            publish_timer_->cancel();
            publish_timer_.reset();
        }
        
    }

private:
    // ZeroMQ
    std::unique_ptr<zmq::context_t> zmq_context_;
    std::unique_ptr<zmq::socket_t> zmq_publisher_;
    std::string receiver_ip_;
    int zmq_port_ = 5555;
    std::string pcd_save_dir_;
    int zip_package_id_ = 0;

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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr control_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    //  Z 拼接：odom的x
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
                    ready_queue_.push_back(std::move(last_package));
                    queue_cv_.notify_one();
                }
                if (publish_timer_) publish_timer_->cancel();
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

        lock.unlock();

        if (publish_timer_) publish_timer_->cancel();
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(msg->data),
            std::bind(&SingleRadarSplicingNode::TimerPublishCallback, this));

        RCLCPP_INFO(this->get_logger(), "开始拼接 | 周期 %.2f 秒", msg->data);
    }

    void TimerPublishCallback() {
        std::vector<PointData> package_to_queue;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            double moved = std::abs(latest_odom_x_ - last_publish_odom_x_);
            if (moved < odom_threshold_m_) {
                current_buffer_.clear();
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "小车静止 (%.4f < %.3f 米)，跳过本次切包", moved, odom_threshold_m_);
                return;
            }
            if (current_buffer_.empty()) return;

            package_to_queue.swap(current_buffer_);
        }

        {
            std::lock_guard<std::mutex> qlock(queue_mutex_);
            ready_queue_.push_back(std::move(package_to_queue));
            queue_cv_.notify_one();
        }
        last_publish_odom_x_ = latest_odom_x_;
    }

    void PublishWorker() {
        while (running_ && rclcpp::ok()) {          
            std::vector<PointData> points_to_pub;
            
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                
                //  wait_for + 超时 + 完整退出条件
                queue_cv_.wait_for(lock, std::chrono::milliseconds(300),
                    [this]() { 
                        return !ready_queue_.empty() || !running_ || !rclcpp::ok(); 
                    });
                
                if (!running_ || !rclcpp::ok()) {
                    break;
                }
                
                if (!ready_queue_.empty()) {
                    points_to_pub = std::move(ready_queue_.front());
                    ready_queue_.pop_front();
                }
            }   // 锁在这里释放
            
            if (!points_to_pub.empty()) {
                PublishOnePackage(points_to_pub);
            }
        }
        
    }

    // PCD + ZIP + ZeroMQ 发送 
    void PublishOnePackage(std::vector<PointData>& points_to_pub) {
        if (points_to_pub.empty()) return;
        zip_package_id_++;

        // 1. 转为 PCL 点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->reserve(points_to_pub.size());
        for (const auto& p : points_to_pub) {
            cloud->push_back({p.x/1000.0f, p.y/1000.0f, p.z/1000.0f, p.intensity});
        }

        // 2. XY 裁剪
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& p : *cloud) {
            if ((p.y > -1.5 && p.y < 2.0) && (p.x > -8.0 && p.x < 8.0)) {   // 根据场景调整范围
                cloud_xy->push_back(p);
            }
        }

        // 3. 半径滤波 
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        if (cloud_xy->size() > 100) {
            pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_filter;
            radius_filter.setInputCloud(cloud_xy);
            radius_filter.setRadiusSearch(0.55);        // 半径（米）
            radius_filter.setMinNeighborsInRadius(10);  // 邻居数
            radius_filter.setNegative(false);
            radius_filter.filter(*cloud_filtered);
        } else {
            cloud_filtered = cloud_xy;
        }

        // 4. 保存 PCD（滤波后版本）
        std::string pcd_path = pcd_save_dir_ + "/temp_" + std::to_string(zip_package_id_) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_path, *cloud_filtered);

        // 5. 压缩 + 发送
        std::string zip_path = pcd_save_dir_ + "/scan_" + std::to_string(zip_package_id_) + ".zip";
        std::system(("zip -q -j " + zip_path + " " + pcd_path).c_str());

        std::ifstream ifs(zip_path, std::ios::binary | std::ios::ate);
        std::streamsize size = ifs.tellg();
        ifs.seekg(0, std::ios::beg);
        std::vector<char> data(size);
        ifs.read(data.data(), size);

        zmq::message_t msg(data.data(), size);
        zmq_publisher_->send(msg, zmq::send_flags::none);

        RCLCPP_INFO(this->get_logger(), 
            "已发送第 %d 个 zip 包 | 原始:%zu → 过滤后:%zu | 大小:%.2f MB", 
            zip_package_id_, points_to_pub.size(), cloud_filtered->size(), size / (1024.0 * 1024.0));

        fs::remove(pcd_path);
    }
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SingleRadarSplicingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();   // 显式调用
    return 0;
}
