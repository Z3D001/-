#include <rclcpp/rclcpp.hpp>
#include <zmq.hpp>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <thread>
#include <atomic>

namespace fs = std::filesystem;

class PointCloudReceiver : public rclcpp::Node {
public:
    PointCloudReceiver() : Node("pointcloud_receiver") {
        this->declare_parameter("sender_ip", "192.168.2.76");
        this->declare_parameter("zmq_port", 5555);
        this->declare_parameter("save_dir", "received_data");

        sender_ip_ = this->get_parameter("sender_ip").as_string();
        zmq_port_ = this->get_parameter("zmq_port").as_int();
        base_save_dir_ = this->get_parameter("save_dir").as_string();

        zip_dir_ = base_save_dir_ + "/zip";
        pcd_dir_ = base_save_dir_ + "/pcd";

        fs::create_directories(zip_dir_);
        fs::create_directories(pcd_dir_);

        zmq_context_ = std::make_unique<zmq::context_t>(1);
        zmq_subscriber_ = std::make_unique<zmq::socket_t>(*zmq_context_, zmq::socket_type::sub);
        
        std::string addr = "tcp://" + sender_ip_ + ":" + std::to_string(zmq_port_);
        zmq_subscriber_->connect(addr);
        zmq_subscriber_->set(zmq::sockopt::subscribe, "");
        zmq_subscriber_->set(zmq::sockopt::rcvhwm, 100);
        zmq_subscriber_->set(zmq::sockopt::rcvbuf, 128 * 1024 * 1024);

        receive_thread_ = std::thread(&PointCloudReceiver::receiveWorker, this);

        RCLCPP_INFO(this->get_logger(), "PointCloudReceiver 已启动");
        RCLCPP_INFO(this->get_logger(), "ZIP 保存 → %s", zip_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "PCD 保存 → %s", pcd_dir_.c_str());
    }

    ~PointCloudReceiver() {
        running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
    }

private:
    void receiveWorker() {
        int package_id = 0;

        while (running_) {
            zmq::message_t message;
            if (!zmq_subscriber_->recv(message, zmq::recv_flags::none)) continue;

            package_id++;

            auto now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");

            std::string zip_path = zip_dir_ + "/scan_" + ss.str() + "_" + 
                                  std::to_string(package_id) + ".zip";

            // 保存 zip
            std::ofstream ofs(zip_path, std::ios::binary);
            ofs.write(static_cast<const char*>(message.data()), message.size());
            ofs.close();

            // 解压到 pcd 目录
            std::string cmd = "unzip -oq \"" + zip_path + "\" -d \"" + pcd_dir_ + "\"";
            int ret = std::system(cmd.c_str());

            if (ret == 0) {
                RCLCPP_INFO(this->get_logger(), "成功接收并解压第 %d 个包 | 大小: %.2f MB", 
                            package_id, message.size() / (1024.0 * 1024.0));
            } else {
                RCLCPP_ERROR(this->get_logger(), "解压失败: %s", zip_path.c_str());
            }
        }
    }

    std::unique_ptr<zmq::context_t> zmq_context_;
    std::unique_ptr<zmq::socket_t> zmq_subscriber_;
    std::string sender_ip_;
    int zmq_port_;
    std::string base_save_dir_;
    std::string zip_dir_;
    std::string pcd_dir_;

    std::thread receive_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
