#include "serial_station.hpp"
#include <fmt/core.h>
#include <sstream>
#include <iomanip>

SerialStation::SerialStation(SerialConfig_t &config) : Node("serial_station") {
    port_name_ = config.port;
    start_time_ = this->now();
    last_rx_time_ = this->now();

    // Set up the serial port
    serial_.setPort(config.port);
    serial_.setBaudrate(config.baudrate);
    serial_.setTimeout(config.timeout);
    serial_.setBytesize(config.bytesize);
    serial_.setParity(config.parity);
    serial_.setStopbits(config.stopbits);
    serial_.setFlowcontrol(config.flowcontrol);

    // Open the serial port
    try {
        serial_.open();
    } catch (serial::IOException &e) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open serial port %s", config.port.c_str());
    }
    if (serial_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Serial port %s is open", config.port.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Serial port %s is not open", config.port.c_str());
    }

    serial_.flush();

    // 状态发布器 — 每1秒发布一次连接状态和统计信息
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/serial_station/status", 10);
    status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() { this->publishStatus(); }
    );

    // TODO： 应当得改为用多线程实现，使用wall_timer，本质上只有一个线程
    tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(config.tx_handle_period),
            [this]() { this->loadTx(); }
    );

    rx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(config.rx_handle_period),
            [this]() { this->rxCallback(); }
    );
}

SerialStation::~SerialStation() {
    serial_.close();
}

void SerialStation::transmit(std::vector<uint8_t> &data, MessageType_e dataID) {
    // 先将发送数据存入队列，防止外部数据更新过快
    tx_queue_.push(std::move(data));
    if (encode_func_) {
        encode_func_(tx_queue_.back(), dataID);
    }
}

void SerialStation::loadTx() {
    if (!tx_queue_.empty()) {
        size_t written = serial_.write(tx_queue_.front());
        if (written > 0) {
            total_bytes_tx_ += written;
            total_packets_tx_++;
        }
        // TODO: 存在风险，serial_尚未将数据放入串口发送缓冲区，而数据已经pop，此时将跳出内存错误
        //  需要一套生命周期管理机制

//        // Debug
//        std::string data_str;
//        for (const auto& byte : tx_queue_.front())
//        {
//            data_str += fmt::format("{:02X}", byte);
//        }
//        RCLCPP_INFO(this->get_logger(), "Data:[%s], TX Queue size: %zu",
//                    data_str.c_str(), tx_queue_.size());

        tx_queue_.pop();
    }
}

void SerialStation::rxCallback() {
    rx_buffer_.clear(); // 清空缓冲区

    auto bytes_read = serial_.read(rx_buffer_, 200);
    if (bytes_read > 0) {
        total_bytes_rx_ += bytes_read;
        total_packets_rx_++;
        last_rx_time_ = this->now();

        if (decode_func_) {    // 复制一份缓冲区数据
            std::vector<uint8_t> data(rx_buffer_.begin(), rx_buffer_.begin() + bytes_read);

            // Debug
//            std::string data_str;
//            for (const auto& byte : data)
//            {
//                data_str += fmt::format("{:02X}", byte);
//            }
//            RCLCPP_INFO(this->get_logger(), "Data: [%s]", data_str.c_str());

            decode_func_(std::move(data));
        }
    }
}

void SerialStation::publishStatus() {
    auto now = this->now();
    double uptime = (now - start_time_).seconds();
    double since_last_rx = (now - last_rx_time_).seconds();

    std::stringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "--- Serial Station Status ---\n";
    ss << "port: " << port_name_ << "\n";
    ss << "open: " << (serial_.isOpen() ? "true" : "false") << "\n";
    ss << "uptime: " << uptime << " s\n";
    ss << "tx_bytes: " << total_bytes_tx_ << "\n";
    ss << "tx_packets: " << total_packets_tx_ << "\n";
    ss << "rx_bytes: " << total_bytes_rx_ << "\n";
    ss << "rx_packets: " << total_packets_rx_ << "\n";
    ss << "since_last_rx: " << since_last_rx << " s\n";

    // 诊断提示
    if (!serial_.isOpen()) {
        ss << "WARN: Serial port is NOT open!\n";
    } else if (total_packets_rx_ == 0 && uptime > 3.0) {
        ss << "WARN: No data received since start. Check:\n";
        ss << "  - Is the device connected?\n";
        ss << "  - Is the baudrate correct? (current: " << serial_.getBaudrate() << ")\n";
        ss << "  - Is the port correct?\n";
    } else if (since_last_rx > 2.0 && total_packets_rx_ > 0) {
        ss << "WARN: No data received for " << since_last_rx << " s\n";
    } else if (total_packets_rx_ > 0) {
        ss << "STATUS: Receiving data OK\n";
    }

    auto msg = std_msgs::msg::String();
    msg.data = ss.str();
    status_pub_->publish(msg);

    // 同时输出到日志（每10秒一次，避免刷屏）
    static int log_counter = 0;
    if (++log_counter % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "Status: port=%s, open=%s, rx_pkts=%lu, rx_bytes=%lu, since_last_rx=%.1fs",
                    port_name_.c_str(),
                    serial_.isOpen() ? "yes" : "NO",
                    total_packets_rx_,
                    total_bytes_rx_,
                    since_last_rx);
        if (!serial_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "SERIAL PORT NOT OPEN! Check port: %s", port_name_.c_str());
        } else if (total_packets_rx_ == 0 && uptime > 3.0) {
            RCLCPP_WARN(this->get_logger(), "NO DATA RECEIVED! Check wiring and baudrate.");
        }
    }
}
