#ifndef FINES_NODE_SERIAL_STATION_HPP
#define FINES_NODE_SERIAL_STATION_HPP

#include <queue>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "serial/serial.h"
#include "msg_types.hpp"

typedef struct
{
    const std::string port; // e.g. "/dev/ttyUSB0"
    uint32_t baudrate;
    serial::Timeout timeout;
    serial::bytesize_t bytesize;
    serial::parity_t parity;
    serial::stopbits_t stopbits;
    serial::flowcontrol_t flowcontrol;
    int tx_handle_period; // 发送处理周期，单位ms
    int rx_handle_period; // 接收处理周期，单位ms
} SerialConfig_t;

class SerialStation : public rclcpp::Node
{
public:
    explicit SerialStation(SerialConfig_t &config);
    ~SerialStation();

    void bindEncodeFunc(std::function<void(std::vector<uint8_t>&, MessageType_e)> && encode_func) {
        encode_func_ = encode_func;
    }

    void bindDecodeFunc(std::function<void(std::vector<uint8_t>)> && decode_func) {
        decode_func_ = decode_func;
    }

    /**
     * @brief Transmit data over the serial port
     * @param data The data to transmit
     * @param dataID 用户通信协议规范的数据ID，范围0～255
     * @note 用户不应当使用调用transmit的data，否则将导致未定义行为
     * @note 只有在spin() 或 spinOnce() 后才能工作
     */
    void transmit(std::vector<uint8_t> &data, MessageType_e dataID);

private:
    void loadTx();
    void rxCallback();
    void publishStatus();

private:
    serial::Serial serial_;
    rclcpp::TimerBase::SharedPtr tx_timer_;
    rclcpp::TimerBase::SharedPtr rx_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    std::queue<std::vector<uint8_t>> tx_queue_; // 串口发送任务缓冲队列，存储数据对象地址
    std::vector<uint8_t> rx_buffer_; // 串口接收缓冲区，即取即刷

    std::function<void(std::vector<uint8_t>&, MessageType_e)> encode_func_ = nullptr;
    std::function<void(std::vector<uint8_t>)> decode_func_ = nullptr;

    // 统计信息
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    uint64_t total_bytes_tx_ = 0;
    uint64_t total_bytes_rx_ = 0;
    uint64_t total_packets_tx_ = 0;
    uint64_t total_packets_rx_ = 0;
    rclcpp::Time last_rx_time_;
    rclcpp::Time start_time_;
    std::string port_name_;
};

#endif //FINES_NODE_SERIAL_STATION_HPP
