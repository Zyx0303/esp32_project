#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>

#include <mutex>
#include <condition_variable>
#include <bitset>
#include <sstream>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <fstream>

using boost::asio::ip::tcp;
std::vector<std::shared_ptr<tcp::socket>> sockets;
std::string filepath = "/home/hz/phs_ws/src/data_0528/";
std::string gaitName = "11100np2";
std::condition_variable cv;





std::bitset<5> threadReady;  // Using bitset to track which threads are ready
std::vector<std::string> globalBuffer(5);  // 预分配5个空间，对应5个线程


/*
                            uint8_t Gait_table[4][6] = {{1,1,0,0,0,1},
														{1,1,1,0,0,1},
														{1,1,1,1,0,1},
														{1,1,1,0,0,2},
														{0,0,0,0,0,0}};
*/
const std::vector<int> PORTS = {12340, 12341, 12342, 12343, 12344};

ros::Subscriber sub;

typedef struct {
    float q0, q1, q2, q3;        // Quaternion
    float accX, accY, accZ;     // Acceleration
    float gyrX, gyrY, gyrZ;     // Gyroscope (Rate of Turn)
} MTIData;

const size_t MTI_DATA_SIZE = sizeof(MTIData);
const size_t FRAME_HEADER_SIZE = 2;
const size_t FRAME_TAIL_SIZE = 2;
const size_t STATE_SIZE = 2;
const size_t TOTAL_FRAME_SIZE = FRAME_HEADER_SIZE + MTI_DATA_SIZE + FRAME_TAIL_SIZE + STATE_SIZE;


// 全局变量或成员变量
std::chrono::high_resolution_clock::time_point lastTimeCheck = std::chrono::high_resolution_clock::now();
int processDataCount[5] = {0};

sensor_msgs::Imu processData(const MTIData& data, const std::string& frame_id, const int& port) {
    sensor_msgs::Imu imu_msg;

    // 设置消息头部
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = frame_id;
    // 填充四元数数据
    imu_msg.orientation.w = data.q0;
    imu_msg.orientation.x = data.q1;
    imu_msg.orientation.y = data.q2;
    imu_msg.orientation.z = data.q3;

    // 填充加速度数据
    imu_msg.linear_acceleration.x = data.accX;
    imu_msg.linear_acceleration.y = data.accY;
    imu_msg.linear_acceleration.z = data.accZ;

    // 填充陀螺仪数据
    imu_msg.angular_velocity.x = data.gyrX;
    imu_msg.angular_velocity.y = data.gyrY;
    imu_msg.angular_velocity.z = data.gyrZ;

    // 如果有协方差信息，也可以填充它
    // 示例：imu_msg.orientation_covariance = {...};
    processDataCount[port]++;

    // // 检查是否过去了1秒
    // auto currentTime = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTimeCheck);

    // if (elapsed.count() >= 1000) {
    //     ROS_INFO("ProcessData rate: %d Hz", processDataCount);
    //     processDataCount = 0;
    //     lastTimeCheck = std::chrono::high_resolution_clock::now();
    // }

    return imu_msg;
}


void handleIncomingByte(const u_int8_t& byte, ros::Publisher& imu_pub, const std::string& frame_id, const int& port,
                std::vector<uint8_t>& frameBuffer, bool& flag_start, int& bytesToRead,
                uint8_t& now_state, uint8_t& last_state){
        // 调试：打印收到的每一个字节
    // ROS_INFO("Received byte: 0x%02X", byte);  // 以十六进制格式显示字节
    if (bytesToRead > 0 && flag_start) {
        frameBuffer.push_back(byte);
        bytesToRead--;

        if (bytesToRead == 0 && flag_start) {
            // 完整帧已接收，处理数据
            MTIData data;
                // printf("%02X %02X %d\n",frameBuffer[TOTAL_FRAME_SIZE - 2], frameBuffer[TOTAL_FRAME_SIZE - 1],TOTAL_FRAME_SIZE);

            // 如果使用帧尾，检查它,完整性检验,可扩展至CRC.
            if (frameBuffer[TOTAL_FRAME_SIZE - 2] != 0xFB && frameBuffer[TOTAL_FRAME_SIZE - 1] != 0xFC) {
                // ROS_WARN("Invalid frame tail detected!");
                flag_start = false;
                frameBuffer.clear();
                return;
            }
            int segNo = port - 12340;
            std::memcpy(&data, &frameBuffer[FRAME_HEADER_SIZE], sizeof(MTIData));
            sensor_msgs::Imu imu_msg = processData(data, frame_id, segNo);
            imu_pub.publish(imu_msg);
            now_state = frameBuffer[TOTAL_FRAME_SIZE - 4];
            last_state = frameBuffer[TOTAL_FRAME_SIZE - 3]; //segment state;
            std::string portStr = std::to_string(segNo);
            std::stringstream ss;
            ss << static_cast<int>(now_state) << " ";
            ss << static_cast<int>(last_state) << " ";
            ss << imu_msg.linear_acceleration.x << " ";
            ss << imu_msg.linear_acceleration.y << " ";
            ss << imu_msg.linear_acceleration.z << " ";
            ss << imu_msg.angular_velocity.x << " ";
            ss << imu_msg.angular_velocity.y << " ";
            ss << imu_msg.angular_velocity.z << " ";
            ss << imu_msg.orientation.w << " ";
            ss << imu_msg.orientation.x << " ";
            ss << imu_msg.orientation.y << " ";
            ss << imu_msg.orientation.z;

            std::ofstream localOutFile;  // 线程局部的文件输出对象
            std::string filename = filepath + gaitName + "_seg" + portStr + ".txt";
            localOutFile.open(filename, std::ios::out | std::ios::app);
            if(localOutFile.is_open()) {
                localOutFile << ss.str() << std::endl; // 将数据写入文件
                localOutFile.close(); // 关闭文件
            } else {
                // 如果出现问题，可以打印错误或其他反馈信息
                std::cerr << "Failed to open file for writing: " << filename << std::endl;
            }

            frameBuffer.clear();
        }
            return;
    }
    // 检查帧头
    if (byte == 0xFA) {
        frameBuffer.push_back(byte);
    } else if (!frameBuffer.empty() && byte == 0xFF) {
        frameBuffer.push_back(byte);
        bytesToRead = MTI_DATA_SIZE + FRAME_TAIL_SIZE + STATE_SIZE;
        flag_start = true;
    } else {
        frameBuffer.clear();  // 重置缓冲区
    }
}

// void keyboardInputCallback(const std_msgs::ByteMultiArray::ConstPtr& msg, const std::shared_ptr<tcp::socket>& socket_ptr) {
//     // 将数据发送到ESP32
//     if (socket_ptr->is_open()) {
//         try {
//             boost::asio::write(*socket_ptr, boost::asio::buffer(msg->data));
//             ROS_INFO("%s have sent.",msg);
//         } catch (const boost::system::system_error& e) {
//             ROS_WARN("Error sending data: %s", e.what());
//         }
//     } else {
//         ROS_WARN("Socket not connected. Can't send data.");
//     }
// }
std::string byteArrayToHexString(const std::vector<uint8_t>& data) {
    std::stringstream ss;
    ss << std::hex;
    for (uint8_t byte : data) {
        ss << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    return ss.str();
}

void keyboardInputCallback(const std_msgs::ByteMultiArray::ConstPtr& msg, const std::shared_ptr<tcp::socket>& socket_ptr) {
    // Convert data to appropriate type
    const std::vector<uint8_t> data_to_send(msg->data.begin(), msg->data.end());

    // 调试：打印即将发送的数据
    std::string dataStr = byteArrayToHexString(data_to_send);
    ROS_INFO("Sending data to ESP32: %s", dataStr.c_str());

    // Send data to ESP32
    if (socket_ptr->is_open()) {
        try {
            boost::asio::write(*socket_ptr, boost::asio::buffer(data_to_send));
            std::string dataStr = byteArrayToHexString(data_to_send);
            ROS_INFO("Data sent: %s", dataStr.c_str());
        } catch (const boost::system::system_error& e) {
            ROS_WARN("Error sending data: %s", e.what());
        }
    } else {
        ROS_WARN("Socket not connected. Can't send data.");
    }
}

void accept_connection(boost::asio::io_service& io_service,
                       std::shared_ptr<boost::asio::ip::tcp::socket> &socket_ptr,
                       boost::asio::ip::tcp::acceptor& acceptor) {
    std::cout << "Waiting for a connection..." << std::endl;
    acceptor.accept(*socket_ptr);
    std::cout << "Connection established!" << std::endl;
}



/**
 * @brief Establishes a TCP server to receive data from ESP32 and publishes the data to a ROS topic.
 *
 * @param port The TCP port number on which the server listens.
 * @param topic_name The name of the ROS topic to which the data is published.
 */
void tcpServer(int port, const std::string& topic_name) {
    // Initialize a ROS node handle
    ros::NodeHandle nh;
    std::vector<uint8_t> frameBuffer;  // 线程专有的frame buffer
    bool flag_start = false; // 将flag_start变量移动到这里
    int bytesToRead = MTI_DATA_SIZE + FRAME_TAIL_SIZE + STATE_SIZE; // 将bytesToRead变量移动到这里
    uint8_t now_state = 0;
    uint8_t last_state = 0;

    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>(topic_name, 1000);

    // Generate a frame ID using the port number
    std::string frame_id = "port_" + std::to_string(port);

    boost::asio::io_service io_service;
    tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), port));

    ROS_INFO("Waiting for ESP32 connection on port %d...", port);

    auto socket_ptr = std::make_shared<tcp::socket>(io_service);
    acceptor.accept(*socket_ptr);  // This will block until a connection is established
    sockets.push_back(socket_ptr);
    ROS_INFO("ESP32 connected on port %d!", port);

    ros::Subscriber sub = nh.subscribe<std_msgs::ByteMultiArray>(
        "keyboard_input",
        1000,
        boost::bind(keyboardInputCallback, _1, socket_ptr)
    );

    while (ros::ok()) {
        // ROS_INFO_THROTTLE(5, "Waiting for data from ESP32 on port %d...", port);
        boost::array<uint8_t, 9200> buf; // Buffer to store incoming data
        boost::system::error_code error; // Variable to store any error information
        // printf("\run here\n");
        size_t len = socket_ptr->read_some(boost::asio::buffer(buf), error);
        // ROS_INFO("len receive on port %d!", len);
        if (error == boost::asio::error::eof) {
        // Connection closed by the client
        std::cerr << "Connection closed by the client." << std::endl;

        // Reconnect logic
        while (ros::ok()) {
            try {
                std::cout << "Trying to reconnect...port : " << port <<std::endl;
                // Assuming reconnect function attempts to re-establish the connection
                accept_connection(io_service, socket_ptr, acceptor);
                std::cout << "Reconnected successfully port :"<< port<<" !" << std::endl;
                break;
            } catch (...) {
                std::cerr << "Reconnection failed, trying again in 5 seconds..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
        }
    } else if (error) {
        // If any other error occurs, throw an exception
        throw boost::system::system_error(error);
    } else {
        // ROS_INFO("Received run here \n");  // 打印接收到的字节数
        for (size_t i = 0; i < len; i++) {
            handleIncomingByte(buf[i], pub, frame_id, port, frameBuffer, flag_start, bytesToRead,now_state,last_state);
        }
    }
        ros::spinOnce();
    }
}

void timerCallback(const ros::TimerEvent&) {
    // 打印所有线程的processDataCount
    ROS_INFO("%d: %d, %d : %d, %d : %d, %d : %d, %d : %d",
             PORTS[0]-12340, processDataCount[0],
             PORTS[1]-12340, processDataCount[1],
             PORTS[2]-12340, processDataCount[2],
             PORTS[3]-12340, processDataCount[3],
             PORTS[4]-12340, processDataCount[4]);
    for (size_t i = 0; i < PORTS.size(); i++) {
        processDataCount[i] = 0;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "esp32_tcp_server");

    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(1), timerCallback); // 1-second duration

    std::vector<std::thread> serverThreads;
    for (int port : PORTS) {
        std::string topic_name = "imu_data_" + std::to_string(port);
        serverThreads.push_back(std::thread(tcpServer, port, topic_name));
    }

    for (std::thread& t : serverThreads) {
        t.join();
    }

    return 0;
}
