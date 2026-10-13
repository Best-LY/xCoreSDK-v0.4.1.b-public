#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>

#define PORT 12345
#define MAX_CONNECTIONS 5
#define BUFFER_SIZE 1024

// 定义数学常量（四舍五入到两位小数）
#define M_E_2DEC		2.72
#define M_PI_2DEC		3.14
#define M_PI_2_2DEC	1.57
#define M_PI_4_2DEC	0.79
#define M_1_PI_2DEC	0.32
#define M_2_PI_2DEC	0.64
#define M_SQRT2_2DEC	1.41
#define M_SQRT1_2_2DEC	0.71

int main() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    int opt = 1;

    // 创建 socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        return 1;
    }

    // 设置 socket 选项以允许重用地址
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt failed");
        return 1;
    }

    // 绑定 socket 到指定端口
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        return 1;
    }

    // 开始监听
    if (listen(server_fd, MAX_CONNECTIONS) < 0) {
        perror("listen failed");
        return 1;
    }

    std::cout << "服务器已启动，等待连接..." << std::endl;

    // 接受客户端连接
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0) {
        perror("accept failed");
        return 1;
    }

    std::cout << "客户端已连接" << std::endl;

    char buffer[BUFFER_SIZE];
    std::string input;

    while (true) {
        // 从客户端接收数据
        int valread = read(new_socket, buffer, BUFFER_SIZE);
        if (valread <= 0) {
            break;
        }

        buffer[valread] = '\0';
        input = buffer;

        // 检查是否需要退出
        if (input == "exit") {
            break;
        }

        // 解析输入的 6 个数字
        std::istringstream iss(input);
        std::vector<double> numbers;
        std::string token;

        while (std::getline(iss, token, ',')) {
            try {
                double num = std::stod(token);
                // 四舍五入到两位小数
                num = std::round(num * 100) / 100;
                numbers.push_back(num);
            } catch (const std::exception& e) {
                std::cerr << "输入格式错误: " << e.what() << std::endl;
                break;
            }
        }

        if (numbers.size() != 6) {
            std::cerr << "输入必须包含 6 个数字！" << std::endl;
            continue;
        }

        // 生成 std::vector<double> 的初始化字符串
        std::string output = "{";
        for (size_t i = 0; i < numbers.size(); ++i) {
            if (i > 0) {
                output += ", ";
            }

            double num = numbers[i];
            // 检查是否可以表示为数学常量
            if (std::abs(num) < 0.005) {
                output += "0";  // 零
            } else if (std::abs(num - M_E_2DEC) < 0.005) {
                output += "M_E";
            } else if (std::abs(num - M_PI_2DEC) < 0.005) {
                output += "M_PI";
            } else if (std::abs(num - M_PI_2_2DEC) < 0.005) {
                output += "M_PI_2";
            } else if (std::abs(num - M_PI_4_2DEC) < 0.005) {
                output += "M_PI_4";
            } else if (std::abs(num - M_1_PI_2DEC) < 0.005) {
                output += "M_1_PI";
            } else if (std::abs(num - M_2_PI_2DEC) < 0.005) {
                output += "M_2_PI";
            } else if (std::abs(num - M_SQRT2_2DEC) < 0.005) {
                output += "M_SQRT2";
            } else if (std::abs(num - M_SQRT1_2_2DEC) < 0.005) {
                output += "M_SQRT1_2";
            } else if (std::abs(num + M_PI_2DEC) < 0.005) {
                output += "-M_PI";
            } else if (std::abs(num + M_PI_2_2DEC) < 0.005) {
                output += "-M_PI_2";
            } else if (std::abs(num + M_PI_4_2DEC) < 0.005) {
                output += "-M_PI_4";
            } else if (std::abs(num - (M_PI_2DEC * 0.75)) < 0.005) {
                output += "M_PI*0.75";
            } else if (std::abs(num + (M_PI_2DEC * 0.75)) < 0.005) {
                output += "-M_PI*0.75";
            } else {
                // 保留两位小数
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << num;
                output += oss.str();
            }
        }
        output += "}";

        // 在终端显示转换后的数据
        std::cout << "转换后的数据：" << output << std::endl;

        // 将结果发送回客户端
        send(new_socket, output.c_str(), output.length(), 0);
    }

    // 关闭 socket
    close(new_socket);
    close(server_fd);
    std::cout << "服务器已关闭" << std::endl;

    return 0;
}