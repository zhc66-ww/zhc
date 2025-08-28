#include "uart.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace com;

int main()
{
    // 创建串口对象
    my_uart uart;

    // 初始化串口
    uart.UART_INIT();

    // 打开串口
    uart.UART_OPEN();
    
    // 检查是否打开成功 (flag 需要你类里面有 getter 或 public flag)
    // 假设 flag 是 public 或有函数可以访问
    // if(!uart.flag) return -1;

    // 配置串口（波特率、数据位、停止位、校验位）
    uart.UART_CONFIG();

    // 发送数据
    const char* msg = "Hello World";
    uart.UART_SEND(reinterpret_cast<const uint8_t*>(msg), strlen(msg));

    // 等待一会儿接收数据
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 接收数据
    uart.UART_RECEIVE();

    // 关闭串口
    uart.UART_CLOSE();

    return 0;
}
