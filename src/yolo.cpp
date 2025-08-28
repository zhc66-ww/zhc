#include "yolo.hpp"

namespace yolo
{

std::tuple<uint8_t, uint8_t, uint8_t> random_color(int id)
{
    static std::tuple<uint8_t, uint8_t, uint8_t> colors[] = {
        {255,   0,   0},  // 红
        {0,   255,   0},  // 绿
        {0,     0, 255},  // 蓝
        {255, 255,   0},  // 青
        {255,   0, 255},  // 品红
        {0,   255, 255}   // 黄
    };

    return colors[id % (sizeof(colors) / sizeof(colors[0]))];
}

} // namespace yolo
