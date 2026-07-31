#pragma once
#include <cstdint>
#include <string>
#include <vector>

enum class GripperSendFlag : uint8_t {
    DISABLE = 10,
    ENABLE = 11,
    VELOCITY_CTRL = 13,
    EFFORT_CTRL = 15,
    POSITION_CTRL_MIT = 22,
    POSITION_CTRL_POS_VEL = 23
};

bool find_json(const std::string& msg, int& start, int& end);

int hex2dec(const std::string& str);

template <typename T>
std::vector<uint8_t> createBinaryCommand(uint8_t cmd, const std::vector<T>& values) {
    std::vector<uint8_t> binaryCmd;
    binaryCmd.push_back(cmd);
    for (size_t i = 0; i < values.size(); ++i) {
        T value = values[i];
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
        for (size_t b = 0; b < sizeof(T); ++b) {
            binaryCmd.push_back(bytes[b]);
        }
    }
    binaryCmd.push_back('\r');
    binaryCmd.push_back('\n');
    return binaryCmd;
}
