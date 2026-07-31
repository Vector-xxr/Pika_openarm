#include "gripper_protocol.h"
#include <vector>

bool find_json(const std::string& msg, int& start, int& end) {
    std::vector<int> stack;
    for (size_t i = 0; i < msg.size(); ++i) {
        char ch = msg[i];
        if (ch == '{') {
            stack.push_back(static_cast<int>(i));
        } else if (ch == '}') {
            if (!stack.empty()) {
                int index = stack.back();
                stack.pop_back();
                if (stack.empty() || (index > 0 && msg[index - 1] != ':')) {
                    start = index;
                    end = static_cast<int>(i);
                    return true;
                }
            }
        }
    }
    return false;
}

int hex2dec(const std::string& str) {
    std::string s = str;
    if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        s = s.substr(2);
    }
    int num = 0;
    for (size_t i = 0; i < s.size(); ++i) {
        char ch = s[i];
        int temp = 0;
        if (ch >= '0' && ch <= '9') temp = ch - '0';
        else if (ch >= 'A' && ch <= 'F') temp = ch - 'A' + 10;
        else if (ch >= 'a' && ch <= 'f') temp = ch - 'a' + 10;
        else continue;
        num = (num << 4) | temp;
    }
    return num;
}
