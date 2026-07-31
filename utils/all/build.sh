#!/bin/bash
# data_recorder 一键编译脚本

set -e  # 出错即停止

echo "=== Data Recorder Build Script ==="

# 获取脚本所在目录（工程根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 创建构建目录（若不存在）
mkdir -p build
cd build

# 执行 CMake 配置
echo "Running CMake configuration..."
cmake ..

# 开始编译
echo "Building..."
make -j$(nproc)

echo ""
echo "Build successful! Executable located at: $SCRIPT_DIR/build/data_recorder"
echo ""
echo "Usage:"
echo "  $SCRIPT_DIR/build/data_recorder                # 自动生成 episode"
echo "  $SCRIPT_DIR/build/data_recorder my_episode    # 指定 episode 名称"