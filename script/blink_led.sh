#!/bin/bash
###
 # @Author: bangbang 1789228622@qq.com
 # @Date: 2024-12-10 21:15:42
 # @LastEditors: bangbang 1789228622@qq.com
 # @LastEditTime: 2000-01-01 09:01:02
 # @FilePath: /success2025/script/blink_led.sh
 # @Description: 
 # 
 # Copyright (c) 2024 by CDTU-Success, All Rights Reserved. 
### 

# 定义 LED 的 brightness 文件路径
LED_PATH="/sys/class/leds/pwr/brightness"

# 检查是否有权限访问 brightness 文件
if [ ! -w "$LED_PATH" ]; then
    echo "需要超级用户权限运行此脚本。"
    exit 1
fi

# 定义闪烁频率（单位：秒）
ON_TIME=0.25   # 点亮时间
OFF_TIME=0.25  # 熄灭时间

# # 并行执行同一文件夹下的 setup.sh
# SCRIPT_DIR=$(dirname "$0")  # 获取脚本所在目录
# SETUP_SCRIPT="$SCRIPT_DIR/setup.sh"

# if [ -x "$SETUP_SCRIPT" ]; then
#     "$SETUP_SCRIPT" &  # 后台运行 setup.sh
# else
#     echo "setup.sh 不存在或不可执行。"
#     exit 1
# fi

# 开始闪烁
while true; do
    echo 255 > "$LED_PATH"  # 打开 LED（亮度设置为 255）
    sleep "$ON_TIME"
    echo 0 > "$LED_PATH"   # 关闭 LED
    sleep "$OFF_TIME"
done