# sleep 5
###
 # @Author: bangbang 1789228622@qq.com
 # @Date: 2024-12-05 21:18:06
 # @LastEditors: bangbang 1789228622@qq.com
 # @LastEditTime: 2025-01-05 19:13:09
 # @FilePath: /success2025/script/setup.sh
 # @Description: 
 # 
 # Copyright (c) 2024 by CDTU-Success, All Rights Reserved. 
### 
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
sleep 1
sudo chmod 777 /dev/ttyTHS0 #debug
sleep 1
sudo chmod 777 /dev/ttyTHS1 #ToStm32
# sleep 5
# sudo /home/gyxy/Desktop/workspeaseMY/success2025/build/RM_exe --headless --bitrate=4000000