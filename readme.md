# Usage

编译opencv_cuda

首先编译文件

```shell
cd success2025/build
cmake .. && make -j6
```

使用busybox 修改寄存器的值

```shell
sudo apt-get install busybox
busybox devmem 0x0c303018 w 0xc458
busybox devmem 0x0c303010 w 0xc400
busybox devmem 0x0c303008 w 0xc458
busybox devmem 0x0c303000 w 0xc400
```

挂载CAN 内核

```shell
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```

CAN 属性设置
例如将CAN0 波特率设置成1000k

```shell
sudo ip link set down can0
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

发送数据ID 123 数据1122334455667788

```shell
sudo cansend can0 123#1122334455667788
```

要在 Linux 中永久挂载和加载这些内核模块（`can`，`can_raw`，`mttcan`），可以将它们添加到系统的模块加载配置中。以下是具体步骤：

### 1. 编辑模块加载配置文件

将这些模块写入系统的模块配置文件，例如 `/etc/modules-load.d/can.conf`。该文件会在系统启动时加载指定的模块。

#### 创建或编辑配置文件

```shell
sudo vim /etc/modules-load.d/can.conf
```

#### 添加以下内容

```shell
can
can_raw
mttcan
```

保存并退出。

------

### 2. 更新系统的模块加载

运行以下命令以确保配置生效：

```shell
sudo systemctl restart systemd-modules-load.service
```

------

### 3. 确认模块已加载

使用以下命令检查模块是否已加载：

```shell
lsmod | grep -E 'can|mttcan'
```

### 4. 如果需要额外参数

如果加载模块时需要设置参数，可以将参数写入 `/etc/modprobe.d/can.conf`。

创建或编辑参数文件

```shell
sudo vim /etc/modprobe.d/can.conf
```

添加参数，例如：

```shell
options mttcan bitrate=1000000
```

保存并退出。

### 5.开机自动重启

创建`rc-local.service`文件：

```shell
sudo vim /etc/systemd/system/rc-local.service
```

在文件中添加以下内容：

```shell
[Unit]
Description=/etc/rc.local Compatibility
ConditionPathExists=/etc/rc.local

[Service]
ExecStart=/etc/rc.local start
Type=forking
TimeoutSec=0
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

创建`rc.local`文件：

```shell
sudo vim /etc/rc.local
```

在`rc.local`文件中添加要执行的命令或脚本。
保存并关闭文件。

设置`rc.local`文件的权限：

```shell
sudo chmod +x /etc/rc.local
```

启用`rc-local.service`：

```shell
sudo systemctl enable rc-local.service
```

启动`rc-local.service`：

```shell
sudo systemctl start rc-local.service
```

# ARM Linux CAN 异常排查步骤

[ARM Linux CAN 异常排查步骤_linux can错误-CSDN博客](https://blog.csdn.net/aiyanzielf/article/details/111708394)

```shell
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
sudo candump can0
```

**过滤特定 CAN ID**
如果只想监听特定的 CAN ID（例如 `123`），可以使用过滤功能：

```shell
sudo candump can0,123:7FF
```

输出示例：

```shell
can0  123   [3]  11 22 03
```

- `can0`：接口名称。
- `123`：CAN ID（十六进制表示）。
- `[3]`：数据长度（3 个字节）。
- `11 22 03`：数据内容（每字节以十六进制显示）。

```shell
sudo chmod 777 /dev/ttyTHS0
```

```
TensorRT /usr/src/tensorrt/samples/
CUDA /usr/local/cuda10.2/samples/
cuDNN /usr/src/cudnn_samples_v8/
VisionWorks /usr/share/visionworks/sources/samples//
usr/share/visionworks-tracking/sources/samples//
usr/share/visionworks-sfm/sources/samples/ 
OpenCV /usr/share/opencv4/samples/
```

