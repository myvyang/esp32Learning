第一步：香橙派需要外接 Intel AX210 网卡, 确认正确识别并支持 P2P-GO 模式
# 3. 检查是否支持 P2P（关键！）
iw list | grep -A 10 "Supported interface modes"
输出:
    * P2P-client
    * P2P-GO
    * P2P-device

第二步：安装 Miracast 接收器
按要求编译安装即可 https://github.com/albfan/miraclecast/wiki/Building

第三步：启用投屏

# 1. 停用 NetworkManager 对 wlan0 的控制（关键！）
sudo systemctl stop NetworkManager

# 2. 启动 miraclecast 控制端
sudo miracle-wifid &

# 3. 启动 sink（接收端）
sudo miracle-sinkctl
$ sudo miracle-sinkctl
[ADD]  Link: 3
按输出, 再执行 run 3 即可开启投屏热点.

第四步：抓视频数据
iw dev 查看手机连接的网卡
sudo tcpdump -i p2p-wlP2p33-0 -n udp 看走的服务接口是哪个

TODO: 想办法把视频流抓下来.
