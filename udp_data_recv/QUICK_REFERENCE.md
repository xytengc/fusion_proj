# UDP验证程序 快速参考卡

## 快速启动

### 方式1: 使用启动器
```bash
python3 launch_verifier.py
```
显示菜单，选择要运行的程序

### 方式2: 直接运行标准版
```bash
python3 udp_verifier.py
```

### 方式3: 直接运行高级版  
```bash
python3 udp_verifier_advanced.py
```

---

## 查看日志

### 实时查看最新日志
```bash
tail -f udp_verify_*.log
```

### 查看最后30行
```bash
tail -n 30 udp_verify_*.log
```

### 搜索特定信息
```bash
grep "STATS\|ERROR" udp_verify_*.log
grep "Frame#000050" udp_verify_*.log  # 查找特定帧
```

### 查看所有日志文件
```bash
ls -lh udp_verify*.log
wc -l udp_verify*.log
```

---

## 日志管理

### 清理所有日志
```bash
rm udp_verify*.log
```

### 压缩旧日志
```bash
gzip udp_verify_*.log
```

### 删除7天前的日志
```bash
find . -name "udp_verify*.log" -mtime +7 -delete
```

---

## 性能测试

### 1. 基准测试（获取基线性能）
```bash
python3 udp_verifier.py 2>&1 | tee baseline.log
# 运行1-2分钟，按Ctrl+C停止
```

### 2. 对比测试
```bash
# 修改UDP_PORT或其他参数，再次运行
python3 udp_verifier.py 2>&1 | tee compare.log
```

### 3. 对比结果
```bash
diff <(grep "平均吞吐量" baseline.log) <(grep "平均吞吐量" compare.log)
```

---

## 常见修改

### 修改接收端口
编辑文件，修改:
```python
UDP_PORT = 32897  # 从 32896 改为 32897
```

### 修改CPU绑定核心
```python
# 绑定到核心2
os.sched_setaffinity(0, {2})

# 绑定到多个核心
os.sched_setaffinity(0, {1, 2})

# 查看当前进程绑定情况
os.sched_getaffinity(0)
```

### 修改接收缓冲区大小
```python
# 从 4MB 改为 8MB
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
```

### 修改雷达参数阈值
```python
# 增大误差容限（标准版）
if r_err > 1.0 or v_err > 1.0:  # 从0.5改为1.0
```

### 修改图像相似度阈值
```python
# 降低相似度要求（高级版）
if similarity > 80:  # 从85改为80
```

---

## 性能监测命令

### 实时监测系统资源
```bash
# 监测特定进程的CPU和内存
watch -n 1 'ps aux | grep udp_verifier'

# 使用top (实时更新)
top -p $(pgrep -f udp_verifier)
```

### 监测网络流量
```bash
# 监测UDP流量
netstat -suna | grep -E "packets|bytes"

# 使用nethogs查看进程级流量
sudo nethogs

# 使用tcpdump抓包分析（可选）
tcpdump -i eth0 -n udp port 32896
```

### 监测磁盘I/O
```bash
# 如果启用了数据保存
iostat -x 1

# 监测特定文件夹大小变化
watch 'du -sh frames/'
```

---

## 故障诊断

### 1. 连接问题
```bash
# 检查端口是否监听
netstat -anp | grep 32896

# 检查防火墙
sudo iptables -L -n | grep 32896
sudo ufw status

# 开放端口 (如需要)
sudo ufw allow 32896/udp
```

### 2. 丢包问题
```bash
# 查看系统丢包统计
netstat -suna | grep drop
ip -s link show

# 增加缓冲区
sudo sysctl -w net.core.rmem_max=256000000
sudo sysctl -w net.core.rmem_default=256000000
```

### 3. CPU问题
```bash
# 查看进程CPU占用
ps aux | grep udp_verifier

# 查看进程绑定核心
taskset -p $(pgrep -f udp_verifier)

# 查看系统核心分布
lscpu

# 检查核心频率
cat /proc/cpuinfo | grep "cpu MHz"
```

### 4. 内存泄漏检查
```bash
# 监测内存增长
watch -n 1 'ps aux | grep udp_verifier | grep -v grep'

# 使用valgrind (如安装)
valgrind --leak-check=full python3 udp_verifier.py
```

---

## 日志分析示例

### 提取统计数据
```bash
# 获取所有性能统计行
grep "\[统计\]" udp_verify_*.log

# 获取平均吞吐量
grep "平均吞吐量" udp_verify_*.log

# 获取最终统计
grep -A 10 "\[最终统计\]" udp_verify_*.log
```

### 计算平均值
```bash
# 提取吞吐量值并计算平均
grep "平均吞吐量:" udp_verify_*.log | awk '{print $NF}' | awk '{sum+=$1;n++}END{print "平均吞吐量:",sum/n,"Mbps"}'

# 提取丢包率并计算平均
grep "丢包率:" udp_verify_*.log | awk '{print $(NF-1)}' | awk '{sum+=$1;n++}END{print "平均丢包率:",sum/n,"%"}'
```

### 生成性能报告
```bash
cat > gen_report.sh << 'EOF'
#!/bin/bash
LOG=$1
echo "=== 性能分析报告 ==="
echo "日志文件: $LOG"
echo ""
echo "运行时长:"
grep "运行时长:" $LOG
echo ""
echo "性能指标:"
grep "平均吞吐量\|平均包率\|丢包率" $LOG | tail -3
echo ""
echo "验证准确率:"
grep -E "准确率|匹配|错误" $LOG | grep -v "frame\|Frame" | tail -5
EOF
chmod +x gen_report.sh
./gen_report.sh udp_verify_*.log
```

---

## 高级用法

### 1. 并行运行多个实例
```bash
# 启动两个接收器，不同的CPU核心
(python3 udp_verifier.py) &
(python3 udp_verifier_advanced.py) &
wait
```

### 2. 后台运行并重定向输出
```bash
python3 udp_verifier.py > output.log 2>&1 &
```

### 3. 定时自动运行
```bash
# 每小时运行一次验证
0 * * * * cd /home/cat/xyt/proj && python3 udp_verifier.py

# 编辑crontab
crontab -e
```

### 4. 远程运行（SSH）
```bash
ssh user@host "cd /home/cat/xyt/proj && python3 udp_verifier.py" | tee remote.log
```

### 5. Docker运行（如使用）
```bash
docker run -it --network=host -v $(pwd):/work python:3.9 \
  bash -c "cd /work && pip install numpy opencv-python && python3 udp_verifier.py"
```

---

## 故障排查快速清单

- [ ] 检查FPGA是否在发送数据
- [ ] 验证UDP端口是否正确（默认32896）
- [ ] 检查参考文件是否存在（_AKBK0.txt, video10.mp4）
- [ ] 确认网络连接正常
- [ ] 检查系统防火墙设置
- [ ] 监测CPU和内存占用
- [ ] 查看日志中的错误信息
- [ ] 确认数据格式与预期一致
- [ ] 尝试增大接收缓冲区
- [ ] 运行高级版本获取更详细信息

---

## 有用的单行命令

### 实时显示接收速率
```bash
watch -n 1 'grep "吞吐量:" udp_verify_*.log | tail -1'
```

### 计数完整帧数和丢包数
```bash
tail -f udp_verify_*.log | grep "\[最终统计\]" -A 5
```

### 提取所有Frame ID
```bash
grep "Frame#" udp_verify_*.log | awk '{print $3}' | sort -u | wc -l
```

### 查找任何错误或警告
```bash
grep -i "error\|warning\|fail" udp_verify_*.log
```

### 生成CSV格式的性能数据
```bash
grep "统计" udp_verify_*.log | awk -F'|' '{print $2","$3","$4","$5}' > perf.csv
```

---

## 快速参考表

| 项目 | 标准值 | 合理范围 | 告警值 |
|------|-------|--------|-------|
| 吞吐量 | 450 Mbps | 400-500 | <300 |
| 包率 | 32k pps | 28k-35k | <20k |
| 丢包率 | 0% | <1% | >5% |
| 延迟 | <10ms | <20ms | >50ms |
| 图像匹配度 | 95% | >90% | <80% |
| 雷达准确率 | 99% | >95% | <85% |
| CPU占用 | 10% | <30% | >50% |
| 内存占用 | 50MB | <200MB | >500MB |

---

## 获取帮助

### 查看帮助信息
```bash
python3 udp_verifier.py --help  # 如支持
python3 launch_verifier.py      # 选择5显示系统信息
```

### 查看日志
```bash
tail -f udp_verify_*.log | grep "ERROR\|WARN"
```

### 查看本README
```bash
cat UDP_VERIFIER_README.md | less
grep "故障" UDP_VERIFIER_README.md
```

---

**最后更新: 2025-02-07**
