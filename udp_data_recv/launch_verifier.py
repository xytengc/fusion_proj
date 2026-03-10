#!/usr/bin/env python3
"""
快速启动脚本 - UDP验证程序启动器
提供菜单式选择和参数配置
"""

import os
import sys
import subprocess
import time
from datetime import datetime

def print_header():
    print("\n" + "="*70)
    print("  UDP数据接收验证程序 - 启动器".center(70))
    print("="*70 + "\n")

def print_menu():
    print("请选择运行模式:")
    print("  1. 标准验证程序 (udp_verifier.py)")
    print("     - CPU核心绑定")
    print("     - 基础数据验证")
    print("     - 速率和丢包监测")
    print("")
    print("  2. 高级分析程序 (udp_verifier_advanced.py)")
    print("     - 实时性能监测")
    print("     - 详细验证报告")
    print("     - 准确率统计")
    print("")
    print("  3. 查看最新日志")
    print("  4. 清理旧日志")
    print("  5. 显示系统信息")
    print("  7. 仅接收并保存frame (udp_verifier.py --save-only)")
    print("  6. 退出")
    print("")

def show_system_info():
    print("\n" + "-"*70)
    print("系统信息")
    print("-"*70)
    
    # CPU信息
    try:
        os.system("echo '核心数:'; nproc")
        os.system("echo ''; echo '当前进程CPU绑定:'; taskset -p -c $$ 2>/dev/null || echo '  (未设置或无权限)'")
    except:
        print("  (无法获取)")
    
    print("")

def view_latest_log():
    """查看最新日志"""
    try:
        result = subprocess.run(
            "ls -t udp_verify*.log 2>/dev/null | head -1",
            shell=True,
            capture_output=True,
            text=True
        )
        
        if result.stdout.strip():
            latest_log = result.stdout.strip()
            print(f"\n查看日志: {latest_log}\n")
            print("-"*70)
            os.system(f"tail -n 30 {latest_log}")
            print("-"*70 + "\n")
        else:
            print("\n[提示] 没有找到日志文件")
    except Exception as e:
        print(f"\n[错误] 无法打开日志: {e}")

def clean_old_logs():
    """清理旧日志"""
    try:
        result = subprocess.run(
            "ls -t udp_verify*.log 2>/dev/null",
            shell=True,
            capture_output=True,
            text=True
        )
        
        logs = result.stdout.strip().split('\n')
        if not logs or not logs[0]:
            print("\n[提示] 没有日志文件")
            return
        
        print(f"\n找到 {len(logs)} 个日志文件:")
        for i, log in enumerate(logs, 1):
            size_result = subprocess.run(
                f"du -h {log}",
                shell=True,
                capture_output=True,
                text=True
            )
            size = size_result.stdout.split()[0]
            print(f"  {i}. {log} ({size})")
        
        keep = input(f"\n保留最近几个日志? (默认3): ").strip()
        keep = int(keep) if keep else 3
        
        if len(logs) > keep:
            to_delete = logs[keep:]
            print(f"\n将删除 {len(to_delete)} 个日志文件:")
            for log in to_delete:
                print(f"  删除: {log}")
                os.remove(log)
            print("\n[完成] 日志清理完成")
        else:
            print(f"\n[提示] 日志文件数({len(logs)}) <= 保留数({keep}), 无需清理")
    
    except Exception as e:
        print(f"\n[错误] 清理失败: {e}")

def run_verifier(version):
    """运行验证程序"""
    
    if version == 1:
        script = "udp_verifier.py"
        print("\n启动: 标准验证程序")
    else:
        script = "udp_verifier_advanced.py"
        print("\n启动: 高级分析程序")
    
    print("-"*70)
    
    if not os.path.exists(script):
        print(f"[错误] 文件不存在: {script}")
        return
    
    try:
        subprocess.run([sys.executable, script])
    except KeyboardInterrupt:
        print("\n\n程序已停止")
    except Exception as e:
        print(f"\n[错误] 运行失败: {e}")

def check_requirements():
    """检查依赖库"""
    print("\n检查依赖...")
    
    required = ['socket', 'struct', 'numpy', 'cv2', 'logging']
    
    for lib in required:
        try:
            if lib == 'cv2':
                __import__('cv2')
                print(f"  ✓ opencv-python 已安装")
            elif lib == 'numpy':
                __import__('numpy')
                print(f"  ✓ numpy 已安装")
            else:
                __import__(lib)
                print(f"  ✓ {lib} 已安装")
        except ImportError:
            print(f"  ✗ {lib} 未安装")
            if lib in ['cv2', 'numpy']:
                print(f"    请运行: pip install {('opencv-python' if lib == 'cv2' else lib)}")
    
    # 检查参考文件
    print("\n检查参考文件...")
    if os.path.exists("cycle/_AKBK0.txt"):
        size = os.path.getsize("cycle/_AKBK0.txt")
        print(f"  ✓ cycle/_AKBK0.txt ({size/1024:.1f} KB)")
    else:
        print(f"  ✗ cycle/_AKBK0.txt 不存在")
    
    if os.path.exists("cycle/video10.mp4"):
        size = os.path.getsize("cycle/video10.mp4")
        print(f"  ✓ cycle/video10.mp4 ({size/1024/1024:.1f} MB)")
    else:
        print(f"  ✗ cycle/video10.mp4 不存在")
    
    print()

def main():
    print_header()
    
    # 首次运行检查
    if not os.path.exists("udp_verifier.py"):
        print("[错误] 验证程序文件不存在!")
        print("请确保 udp_verifier.py 和 udp_verifier_advanced.py 在当前目录")
        return
    
    check_requirements()
    
    while True:
        print_menu()
        choice = input("请输入选择 (1-6): ").strip()
        
        if choice == '1':
            run_verifier(1)
        elif choice == '2':
            run_verifier(2)
        elif choice == '7':
            script = "udp_verifier.py"
            if not os.path.exists(script):
                print(f"[错误] 文件不存在: {script}")
            else:
                try:
                    subprocess.run([sys.executable, script, '--save-only'])
                except KeyboardInterrupt:
                    print("\n\n程序已停止")
                except Exception as e:
                    print(f"\n[错误] 运行失败: {e}")
        elif choice == '3':
            view_latest_log()
        elif choice == '4':
            clean_old_logs()
        elif choice == '5':
            show_system_info()
        elif choice == '6':
            print("再见!\n")
            break
        else:
            print("\n[错误] 无效的选择，请重试\n")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被中断")
        sys.exit(0)
