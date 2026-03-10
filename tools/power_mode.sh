#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-status}"

NET_IFACE="eth0"
NET_IP="192.168.1.3"
NET_MASK="255.255.255.0"
RADAR_IP="192.168.1.10"
RADAR_MAC="01:02:03:04:05:06"

CPU_POLICY_GLOB="/sys/devices/system/cpu/cpufreq/policy*"
DEVFREQ_GLOB="/sys/class/devfreq/*"

require_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    echo "[INFO] 需要 root 权限，自动通过 sudo 重启脚本..."
    exec sudo bash "$0" "$MODE"
  fi
}

set_cpu_governor() {
  local gov="$1"
  for p in ${CPU_POLICY_GLOB}; do
    [[ -d "$p" ]] || continue
    if [[ -w "$p/scaling_governor" ]]; then
      echo "$gov" > "$p/scaling_governor"
    fi
  done
}

set_devfreq_governor_prefer() {
  local node="$1"
  shift
  local prefs=("$@")

  [[ -d "$node" ]] || return 0
  [[ -f "$node/available_governors" ]] || return 0
  [[ -w "$node/governor" ]] || return 0

  local avail
  avail="$(cat "$node/available_governors")"
  for g in "${prefs[@]}"; do
    if [[ " $avail " == *" $g "* ]]; then
      echo "$g" > "$node/governor"
      return 0
    fi
  done
}

set_performance_mode() {
  require_root
  echo "[ACTION] 切换到高性能模式..."
  set_cpu_governor "performance"

  for d in ${DEVFREQ_GLOB}; do
    [[ -d "$d" ]] || continue
    set_devfreq_governor_prefer "$d" "performance"
  done

  echo "[OK] 已切换到高性能模式"
}

set_powersave_mode() {
  require_root
  echo "[ACTION] 切换到省电模式..."
  set_cpu_governor "powersave"

  for d in ${DEVFREQ_GLOB}; do
    [[ -d "$d" ]] || continue
    local_name="$(basename "$d")"
    case "$local_name" in
      dmc)
        set_devfreq_governor_prefer "$d" "dmc_ondemand" "simple_ondemand" "powersave"
        ;;
      *npu*)
        set_devfreq_governor_prefer "$d" "rknpu_ondemand" "simple_ondemand" "powersave"
        ;;
      *gpu*|*mali*)
        set_devfreq_governor_prefer "$d" "simple_ondemand" "powersave"
        ;;
      *)
        set_devfreq_governor_prefer "$d" "powersave" "simple_ondemand"
        ;;
    esac
  done

  echo "[OK] 已切换到省电模式"
}

show_status() {
  echo "========== CPU Governor =========="
  for p in ${CPU_POLICY_GLOB}; do
    [[ -d "$p" ]] || continue
    echo "$(basename "$p"): gov=$(cat "$p/scaling_governor" 2>/dev/null || echo N/A) cur=$(cat "$p/scaling_cur_freq" 2>/dev/null || echo N/A) max=$(cat "$p/scaling_max_freq" 2>/dev/null || echo N/A)"
  done

  echo "========== DEVFREQ Governor =========="
  for d in ${DEVFREQ_GLOB}; do
    [[ -d "$d" ]] || continue
    echo "$(basename "$d"): gov=$(cat "$d/governor" 2>/dev/null || echo N/A) cur=$(cat "$d/cur_freq" 2>/dev/null || echo N/A) max=$(cat "$d/max_freq" 2>/dev/null || echo N/A)"
  done
}

configure_network() {
  require_root
  echo "[ACTION] 配置网络: ${NET_IFACE} ${NET_IP}/${NET_MASK}, ARP ${RADAR_IP} -> ${RADAR_MAC}"
  ifconfig "${NET_IFACE}" "${NET_IP}" netmask "${NET_MASK}"
  arp -s "${RADAR_IP}" "${RADAR_MAC}"
  echo "[OK] 网络配置完成"
}

usage() {
  cat <<EOF
用法:
  $(basename "$0") performance   切换高性能模式
  $(basename "$0") powersave     切换省电模式
  $(basename "$0") status        查看当前状态
  $(basename "$0") netcfg        配置IP与ARP
EOF
}

case "$MODE" in
  performance)
    set_performance_mode
    show_status
    ;;
  powersave)
    set_powersave_mode
    show_status
    ;;
  status)
    show_status
    ;;
  netcfg)
    configure_network
    ;;
  *)
    usage
    exit 1
    ;;
esac
