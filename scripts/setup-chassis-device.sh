#!/usr/bin/env bash
#
# 为底盘串口设备创建稳定的 /dev/chassis 符号链接。
#
# 每次启动 /dev/ttyACM* 的编号都会变化，本脚本让用户从当前串口设备中
# 选择底盘对应的那个，然后自动：
#   1. 读取该设备的 idVendor / idProduct / serial
#   2. 生成 udev 规则 /etc/udev/rules.d/99-chassis.rules
#   3. 重载并触发 udev，生成稳定的 /dev/chassis
#
# 用法:
#   sudo ./setup-chassis-device.sh        # 交互选择并配置
#   ./setup-chassis-device.sh --list      # 仅列出设备（不需要 root）
set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-chassis.rules"
SYMLINK="chassis"

info() { printf '\033[1;36m%s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m%s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m%s\033[0m\n' "$*" >&2; }
die()  { printf '\033[1;31m%s\033[0m\n' "$*" >&2; exit 1; }

# --- 收集候选串口设备（仅 USB 串口，排除 Jetson 原生 UART ttyS*）---
candidates=()
for p in /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "$p" ]] && candidates+=("$p")
done

# --- 解析某个设备的 USB 属性（取设备树中最近的 USB 属性）---
attrs() {
    local dev=$1
    local info vid pid serial
    info=$(udevadm info -a -n "$dev" 2>/dev/null) || return 1
    vid=$(grep -oP 'ATTRS\{idVendor\}=="\K[0-9a-fA-F]{4}' <<<"$info" | head -1 || true)
    pid=$(grep -oP 'ATTRS\{idProduct\}=="\K[0-9a-fA-F]{4}' <<<"$info" | head -1 || true)
    serial=$(grep -oP 'ATTRS\{serial\}=="\K[^"]+' <<<"$info" | head -1 || true)
    printf '%s\t%s\t%s\n' "${vid:-}" "${pid:-}" "${serial:-}"
}

# --- 列出所有设备 ---
list_devices() {
    if [[ ${#candidates[@]} -eq 0 ]]; then
        warn "未发现 USB 串口设备 (/dev/ttyACM* /dev/ttyUSB*)。"
        warn "请确认底盘已上电、USB 线已连接，然后重试。"
        return 1
    fi
    info "检测到的串口设备:"
    local i dev a
    for i in "${!candidates[@]}"; do
        dev=${candidates[$i]}
        a=$(attrs "$dev" || true)
        IFS=$'\t' read -r vid pid serial <<<"$a"
        printf '  [%d] %-14s vendor=%-5s product=%-5s serial=%s\n' \
            "$((i + 1))" "$dev" "${vid:-?}" "${pid:-?}" "${serial:-<空>}"
    done
}

# --- 选择设备 ---
choose_device() {
    [[ ${#candidates[@]} -gt 0 ]] || die "未发现串口设备，请先连接底盘。"
    list_devices
    local choice n
    while :; do
        read -rp "选择底盘对应的设备编号 [1-${#candidates[@]}，输入 q 退出]: " choice
        [[ "$choice" == "q" ]] && exit 0
        n=$((choice))
        if [[ "$choice" =~ ^[0-9]+$ ]] && ((n >= 1 && n <= ${#candidates[@]})); then
            CHOSEN=${candidates[$((n - 1))]}
            break
        fi
        warn "无效输入: $choice"
    done
}

# --- 生成 udev 规则 ---
write_rule() {
    local dev=$1 vid pid serial
    IFS=$'\t' read -r vid pid serial <<<"$(attrs "$dev")"
    [[ -n "$vid" && -n "$pid" ]] || die "无法读取 $dev 的 idVendor/idProduct，无法生成规则。"

    ok "所选设备: $dev  vendor=$vid product=$pid serial=${serial:-<空>}"
    read -rp "确认使用这个设备？[Y/n] " yn
    [[ "${yn:-Y}" =~ ^[Yy]$ ]] || { warn "已取消。"; exit 0; }

    local rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\""
    if [[ -n "$serial" ]]; then
        rule+=", ATTRS{serial}==\"$serial\""
    else
        warn "注意: 该设备没有序列号，规则只按 vendor/product 匹配；若同型号设备有多个可能误匹配。"
    fi
    rule+=", SYMLINK+=\"$SYMLINK\", MODE=\"0666\""

    info "写入 udev 规则:"
    echo "  $rule" >&2
    printf '%s\n' "$rule" > "$RULE_FILE"
    chmod 644 "$RULE_FILE"
}

# --- 应用规则 ---
apply_rule() {
    info "重载 udev 规则并触发..."
    udevadm control --reload-rules
    udevadm trigger
    sleep 1

    if [[ -L "/dev/$SYMLINK" || -e "/dev/$SYMLINK" ]]; then
        ok "成功: /dev/$SYMLINK -> $(readlink -f "/dev/$SYMLINK")"
    else
        warn "/dev/$SYMLINK 尚未生成，尝试拔插 USB 线，或检查 dmesg。"
    fi
}

# --- 主流程 ---
if [[ "${1:-}" == "--list" ]]; then
    list_devices
    exit 0
fi

[[ $EUID -eq 0 ]] || die "需要 root 权限，请用: sudo $0"

choose_device
write_rule "$CHOSEN"
apply_rule

# 可选: 验证驱动
if command -v chassis-driver >/dev/null 2>&1; then
    read -rp "是否用 chassis-driver 验证 /dev/$SYMLINK? [Y/n] " yn
    if [[ "${yn:-Y}" =~ ^[Yy]$ ]]; then
        chassis-driver --serial-port "/dev/$SYMLINK" status || \
            warn "验证失败（若 chassis-driver 不在 PATH，可手动运行）。"
    fi
else
    ok "完成。以后用: chassis-driver --serial-port /dev/$SYMLINK ..."
fi
