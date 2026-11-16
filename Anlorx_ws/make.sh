#!/usr/bin/env bash
# 将本脚本放在 ROS 2 工作空间根目录（通常与 src 目录同级）。
# 使用前，请在当前终端加载所需的 ROS 2 环境。
set -euo pipefail

usage() {
    cat <<'EOF'
用法：
  ./make.sh                         编译全部功能包
  ./make.sh 包名                    编译指定功能包及其递归依赖
  ./make.sh 包名1 包名2             编译多个功能包及其递归依赖
  ./make.sh -h | --help             显示帮助

示例：
  ./make.sh
  ./make.sh autoware_core_control

所有编译均使用 --symlink-install 和 -DCMAKE_BUILD_TYPE=Release。
脚本始终在自身所在目录执行编译。
EOF
}

if [[ $# -eq 1 && ( "$1" == "-h" || "$1" == "--help" ) ]]; then
    usage
    exit 0
fi

# 位置参数仅接受包名，避免将误传的选项交给 colcon。
for package in "$@"; do
    if [[ -z "$package" || "$package" == -* ]]; then
        printf '错误：无效包名 %q；请使用 ./make.sh --help 查看用法。\n' "$package" >&2
        exit 2
    fi
done

if ! command -v colcon >/dev/null 2>&1; then
    printf '%s\n' '错误：找不到 colcon，请确认已安装 colcon，并已加载所需的 ROS 2 环境。' >&2
    exit 127
fi

# 使用脚本目录作为工作空间，支持从其他目录调用脚本。
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$workspace_dir"

build_cmd=(
    colcon build
    --symlink-install
    --cmake-args -DCMAKE_BUILD_TYPE=Release
)

if (( $# > 0 )); then
    build_cmd+=(--packages-up-to "$@")
fi

printf '工作空间：%s\n' "$workspace_dir"
printf '执行命令：'
printf ' %q' "${build_cmd[@]}"
printf '\n'

# 直接交给 colcon 执行，保留真实退出码和终端信号处理。
exec "${build_cmd[@]}"
