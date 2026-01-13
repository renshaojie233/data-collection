#!/bin/bash
#
# 视频+动作数据同步录制系统 - 启动脚本
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 初始化conda
init_conda() {
    local CONDA_PATHS=(
        "$HOME/anaconda3/bin/conda"
        "$HOME/miniconda3/bin/conda"
        "/opt/anaconda3/bin/conda"
        "/opt/miniconda3/bin/conda"
    )

    for conda_path in "${CONDA_PATHS[@]}"; do
        if [ -x "$conda_path" ]; then
            eval "$("$conda_path" shell.bash hook 2>/dev/null)"
            return 0
        fi
    done

    local CONDA_SH_PATHS=(
        "$HOME/anaconda3/etc/profile.d/conda.sh"
        "$HOME/miniconda3/etc/profile.d/conda.sh"
    )

    for conda_sh in "${CONDA_SH_PATHS[@]}"; do
        if [ -f "$conda_sh" ]; then
            source "$conda_sh"
            return 0
        fi
    done

    return 1
}

# 初始化conda
if ! init_conda; then
    echo "错误：无法找到Conda"
    exit 1
fi

# 激活环境
conda activate take_data 2>/dev/null || source activate take_data 2>/dev/null

if [ $? -ne 0 ]; then
    echo "错误：无法激活take_data环境"
    echo "请先运行：conda env create -f /home/ubuntu/take_data/take_action/environment.yml"
    exit 1
fi

# 检查依赖
python3 -c "import cv2, pyrealsense2, tkinter, h5py" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "正在安装缺失的依赖..."
    pip install -q opencv-python pyrealsense2 pillow h5py
fi

# 启动程序
echo "启动视频+动作数据同步录制系统..."
python3 video_action_recorder.py

# 如果程序关闭，保持终端打开
if [ $? -ne 0 ]; then
    echo ""
    echo "程序异常退出，按回车键退出..."
    read
fi
