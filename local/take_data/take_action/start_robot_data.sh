#!/bin/bash
#
# One-Click Robot Data Collection Launcher
# 一键启动机器人数据采集系统
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Banner
clear
echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                                                            ║"
echo "║         机器人数据采集系统 - 一键启动                      ║"
echo "║       Robot Data Collection System - Quick Start          ║"
echo "║                                                            ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""

# Initialize conda
init_conda() {
    # Try multiple conda locations
    local CONDA_PATHS=(
        "$HOME/anaconda3/bin/conda"
        "$HOME/miniconda3/bin/conda"
        "/opt/anaconda3/bin/conda"
        "/opt/miniconda3/bin/conda"
        "$(which conda 2>/dev/null)"
    )

    for conda_path in "${CONDA_PATHS[@]}"; do
        if [ -x "$conda_path" ]; then
            # Found conda, initialize it
            eval "$("$conda_path" shell.bash hook 2>/dev/null)"
            return 0
        fi
    done

    # Try sourcing conda.sh
    local CONDA_SH_PATHS=(
        "$HOME/anaconda3/etc/profile.d/conda.sh"
        "$HOME/miniconda3/etc/profile.d/conda.sh"
        "/opt/anaconda3/etc/profile.d/conda.sh"
        "/opt/miniconda3/etc/profile.d/conda.sh"
    )

    for conda_sh in "${CONDA_SH_PATHS[@]}"; do
        if [ -f "$conda_sh" ]; then
            source "$conda_sh"
            return 0
        fi
    done

    return 1
}

# Check conda
echo -e "${YELLOW}[1/4] 检查Conda环境...${NC}"
if ! init_conda; then
    echo -e "${RED}✗ 无法找到Conda${NC}"
    echo -e "${YELLOW}提示：请确保已安装Anaconda或Miniconda${NC}"
    exit 1
fi

# Verify conda is working
if ! command -v conda &> /dev/null; then
    echo -e "${RED}✗ Conda初始化失败${NC}"
    exit 1
fi

echo -e "${GREEN}  ✓ Conda已初始化${NC}"

# Check if environment exists
if ! conda env list | grep -q "^take_data "; then
    echo -e "${YELLOW}  未找到take_data环境，正在创建...${NC}"
    echo -e "${YELLOW}  这可能需要几分钟，请耐心等待...${NC}"
    conda env create -f environment.yml -q
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}  ✓ 环境创建成功${NC}"
    else
        echo -e "${RED}  ✗ 环境创建失败${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}  ✓ 环境已存在${NC}"
fi

# Activate environment
echo -e "${YELLOW}[2/4] 激活Conda环境...${NC}"
conda activate take_data 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}  ✓ 环境已激活: take_data${NC}"
else
    # Try alternative activation method
    source activate take_data 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}  ✓ 环境已激活: take_data${NC}"
    else
        echo -e "${RED}  ✗ 环境激活失败${NC}"
        exit 1
    fi
fi

# Check Python packages
echo -e "${YELLOW}[3/4] 检查Python依赖...${NC}"
python3 -c "import numpy, h5py" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}  ✓ 依赖已安装${NC}"
else
    echo -e "${YELLOW}  部分依赖缺失，正在安装...${NC}"
    pip install -q numpy h5py pandas matplotlib pyyaml tqdm
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}  ✓ 依赖安装成功${NC}"
    else
        echo -e "${RED}  ✗ 依赖安装失败${NC}"
        exit 1
    fi
fi

# Check sshpass
if ! command -v sshpass &> /dev/null; then
    echo -e "${YELLOW}  sshpass未安装，正在安装...${NC}"
    echo "1" | sudo -S apt install -y sshpass -qq > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}  ✓ sshpass安装成功${NC}"
    else
        echo -e "${RED}  ✗ sshpass安装失败，请手动运行: sudo apt install sshpass${NC}"
        exit 1
    fi
fi

# Check remote connection
echo -e "${YELLOW}[4/4] 检查远程连接...${NC}"
if ping -c 1 -W 2 172.16.1.2 > /dev/null 2>&1; then
    echo -e "${GREEN}  ✓ 远程主机 172.16.1.2 可访问${NC}"
else
    echo -e "${RED}  ✗ 无法连接到远程主机 172.16.1.2${NC}"
    echo -e "${YELLOW}  请检查网络连接${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  所有检查通过！正在启动数据采集系统...                     ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
sleep 1

# Launch the collection system
cd "$SCRIPT_DIR/scripts"
exec ./start_collection.sh
