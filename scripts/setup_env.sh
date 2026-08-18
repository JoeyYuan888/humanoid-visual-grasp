#!/usr/bin/env bash
set -euo pipefail

conda create -n detect python=3.10 -y
conda activate detect

conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge
conda config --set show_channel_urls yes
conda install -n detect zbar -c conda-forge -y

echo "Install the matching CUDA PyTorch build first, then run:"
echo "pip install --no-deps -r requirements.txt"
