#!/usr/bin/env bash
# Set up Fast-FoundationStereo (submodule external/Fast-FoundationStereo) for the offline stereo tests
# (tools/stereo_methods.py, tools/stereo_degrade.py): its own Python venv, so ROS keeps the system Python,
# and the C-Fast-FoundationStereo weights from Hugging Face (NVIDIA Open Model Agreement).
# Writes external/ffs-venv/ and external/ffs-weights/ (both gitignored). Needs an NVIDIA GPU (CUDA 12.4 wheels).
set -e
REPO=$(cd "$(dirname "$0")/.." && pwd)
VENV=$REPO/external/ffs-venv
python3 -m venv "$VENV"
# ROS's PYTHONPATH would leak its packages into the venv.
unset PYTHONPATH
"$VENV/bin/pip" install -q --upgrade pip
"$VENV/bin/pip" install -q -r "$REPO/tools/ffs-requirements.txt"
"$VENV/bin/python" -c "import torch; assert torch.cuda.is_available(), 'no CUDA GPU'; print('torch', torch.__version__, torch.cuda.get_device_name(0))"
"$VENV/bin/python" -c "from huggingface_hub import snapshot_download; snapshot_download('nvidia/c-fast-foundationstereo', \
  local_dir='$REPO/external/ffs-weights/c-ffs', allow_patterns=['cfg.yaml', 'model_best_bp2_serialize.pth', 'README.md'])"
echo "Fast-FoundationStereo ready: $VENV/bin/python tools/stereo_methods.py --help"
