#!/bin/sh
set -eu
ROBOTWIN_MAC_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
export PYTHONPATH="$ROBOTWIN_MAC_ROOT:$ROBOTWIN_MAC_ROOT/vendor/RoboTwin${PYTHONPATH:+:$PYTHONPATH}"
export VK_ICD_FILENAMES=/opt/homebrew/opt/molten-vk/etc/vulkan/icd.d/MoltenVK_icd.json
export DYLD_LIBRARY_PATH="/opt/homebrew/opt/vulkan-loader/lib:/opt/homebrew/opt/molten-vk/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export PYTHONUNBUFFERED=1
export ROBOTWIN_MAC_PLANNER="${ROBOTWIN_MAC_PLANNER:-metal}"
if [ "$ROBOTWIN_MAC_PLANNER" = metal ]; then
    export PYTORCH_ENABLE_MPS_FALLBACK=0
fi
cd "$ROBOTWIN_MAC_ROOT/vendor/RoboTwin"
exec "$ROBOTWIN_MAC_ROOT/.venv/bin/python" "$@"
