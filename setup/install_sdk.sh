#!/usr/bin/env bash
#
# Install the Metavision SDK Pro (source build) + build the evk4_sdk_advanced
# package. This is the OPTIONAL advanced layer; the base stack comes from
# install_deps.sh. See docs/sdk/access.md (token) and docs/sdk/install.md.
#
# Two tiers, selected by a flag:
#   (default)  LITE  -> model-free + edgelet pipelines. No GPU. Raspberry Pi, or
#                       any x86/ARM box without CUDA.  (USE_TORCH=OFF, USE_SOPHUS=ON)
#   --ml       FULL  -> also the ML/GPU pipelines (gesture/detection/flow). Needs
#                       a CUDA GPU + LibTorch.  Jetson / x86 + NVIDIA.
#                       (adds USE_TORCH=ON + the model zoo)
#
# The SDK is source-built in BOTH tiers (ARM has no apt binaries, and even on x86
# the ML module needs a Torch-enabled source build), so the only difference is
# Torch. The script does NOT install the GPU driver / CUDA toolkit (platform-
# specific, needs reboots) -- for --ml it CHECKS for nvcc + LibTorch and stops
# with instructions if they are missing.
#
# Prerequisites:
#   - A Prophesee JFrog token at ~/.config/prophesee/jfrog_token  (docs/sdk/access.md)
#   - Your login exported:  export PROPHESEE_USER=you@customers.prophesee.ai
#     (or add a PROPHESEE_USER=... line to the token file)
#   - The base stack already installed (setup/install_deps.sh)
#   - For --ml: CUDA on PATH (nvcc) and CUDA LibTorch; point TORCH_DIR at it
#     (default ~/libtorch), e.g. export TORCH_DIR=$HOME/libtorch/share/cmake/Torch
#
# Usage:
#   export ROS_DISTRO=jazzy
#   ./setup/install_sdk.sh                 # LITE
#   ./setup/install_sdk.sh --ml            # FULL (GPU)
#   ./setup/install_sdk.sh --version 5.3.1 # pin the SDK version (default 5.3.1)

set -euo pipefail

MODE=lite
SDK_VER=5.3.1
SKIP_APT=0
while [ $# -gt 0 ]; do
  case "$1" in
    --ml|--full) MODE=full ;;
    --version) SDK_VER="${2:?--version needs a value}"; shift ;;
    --version=*) SDK_VER="${1#*=}" ;;
    --skip-apt) SKIP_APT=1 ;;   # assume build deps already present (no sudo)
    -h|--help) sed -n '2,40p' "$0"; exit 0 ;;
    *) echo "ERROR: unknown argument '$1' (try --help)" >&2; exit 2 ;;
  esac
  shift
done

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ERROR: ROS_DISTRO is not set. Run: export ROS_DISTRO=jazzy" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
OVERLAY_WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"   # this repo's workspace (~/ros2_ws)
SRC_DIR="$HOME/metavision_src"
SDK_SRC="$SRC_DIR/openeb-$SDK_VER"
TOKEN_FILE="$HOME/.config/prophesee/jfrog_token"
SOPHUS_DIR="/opt/ros/$ROS_DISTRO/share/sophus/cmake"
BASE='https://propheseeai.jfrog.io/artifactory/metavision-sdk-5-archives-nc/main/sources'

echo "Metavision SDK install: tier=$MODE, sdk=$SDK_VER, ros=$ROS_DISTRO"

# --- prerequisite checks (fail early and clearly) -------------------------------
# Source the token file if present (may set PROPHESEE_JFROG_TOKEN + PROPHESEE_USER).
# shellcheck disable=SC1090
[ -f "$TOKEN_FILE" ] && source "$TOKEN_FILE"

# Which source archives are needed, and which are already downloaded?
archives=( "metavision_open_full_$SDK_VER.tar.gz" "metavision_sdk_advanced_sources_$SDK_VER.tar.gz" )
[ "$MODE" = full ] && archives+=( "metavision_sdk_ml_models_$SDK_VER.tar.gz" )
missing=()
for f in "${archives[@]}"; do [ -f "$SRC_DIR/$f" ] || missing+=( "$f" ); done

# A JFrog token + login are needed ONLY to download missing archives. A box that
# already has the sources (e.g. a credential-less runtime machine) builds without.
if [ "${#missing[@]}" -gt 0 ]; then
  [ -n "${PROPHESEE_JFROG_TOKEN:-}" ] || {
    echo "ERROR: ${#missing[@]} SDK archive(s) must be downloaded but no JFrog token found." >&2
    echo "       Put one at $TOKEN_FILE (see docs/sdk/access.md)." >&2; exit 1; }
  [ -n "${PROPHESEE_USER:-}" ] || {
    echo "ERROR: set your login -- export PROPHESEE_USER=you@customers.prophesee.ai" >&2
    echo "       (or add a PROPHESEE_USER=... line to $TOKEN_FILE)." >&2; exit 1; }
fi

# The full tier needs CUDA + LibTorch; the script does NOT install them.
if [ "$MODE" = full ]; then
  [ -d /usr/local/cuda/bin ] && export PATH="/usr/local/cuda/bin:$PATH"  # common CUDA location
  command -v nvcc >/dev/null 2>&1 || {
    echo "ERROR: --ml needs CUDA, but nvcc is not on PATH." >&2
    echo "       Install an NVIDIA driver + CUDA toolkit and add it to PATH" >&2
    echo "       (e.g. export PATH=/usr/local/cuda/bin:\$PATH). See docs/sdk/install.md." >&2
    exit 1; }
  TORCH_DIR="${TORCH_DIR:-$HOME/libtorch/share/cmake/Torch}"
  [ -f "$TORCH_DIR/TorchConfig.cmake" ] || {
    echo "ERROR: --ml needs CUDA LibTorch, not found at TORCH_DIR=$TORCH_DIR." >&2
    echo "       Download a CUDA LibTorch matching the SDK's Torch version and set" >&2
    echo "       TORCH_DIR=<libtorch>/share/cmake/Torch. See docs/sdk/install.md." >&2
    exit 1; }
  echo "  full tier: CUDA $(nvcc --version | sed -n 's/.*release \([0-9.]*\).*/\1/p' | head -1), Torch_DIR=$TORCH_DIR"
fi

# --- 1. build dependencies (apt) -----------------------------------------------
if [ "$SKIP_APT" = 1 ]; then
  echo "[1/5] skipping apt build dependencies (--skip-apt; assuming they are present)"
else
  echo "[1/5] apt: SDK build dependencies (+ ros-$ROS_DISTRO-sophus for cv3d)..."
  sudo apt-get update
  sudo apt-get install -y \
    build-essential cmake git wget unzip curl software-properties-common \
    libopencv-dev libboost-all-dev libusb-1.0-0-dev libprotobuf-dev protobuf-compiler \
    libeigen3-dev libceres-dev libhdf5-dev hdf5-tools libglew-dev libglfw3-dev \
    libcanberra-gtk-module ffmpeg libgtest-dev libgmock-dev libogre-1.12-dev \
    libimgui-dev libfreetype-dev python3-dev python3-pip \
    "ros-$ROS_DISTRO-sophus"
fi

# --- 2. download missing SDK source archives -----------------------------------
mkdir -p "$SRC_DIR"; cd "$SRC_DIR"
if [ "${#missing[@]}" -gt 0 ]; then
  echo "[2/5] download SDK sources (${#missing[@]} archive(s))..."
  for f in "${missing[@]}"; do
    echo "  fetching $f"
    curl -fSL -u "$PROPHESEE_USER:$PROPHESEE_JFROG_TOKEN" -O "$BASE/$f"
  done
else
  echo "[2/5] all SDK archives already present -- skipping download."
fi

# --- 3. extract and merge (Advanced overlays OpenEB; idempotent) ---------------
# Guarded so a re-run does not re-write source files (which would bump mtimes and
# trigger a full recompile). The `ml` module marks the Advanced overlay present.
echo "[3/5] extract..."
[ -d "$SDK_SRC" ] || tar -xzf "metavision_open_full_$SDK_VER.tar.gz"
[ -d "$SDK_SRC/sdk/modules/ml" ] || \
  tar -xzf "metavision_sdk_advanced_sources_$SDK_VER.tar.gz" -C "$SDK_SRC"
if [ "$MODE" = full ] && [ ! -d "$SDK_SRC/sdk/modules/ml/models" ]; then
  tar -xzf "metavision_sdk_ml_models_$SDK_VER.tar.gz" -C "$SDK_SRC/sdk/modules/"
fi

# --- 4. configure + build the SDK ----------------------------------------------
echo "[4/5] build the SDK ($MODE tier -- this takes a while)..."
mkdir -p "$SDK_SRC/build"; cd "$SDK_SRC/build"
cmake_args=(
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  -DCOMPILE_METAVISION_STUDIO=OFF -DCOMPILE_PYTHON3_BINDINGS=OFF
  -DUSE_SOPHUS=ON -DSophus_DIR="$SOPHUS_DIR"
)
if [ "$MODE" = full ]; then
  export PATH="/usr/local/cuda/bin:$PATH"          # nvcc, for CUDA-language detection
  cmake_args+=( -DUSE_TORCH=ON -DTorch_DIR="$TORCH_DIR" )
else
  cmake_args+=( -DUSE_TORCH=OFF )
fi
cmake .. "${cmake_args[@]}"
cmake --build . -- -j"$(nproc)" -k

# --- 5. build the evk4_sdk_advanced package ------------------------------------
echo "[5/5] build evk4_sdk_advanced ($OVERLAY_WS)..."
set +u   # ROS setup scripts reference unbound vars
source "/opt/ros/$ROS_DISTRO/setup.bash"
[ -f "$HOME/workspaces/3rd_party_ws/install/setup.bash" ] && \
  source "$HOME/workspaces/3rd_party_ws/install/setup.bash"
set -u
cd "$OVERLAY_WS"
pkg_args=(
  -DMetavisionSDK_DIR="$SDK_SRC/build/generated/share/cmake/MetavisionSDKCMakePackagesFilesDir"
  -DSophus_DIR="$SOPHUS_DIR"
)
if [ "$MODE" = full ]; then
  export LD_LIBRARY_PATH="$(dirname "$(dirname "$(dirname "$TORCH_DIR")")")/lib:${LD_LIBRARY_PATH:-}"
  pkg_args+=( -DTorch_DIR="$TORCH_DIR" )
fi
colcon build --packages-select evk4_sdk_advanced --cmake-args "${pkg_args[@]}"

echo
echo "Done ($MODE tier). Open a new terminal (or 'source $OVERLAY_WS/install/setup.bash'), then:"
echo "    ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=optical_flow params_file:=\$HOME/my_params.yaml"
if [ "$MODE" = lite ]; then
  echo "    (ML pipelines were skipped -- re-run with --ml on a CUDA box to add them.)"
fi
