# Installing the Metavision SDK

You need a Prophesee account and identity token first — see
[access.md](access.md). Then pick your platform:

- **x86_64** → apt binaries (fast). [Jump to x86.](#x86_64-apt)
- **ARM (Raspberry Pi 5, Jetson)** → build from source (no ARM binaries exist).
  [Jump to ARM.](#arm-build-from-source)

Either way, the rest is identical ([README.md](README.md)) — `evk4_sdk_advanced`
finds an **apt** SDK automatically; a **source** build (ARM, or the ML / cv3d
tiers) needs `-DMetavisionSDK_DIR=` pointed at its build tree.

Set your distro once:

```bash
export ROS_DISTRO=jazzy
```

## x86_64 (apt)

> The steps below are the standard Prophesee apt flow for x86_64.

Install the repository signing key:

```bash
sudo apt -y install curl
curl -L https://propheseeai.jfrog.io/artifactory/api/security/keypair/prophesee-gpg/public \
  > /tmp/propheseeai.jfrog.asc
sudo cp /tmp/propheseeai.jfrog.asc /etc/apt/trusted.gpg.d/
```

Add the repository. Free / non-commercial accounts (the EVK4-owner case) use the
**`-nc`** repositories; list what your token can read to be sure:

```bash
source ~/.config/prophesee/jfrog_token       # exports $PROPHESEE_JFROG_TOKEN (see access.md)
curl -s -u "<USER>:$PROPHESEE_JFROG_TOKEN" \
  https://propheseeai.jfrog.io/artifactory/api/repositories
```

```bash
sudo add-apt-repository \
  'https://<USER>:<TOKEN>@propheseeai.jfrog.io/artifactory/metavision-sdk-5-debian-nc/'
sudo apt update
sudo apt -y install metavision-sdk
```

> **Encode the `@` in your login.** `<USER>` is an email, so the `@` must be
> `%40` inside the URL (`you%40customers.prophesee.ai`) or apt misparses it.

`metavision-sdk` is a metapackage delivering all modules. Verify:

```bash
apt list --installed 2>/dev/null | grep metavision
```

**Next:** [README.md](README.md) — build the package and run a pipeline. (The apt
SDK covers the model-free pipelines; the ML/GPU tier needs a source build with
Torch — see [ML pipelines (GPU, x86)](#ml-pipelines-gpu-x86) below.)

## ARM (build from source)

Validated on a **Raspberry Pi 5 (16 GB), Ubuntu 24.04, ROS 2 Jazzy**: the full
build took ~22 minutes with zero errors. The same procedure
is expected to work on a Jetson (aarch64), which is not tested here.

### 1. Download the source archives

The SDK source lives in the **`-archives-nc`** repository. Download the two you
need (OpenEB + the closed Advanced modules) using your token — you do **not**
need the 522 MB ML-models archive for optical flow:

```bash
source ~/.config/prophesee/jfrog_token        # $PROPHESEE_JFROG_TOKEN
U='<your-login>@customers.prophesee.ai'
BASE='https://propheseeai.jfrog.io/artifactory/metavision-sdk-5-archives-nc/main/sources'
mkdir -p ~/metavision_src && cd ~/metavision_src
curl -fSL -u "$U:$PROPHESEE_JFROG_TOKEN" -O "$BASE/metavision_open_full_5.3.1.tar.gz"
curl -fSL -u "$U:$PROPHESEE_JFROG_TOKEN" -O "$BASE/metavision_sdk_advanced_sources_5.3.1.tar.gz"
```

(Adjust `5.3.1` to the latest version your account lists. Credentials in a `curl
-u` argument need no URL-encoding, unlike the apt URL above.)

### 2. Install build dependencies

```bash
sudo apt update
sudo apt -y install build-essential cmake git wget unzip curl software-properties-common \
  libopencv-dev libboost-all-dev libusb-1.0-0-dev libprotobuf-dev protobuf-compiler \
  libeigen3-dev libceres-dev libhdf5-dev hdf5-tools libglew-dev libglfw3-dev \
  libcanberra-gtk-module ffmpeg libgtest-dev libgmock-dev libogre-1.12-dev \
  libimgui-dev libfreetype-dev python3-dev python3-pip
```

### 3. Extract and merge

The Advanced archive overlays the OpenEB tree, so extract OpenEB first, then the
Advanced sources **into** it:

```bash
cd ~/metavision_src
tar -xzf metavision_open_full_5.3.1.tar.gz                          # -> openeb-5.3.1/
tar -xzf metavision_sdk_advanced_sources_5.3.1.tar.gz -C openeb-5.3.1/
```

### 4. Configure and build (lean)

Disable the parts the optical-flow pipeline does not need, which also drops
the heaviest ARM dependencies:

- `-DCOMPILE_METAVISION_STUDIO=OFF` — **required** on ARM (Studio is amd64-only).
- `-DUSE_TORCH=OFF` — skip the ML module (needs LibTorch; the Pi has no CUDA).
- `-DUSE_SOPHUS=OFF` — skip the `cv3d` module (geometric/3D algorithms; enable it
  for the edgelet pipeline — see the cv3d tier below).
- `-DCOMPILE_PYTHON3_BINDINGS=OFF`, `-DBUILD_TESTING=OFF` — not needed here.

```bash
cd ~/metavision_src/openeb-5.3.1
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DUSE_TORCH=OFF -DUSE_SOPHUS=OFF \
  -DCOMPILE_METAVISION_STUDIO=OFF -DCOMPILE_PYTHON3_BINDINGS=OFF
cmake --build . -- -j"$(nproc)" -k
```

> The configure step warns `Could NOT find METIS` — harmless (an optional Ceres
> extra that optical flow does not use).

**No install step.** The build tree exports usable CMake config files, so
`evk4_sdk_advanced` builds directly against it — see
[README.md](README.md). (Do not delete `~/metavision_src`; the
package links against the libraries there.) A `sudo cmake --build . --target
install` is **not** recommended: it aborts on a hardcoded `/etc/udev` step that
needs root, and a system install risks colliding with `openeb_vendor`.

**Next:** [README.md](README.md) — build the package and run a pipeline.

## ML pipelines (GPU, x86)

The neural-network pipelines (gesture / detection / flow inference) need the SDK
`ml` module built **against LibTorch with CUDA** — which the **apt binaries do not
provide**. So even on x86 the ML tier requires a **source SDK build** with
`-DUSE_TORCH=ON` (the same source build as the ARM section above, plus Torch); the
apt path covers only the model-free pipelines. The lean Pi build skips the ML tier
automatically.

1. **Build the SDK with Torch.** Re-run the source build above with a CUDA
   LibTorch: install an NVIDIA driver + CUDA toolkit (e.g. `cuda-toolkit-12-6`),
   download and unzip CUDA LibTorch (e.g. `cu126`, version matching the SDK's —
   2.9.1), then configure with `-DUSE_TORCH=ON
   -DTorch_DIR=<libtorch>/share/cmake/Torch/` (add `-DCOMPILE_PYTHON3_BINDINGS=ON`
   for the Python samples). Validated on a 2× RTX 2080 Ti box (Ubuntu 24.04,
   CUDA 12.6, LibTorch 2.9.1).
2. **Get the pretrained models.** Download `metavision_sdk_ml_models_<ver>.tar.gz`
   from the JFrog `metavision-sdk-5-archives-nc/main/sources` folder and extract
   it into `<src>/sdk/modules/` (it lands as `ml/models/` and `core_ml/models/`).
3. **Build the `evk4_sdk_advanced` ML tier.** Point CMake at both the SDK and
   LibTorch, with `nvcc` on PATH (Torch enables CMake's CUDA language):
   ```bash
   cd ~/ros2_ws
   export PATH=/usr/local/cuda/bin:$PATH          # nvcc, for CUDA-language detection
   export LD_LIBRARY_PATH=$HOME/libtorch/lib:$LD_LIBRARY_PATH
   colcon build --packages-select evk4_sdk_advanced --cmake-args \
     -DMetavisionSDK_DIR=<src>/build/generated/share/cmake/MetavisionSDKCMakePackagesFilesDir \
     -DTorch_DIR=$HOME/libtorch/share/cmake/Torch
   ```
   CMake auto-generates the `hdf5_ecf` config the no-install SDK build doesn't
   export, then builds the ML pipelines (gesture/…); without Torch they are
   skipped and the model-free pipelines build as usual.

Run one with its model on the GPU — set `model_path` (the `.ptjit`) and `gpu_id`
in a node-params YAML (see [pipelines.md](pipelines.md) for the full ML param set):
```bash
cat > /tmp/ml.yaml <<'YAML'
/**:
  ros__parameters:
    model_path: <MODELS>/classification/convRNN_chifoumi/rnn_model_classifier.ptjit
    gpu_id: 0
YAML
ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=gesture \
    params_file:=$HOME/my_params.yaml node_params_file:=/tmp/ml.yaml
```

## cv3d tier (geometric pipelines)

The `edgelet` pipeline needs the SDK `cv3d` module, which builds only when the SDK
is configured with `-DUSE_SOPHUS=ON`. Unlike the ML tier this needs no GPU, so it
is not x86-only.

1. **Install Sophus** — `cv3d`'s pose optimizer needs it (Ceres and fmt, its other
   dependencies, are already among the SDK's apt prerequisites):
   ```bash
   sudo apt install -y ros-$ROS_DISTRO-sophus
   ```
2. **Rebuild the SDK with `cv3d`.** Re-configure the existing build tree with
   `-DUSE_SOPHUS=ON` (pointed at the apt Sophus) and rebuild — only `cv3d`
   compiles, the rest is cached:
   ```bash
   cd ~/metavision_src/openeb-5.3.1/build
   cmake -DUSE_SOPHUS=ON -DSophus_DIR=/opt/ros/$ROS_DISTRO/share/sophus/cmake .
   cmake --build . -- -j"$(nproc)" -k
   ```
   `cv3d`'s OGRE-based 3D *sample viewers* need OGRE; the library and the
   algorithms the pipelines use do not, so a missing OGRE only fails those samples
   (harmless with `-k`).
3. **Build the `evk4_sdk_advanced` cv3d tier.** Add `-DSophus_DIR` so the package
   resolves `cv3d`'s transitive Sophus dependency:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select evk4_sdk_advanced --cmake-args \
     -DMetavisionSDK_DIR=<src>/build/generated/share/cmake/MetavisionSDKCMakePackagesFilesDir \
     -DSophus_DIR=/opt/ros/$ROS_DISTRO/share/sophus/cmake
   ```
   `find_package(MetavisionSDK COMPONENTS cv3d)` then succeeds and the `edgelet`
   pipeline builds; without it the cv3d tier is skipped, like the ML tier. On a box
   that also has the ML tier, pass both `-DTorch_DIR=...` and `-DSophus_DIR=...`.

Run it:
```bash
ros2 launch evk4_sdk_advanced pipeline.launch.py pipeline:=edgelet \
    params_file:=$HOME/my_params.yaml
ros2 run rqt_image_view rqt_image_view /event_camera/edgelet_image
```
