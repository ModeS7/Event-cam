# Installing the Metavision SDK

You need a Prophesee account and identity token first — see
[access.md](access.md). Then pick your platform:

- **x86_64** → apt binaries (fast). [Jump to x86.](#x86_64-apt)
- **ARM (Raspberry Pi 5, Jetson)** → build from source (no ARM binaries exist).
  [Jump to ARM.](#arm-build-from-source)

Either way, `evk4_sdk_advanced` finds the SDK by CMake and the rest is identical
([README.md](README.md)).

Set your distro once:

```bash
export ROS_DISTRO=jazzy
```

## x86_64 (apt)

> **Status:** documented from Prophesee's instructions; the *ARM source build*
> is validated, not this apt path. The steps below are the standard
> Prophesee apt flow.

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
- `-DUSE_SOPHUS=OFF` — skip CV3D (needs a manual Sophus build).
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

### Note for x86 ML (later)

To use the ML detection module, rebuild with `-DUSE_TORCH=ON` and a LibTorch
(CUDA on x86/Jetson). That is the experimental tier — not needed for, and not
covered by, the optical-flow pipeline.
