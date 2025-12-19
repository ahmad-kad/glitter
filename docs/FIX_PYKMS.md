# Fix: Missing pykms Module

## The Problem

picamera2 installed successfully, but when testing, you get:
```
✗ Import error: No module named 'pykms'
```

This is because `python3-kms++` (which provides `pykms`) is not available in Ubuntu 24.04 repositories.

## The Solution

Build `kms++` from source with `pykms` Python bindings enabled. `pykms` is not a separate repository - it's built as part of `kms++` using the `-Dpykms=true` meson option.

## Quick Fix

Run this script:

```bash
cd ~/glitter
./install_pykms.sh
```

This will:
1. Install build dependencies (meson, ninja-build, libdrm-dev, etc.)
2. **Build kms++ with pykms enabled** (using `-Dpykms=true` option)
3. Test the installation

## Manual Installation

If the script doesn't work, install manually:

```bash
# Install dependencies
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    python3-dev \
    meson \
    ninja-build \
    git \
    libdrm-dev \
    libevdev-dev \
    libboost-dev \
    pkg-config

# Build kms++ with pykms enabled
cd /tmp
git clone https://github.com/tomba/kmsxx.git
cd kmsxx
meson setup build --buildtype=release -Dpykms=enabled
ninja -C build
sudo ninja -C build install
sudo ldconfig

# Test
python3 -c "import pykms; print('OK')"
```

## After Installing pykms

Test picamera2 again:

```bash
python3 -c "from picamera2 import Picamera2; print('OK')"
```

If that works, test the camera:

```bash
./start_camera.sh
```

## Why pykms is Needed

`pykms` (Python bindings for kms++) is used by picamera2 for:
- Display features (preview windows)
- Advanced rendering
- Hardware acceleration

However, **basic camera capture may work without it** in some cases, but picamera2 tries to import it at startup, so it's generally required.

## Troubleshooting

### Build Fails

If `meson setup build` fails:
- Check dependencies: `sudo apt install -y meson ninja-build`
- Check Python dev: `sudo apt install -y python3-dev`
- Try: `meson setup build --prefix=/usr/local`

### Import Still Fails After Install

If `import pykms` still fails:
1. Check installation path: `python3 -c "import sys; print(sys.path)"`
2. May need to add to PYTHONPATH:
   ```bash
   export PYTHONPATH=/usr/local/lib/python3.12/site-packages:$PYTHONPATH
   ```
3. Or reinstall with explicit path:
   ```bash
   cd /tmp/pykms
   meson setup build --prefix=/usr/local
   sudo ninja -C build install
   ```

### Alternative: Use System Packages

If you're on Raspberry Pi OS (not Ubuntu), try:
```bash
sudo apt install -y python3-kms++
```

But this won't work on Ubuntu 24.04, which is why we build from source.










