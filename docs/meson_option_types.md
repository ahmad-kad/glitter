# Meson Option Types - Understanding Build Configuration

## The Error You Saw

```
ERROR: Value "true" (of type "string") for combo option "Enable DRM preview window support" 
is not one of the choices. Possible choices are (as string): "enabled", "disabled", "auto".
```

## What This Means

Meson (the build system) has different types of options:

### 1. Boolean Options (Old Style)
```python
# In meson.build (old style)
option('enable_drm', type: 'boolean', value: true)
```

Used in command line as:
```bash
meson setup build -Denable_drm=true   # or =false
```

### 2. Feature Options (New Style - Preferred)
```python
# In meson.build (new style)
option('enable_drm', type: 'feature', value: 'auto')
```

Used in command line as:
```bash
meson setup build -Denable_drm=enabled   # or =disabled or =auto
```

### 3. Combo Options (Multiple Choices)
```python
# In meson.build
option('backend', type: 'combo', choices: ['opengl', 'vulkan', 'software'])
```

Used in command line as:
```bash
meson setup build -Dbackend=opengl
```

## Why Feature Options Are Better

| Boolean | Feature |
|---------|---------|
| `true` / `false` | `enabled` / `disabled` / `auto` |
| Explicit choice only | Can auto-detect if available |
| Either have it or don't | Can gracefully degrade |

**Example**: With `auto`, meson will:
1. Check if dependencies are available
2. Enable feature if found
3. Disable if not found (no error)
4. Continue build successfully

This makes builds more portable!

## How to Find Correct Options

### Method 1: Read the Error Message
```
Possible choices are (as string): "enabled", "disabled", "auto"
```
The error tells you exactly what values are allowed!

### Method 2: Check meson_options.txt
```bash
cd ~/builds/rpicam-apps
cat meson_options.txt
```

Example output:
```
option('enable_drm', type: 'feature', value: 'auto',
       description: 'Enable DRM preview window support')
option('enable_egl', type: 'feature', value: 'disabled',
       description: 'Enable EGL preview window support')
```

### Method 3: Use meson configure
```bash
cd ~/builds/rpicam-apps/build
meson configure

# Or to see specific option:
meson configure | grep enable_drm
```

### Method 4: Check Documentation
```bash
# Usually in README.md or INSTALL.md
cat ~/builds/rpicam-apps/README.md
```

## Common Meson Options

### Standard Options (All Projects)
```bash
--buildtype=release        # Optimization level: debug, debugoptimized, release
--prefix=/usr/local        # Installation prefix
--libdir=lib               # Library directory
-Dtests=true              # Enable tests (common pattern)
```

### Project-Specific Options (rpicam-apps)
```bash
-Denable_drm=enabled      # DRM display output
-Denable_egl=enabled      # EGL display output  
-Denable_qt=enabled       # Qt GUI applications
-Denable_opencv=disabled  # OpenCV integration
-Denable_tflite=disabled  # TensorFlow Lite support
-Denable_libav=enabled    # FFmpeg/libav for video encoding
```

## Debugging Build Configuration

### Check Current Configuration
```bash
cd ~/builds/rpicam-apps/build
meson configure

# Shows all options and their current values
```

### Change Option After Configuration
```bash
# Don't need to delete build directory!
meson configure build -Denable_drm=enabled

# Then rebuild:
ninja -C build
```

### Full Reconfigure (Clean Start)
```bash
rm -rf build
meson setup build -Denable_drm=enabled ...
```

## Why Projects Change Option Types

### Historical Evolution

**Phase 1**: Early meson (< 0.47)
- Only had boolean options
- Projects used: `type: 'boolean'`

**Phase 2**: Meson 0.47+ (2018)
- Introduced feature options
- Added `auto` detection capability

**Phase 3**: Modern meson (1.0+)
- Feature options recommended
- Projects migrate old booleans to features

**rpicam-apps updated their meson.build** to use modern feature options, which is why our old syntax failed!

## Practical Example - Before vs After

### Before (Old, Broken)
```bash
meson setup build \
    -Denable_drm=true \      # ✗ Wrong type
    -Denable_egl=false       # ✗ Wrong type
```

### After (New, Fixed)
```bash
meson setup build \
    -Denable_drm=enabled \   # ✓ Correct
    -Denable_egl=disabled    # ✓ Correct
```

### Using Auto-Detection
```bash
meson setup build \
    -Denable_drm=auto \      # ✓ Enable if libdrm found
    -Denable_egl=auto        # ✓ Enable if EGL found
```

This is **more robust** - build won't fail if optional dependencies missing!

## Key Takeaways

1. **Read error messages carefully** - they often contain the solution
2. **Check meson_options.txt** - authoritative source of options
3. **Feature options are modern** - prefer `enabled/disabled/auto` over `true/false`
4. **Use `auto` for optional features** - makes builds more portable
5. **Projects evolve** - syntax that worked last year may need updates

## Comparison with Other Build Systems

| Build System | Configuration |
|--------------|---------------|
| **Autotools** | `./configure --enable-drm --disable-opencv` |
| **CMake** | `cmake -DENABLE_DRM=ON -DENABLE_OPENCV=OFF` |
| **Meson** | `meson setup build -Denable_drm=enabled -Denable_opencv=disabled` |

Meson's approach:
- ✓ Consistent syntax
- ✓ Type-safe (catches errors early)
- ✓ Auto-detection with `auto`
- ✓ Fast (ninja backend)

## Additional Resources

- [Meson Documentation](https://mesonbuild.com/Build-options.html)
- [Feature Options Guide](https://mesonbuild.com/Build-options.html#features)
- [rpicam-apps Build Options](https://github.com/raspberrypi/rpicam-apps#build-options)



