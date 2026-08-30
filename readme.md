# Engine Simulator (3D) JSON Engine Generator

> [!NOTE]
> Usage instructions/specification coming soon. Please see include/scene_builder.h for definitions of available methods for JSON editing.

## Installing the Python bindings

1. Go to the [latest release](https://github.com/MidnightMagenta/ES-Part-Generator/releases/latest)
2. Download the `.whl` file matching your OS and Python version (note your OS and Python version needs to match EXACTLY). Python version is specified with `cpXXX` with for example, `cp312` meaning `Python 3.12`:
   - Windows: `esjson-...-win_amd64.whl`
   - macOS (Apple Silicon): `esjson-...-macosx_13_0_arm64.whl`
   - Linux: `esjson-...-manylinux_2_28_x86_64.whl`
3. Install it with `pip install path/to/esjson-0.1.0-<your-file>.whl`
4. You can verify it worked with `python -c "import esjson; print(esjson.SceneBuilder)"`
