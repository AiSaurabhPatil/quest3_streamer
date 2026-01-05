# cuRobo Installation Guide for Isaac Sim (Remote Server)

This guide covers installing NVIDIA cuRobo in Isaac Sim's Python environment on a remote server.

## Prerequisites

- **Isaac Sim 4.0 or later** installed on the server
- **NVIDIA GPU** with VOLTA architecture or newer (RTX 20xx, 30xx, 40xx, A100, etc.)
- **CUDA 11.8+** (bundled with Isaac Sim)
- **git** and **git-lfs** installed

## Step 1: Find Your Isaac Sim Installation Path

```bash
# Common Isaac Sim installation locations:
# Default: ~/.local/share/ov/pkg/isaac_sim-4.0.0
# Or: /opt/nvidia/isaac-sim

# Verify by listing:
ls ~/.local/share/ov/pkg/ | grep isaac

# Set the path (adjust as needed):
export ISAAC_SIM_PATH=~/.local/share/ov/pkg/isaac_sim-4.0.0
```

## Step 2: Create an Alias for Isaac Sim Python

```bash
# Add to your ~/.bashrc:
echo 'alias omni_python="${ISAAC_SIM_PATH}/python.sh"' >> ~/.bashrc
source ~/.bashrc

# Verify it works:
omni_python --version
```

## Step 3: Install git-lfs (if not installed)

```bash
# Ubuntu/Debian:
sudo apt-get install git-lfs

# Initialize git-lfs:
git lfs install
```

## Step 4: Install Required Python Packages

```bash
# Install prerequisites in Isaac Sim's Python environment:
omni_python -m pip install tomli wheel ninja
```

## Step 5: Clone and Install cuRobo

```bash
# Navigate to your preferred directory:
cd ~/Development  # or wherever you prefer

# Clone cuRobo repository:
git clone https://github.com/NVlabs/curobo.git
cd curobo

# Install cuRobo (this may take 10-20 minutes for compilation):
omni_python -m pip install -e .
```

> **Note**: The `-e` flag installs in "editable" mode - changes to the source are reflected immediately without reinstallation.

## Step 6: Verify Installation

```bash
# Test import:
omni_python -c "from curobo.wrap.reacher.ik_solver import IKSolver; print('✓ cuRobo IK Solver imported successfully')"

# Test with GPU:
omni_python -c "import torch; print(f'✓ CUDA available: {torch.cuda.is_available()}')"
```

## Step 7: Get Modified Teleop Script

After cuRobo is installed, I will update `isaac_teleop_usd.py` on your local machine. You'll need to copy it to the server:

```bash
# From your local machine:
scp /home/saurabh/Development/meta_openxr/isaac_teleop_usd.py user@remote-server:~/path/to/meta_openxr/
```

## Troubleshooting

### "No module named 'curobo'"
Ensure you're using Isaac Sim's Python:
```bash
which python  # Should NOT be your system Python
omni_python -m pip list | grep curobo
```

### Compilation Errors During Install
Install build dependencies:
```bash
sudo apt-get install build-essential cmake
```

### CUDA Version Mismatch
cuRobo needs CUDA 11.8+. Isaac Sim 4.0 includes this, but verify:
```bash
omni_python -c "import torch; print(torch.version.cuda)"
```

## Quick Reference Commands

| Task | Command |
|------|---------|
| Isaac Python REPL | `omni_python` |
| Install package | `omni_python -m pip install <package>` |
| List packages | `omni_python -m pip list` |
| Run script | `omni_python script.py` |
