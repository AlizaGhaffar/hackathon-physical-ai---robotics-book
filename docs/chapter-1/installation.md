---
sidebar_position: 7
---

# Installing ROS 2 Humble

## Prerequisites

Before installing ROS 2, ensure your system meets these requirements:

### System Requirements
- **OS**: Ubuntu 22.04 (Jammy Jellyfish) recommended
- **RAM**: Minimum 4GB, 8GB+ recommended
- **Storage**: At least 10GB free space
- **Internet**: Stable connection for downloads

### Alternative Platforms
- **Windows**: Use WSL2 (Windows Subsystem for Linux)
- **macOS**: Use Docker or virtual machine
- **Other Linux**: Check ROS 2 documentation for compatibility

---

## Step 1: Set Up Locale

Ensure your system uses UTF-8 encoding:

```bash
locale  # check current settings

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

---

## Step 2: Add ROS 2 Repository

Add the ROS 2 apt repository to your system:

```bash
# Install required tools
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

## Step 3: Install ROS 2 Humble

Update package index and install ROS 2:

```bash
# Update apt repository cache
sudo apt update
sudo apt upgrade

# Install ROS 2 Humble Desktop (recommended for learning)
sudo apt install ros-humble-desktop

# OR install ROS 2 Base (minimal, no GUI tools)
# sudo apt install ros-humble-ros-base
```

**Installation Size**: ~2GB download, ~4GB installed

---

## Step 4: Environment Setup

### Source ROS 2 Setup Script

Every time you open a new terminal, you need to source the ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
```

### Automatic Sourcing (Recommended)

Add the source command to your shell profile so it runs automatically:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

For **zsh** users:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.zshrc
source ~/.zshrc
```

---

## Step 5: Install Development Tools

Install additional tools for building and managing ROS 2 packages:

```bash
# Install colcon (build tool)
sudo apt install python3-colcon-common-extensions

# Install rosdep (dependency management)
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

---

## Step 6: Verify Installation

Test that ROS 2 is installed correctly:

### Check ROS 2 Version
```bash
ros2 --version
```

**Expected output**: `ros2 cli version 0.xx.x`

### Run Demo Talker-Listener

**Terminal 1** (Talker):
```bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2** (Listener):
```bash
ros2 run demo_nodes_cpp listener
```

**Expected behavior**: Terminal 1 publishes messages, Terminal 2 receives them.

---

## Common Issues & Solutions

### Issue 1: "ros2: command not found"
**Solution**: Source the setup script:
```bash
source /opt/ros/humble/setup.bash
```

### Issue 2: GPG key error during installation
**Solution**: Re-add the GPG key:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Issue 3: Package not found errors
**Solution**: Update package cache:
```bash
sudo apt update
```

---

## Next Steps

Now that ROS 2 is installed:
1. ✅ Create your first ROS 2 workspace
2. ✅ Build your first node
3. ✅ Explore ROS 2 command-line tools

Continue to the next section to start building! 🚀

---

## Quick Reference

**Source ROS 2**:
```bash
source /opt/ros/humble/setup.bash
```

**Check installed packages**:
```bash
ros2 pkg list
```

**List available nodes**:
```bash
ros2 node list
```

**Get help**:
```bash
ros2 --help
```
