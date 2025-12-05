#!/bin/bash
# Setup script for ArduPilot Gazebo integration

echo "==== BoilerHawk ArduPilot + Gazebo Harmonic Setup ===="
echo ""

# Check if already installed
if [ -d "$HOME/gz_ws/src/ardupilot_gazebo" ]; then
    echo "✓ ArduPilot Gazebo plugin already exists at ~/gz_ws/src/ardupilot_gazebo"
    echo "  To rebuild: cd ~/gz_ws/src/ardupilot_gazebo/build && make -j4"
    echo ""
else
    echo "Installing ArduPilot Gazebo plugin..."
    echo ""
    
    # Install dependencies
    echo "1. Installing dependencies..."
    sudo apt update
    sudo apt install -y libgz-sim8-dev rapidjson-dev \
        libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
    
    # Clone repository
    echo ""
    echo "2. Cloning ardupilot_gazebo repository..."
    mkdir -p $HOME/gz_ws/src
    cd $HOME/gz_ws/src
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git
    
    # Build plugin
    echo ""
    echo "3. Building ArduPilot Gazebo plugin..."
    export GZ_VERSION=harmonic
    cd ardupilot_gazebo
    mkdir -p build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
    make -j4
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "✓ ArduPilot Gazebo plugin built successfully!"
    else
        echo ""
        echo "✗ Build failed. Check error messages above."
        exit 1
    fi
fi

# Setup environment variables
echo ""
echo "4. Configuring environment variables..."
echo ""

# Check if already in bashrc
if grep -q "GZ_SIM_SYSTEM_PLUGIN_PATH.*ardupilot_gazebo" ~/.bashrc; then
    echo "✓ Environment variables already configured in ~/.bashrc"
else
    echo "Adding to ~/.bashrc..."
    echo "" >> ~/.bashrc
    echo "# ArduPilot Gazebo Harmonic Integration" >> ~/.bashrc
    echo "export GZ_VERSION=harmonic" >> ~/.bashrc
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\$HOME/gz_ws/src/ardupilot_gazebo/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH}" >> ~/.bashrc
    echo "export GZ_SIM_RESOURCE_PATH=\$HOME/gz_ws/src/ardupilot_gazebo/models:\$HOME/gz_ws/src/ardupilot_gazebo/worlds:\${GZ_SIM_RESOURCE_PATH}" >> ~/.bashrc
    echo "export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share:\$HOME/boilerHawk_ws/src/sim_models/models:\${GZ_SIM_RESOURCE_PATH}" >> ~/.bashrc
    echo ""
    echo "✓ Environment variables added to ~/.bashrc"
fi

# Set for current session
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share:$HOME/boilerHawk_ws/src/sim_models/models:${GZ_SIM_RESOURCE_PATH}

echo ""
echo "==== Setup Complete ===="
echo ""
echo "Available drone models from ArduPilot:"
echo "  - iris_runway.sdf     (Quadcopter)"
echo "  - zephyr_runway.sdf   (Fixed-wing)"
echo ""
echo "To test ArduPilot integration:"
echo "  Terminal 1: gz sim iris_runway.sdf"
echo "  Terminal 2: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console"
echo ""
echo "NOTE: You need ArduPilot SITL installed to run sim_vehicle.py"
echo "      See: https://ardupilot.org/dev/docs/building-setup-linux.html"
echo ""
echo "To use with BoilerHawk:"
echo "  source ~/.bashrc (or restart terminal)"
echo "  cd ~/boilerHawk_ws"
echo "  ./launch_drone_sim.sh"
echo ""
