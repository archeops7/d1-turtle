#!/bin/bash
# ROS 2 setup.bash references unset variables, so nounset (-u) is omitted.
set -eo pipefail
source /opt/ros/humble/setup.bash

# On WSL2 with Windows NVIDIA Driver 555+, CUDA requires libnvdxgdmal.so.1 from the
# active driver store while keeping /usr/lib/wsl/lib/libcuda.so.1 as the loader.
driver_dir=""
driver_mtime=0
if [ -d /usr/lib/wsl/drivers ]; then
  while IFS= read -r lib; do
    dir=$(dirname "$lib")
    [ -e "$dir/libnvdxgdmal.so.1" ] || continue
    mtime=$(stat -c %Y "$lib" 2>/dev/null || echo 0)
    if [ "$mtime" -ge "$driver_mtime" ]; then
      driver_dir=$dir
      driver_mtime=$mtime
    fi
  done < <(find /usr/lib/wsl/drivers -name 'libcuda.so.1.1' 2>/dev/null || true)
fi

if [ -n "$driver_dir" ]; then
  export LD_LIBRARY_PATH="${driver_dir}:/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  for candidate in /usr/lib/wsl/lib/libcuda.so.1 "$driver_dir/libcuda_loader.so"; do
    [ -e "$candidate" ] || continue
    if python3 -c "import ctypes, sys; sys.exit(0 if ctypes.CDLL(sys.argv[1], mode=ctypes.RTLD_GLOBAL).cuInit(0) == 0 else 1)" "$candidate" 2>/dev/null; then
      export D1_LIBCUDA=$candidate
      for dest in /lib/x86_64-linux-gnu/libcuda.so.1 /usr/lib/x86_64-linux-gnu/libcuda.so.1; do
        [ -f "$dest" ] && mount --bind "$(readlink -f "$candidate")" "$dest" 2>/dev/null || true
      done
      break
    fi
  done
fi

export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}"

# shellcheck disable=SC1091
source /opt/d1_tb3/select_world.sh
exec ros2 launch /opt/d1_tb3/bringup.launch.py \
  "world:=${WORLD}" \
  "spawn_x:=${SPAWN_X}" \
  "spawn_y:=${SPAWN_Y}" \
  "spawn_yaw:=${SPAWN_YAW}"
