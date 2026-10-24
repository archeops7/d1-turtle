FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TURTLEBOT3_MODEL=waffle_pi \
    PYTHONUNBUFFERED=1 \
    HF_HOME=/cache/huggingface \
    QT_X11_NO_MITSHM=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-turtlebot3-gazebo \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install CUDA-enabled PyTorch over any CPU wheels pulled in by transformers.
# --ignore-installed avoids failing on Ubuntu's distutils-managed sympy package,
# and numpy/scipy are pinned for compatibility with ROS 2 Humble system packages.
RUN pip3 install --no-cache-dir --upgrade pip \
    && pip3 install --no-cache-dir "transformers>=5.14" pillow \
    && pip3 install --no-cache-dir --ignore-installed torch torchvision --index-url https://download.pytorch.org/whl/cu124 \
    && pip3 install --no-cache-dir "numpy>=1.23,<1.25" "scipy>=1.11,<1.12"

COPY app /opt/d1_tb3
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
