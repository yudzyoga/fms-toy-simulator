FROM ros:humble

# Install python deps
RUN apt-get update && apt-get install -y python3-pip python3-tk && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-venv \
    && pip3 install --no-cache-dir fastapi uvicorn pydantic websockets httpx

# Add ROS packages, workspace, and Python deps
COPY . /ws
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && colcon build

# Entry point
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "py_robot", "launcher.py"]




