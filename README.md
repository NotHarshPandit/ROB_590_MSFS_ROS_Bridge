## Running the MSFS–ROS2 Pipeline

### Move to Workspace
```bash
cd C:\ros2_ws
```

---

### Start ROS2 (Windows)
```bash
call C:\dev\ros2-windows\local_setup.bat
call install\local_setup.bat
```

---

### Build the Package
```bash
colcon build --packages-select msfs_sensor_node
```

---

## Run Nodes

#### Start Sensor Node
```bash
ros2 run msfs_sensor_node msfs_sensor_node
```

#### Start Control Node
```bash
ros2 run msfs_sensor_node msfs_control_node
```

---

### Record Commands (Rosbag)

Open **two terminals**:

#### Terminal 1 — Run Sensor Node
```bash
ros2 run msfs_sensor_node msfs_sensor_node
```

#### Terminal 2 — Run Recording Script
```bash
python C:\ros2_ws\src\msfs_sensor_node\src\record_msfs_commands.py
```