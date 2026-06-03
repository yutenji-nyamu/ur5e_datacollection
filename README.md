Real World UR5e Data Collection   

This records the entire process of building the system from the ground up, including all environments and fine-grained subfunctions.

PC: right UR, zw account

The system has been reinstalled to Ubuntu 25.04, and all packages need to be set up again.

Conda environment based on RoboTwin 2.0 environment

### (1) UR5e Connection and Basic Control 

#### (1.1) Simplest

The simplest UR5e control script, only the robotic arm, without the gripper

```bash
pip install keyboard
pip install pyserial
```

Check the network connection with the robotic arm: 

```bash
ping 192.168.0.3
```

Switch Polyscope to remote control

```bash
python ur5e_test_mini_clean.py
```

#### (1.2) Freedrive

Entering free drive mode for a period of time:

```bash
python freedrive_socket.py
```

Note: This must be done under Remote Control; the robot cannot execute trajectory commands (such as movej) in Freedrive mode.

Future: More convenient hardware implementation:

https://www.universal-robots.com/articles/ur/interface-communication/external-freedrive-button/

#### (1.3) Gripper

/dev/ttyUSB0 belongs to root:dialout, so need to add current user to the dialout group to avoid using sudo every time:

```bash
sudo usermod -aG dialout zhangw
```

Make sure to turn on the gripper switch; Polyscope remote control. 

Test: 

```bash
python test_gripper_min.py
```

### （2）Camera-related component installation

```bash
sudo apt update
sudo apt install librealsense2-utils librealsense2-dkms librealsense2-dev librealsense2-dbg librealsense2
sudo apt install librealsense2-gl librealsense2-net librealsense2-udev
sudo apt install realsense-viewer
sudo apt install -y apt-transport-https ca-certificates curl
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
  | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo noble main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install -y \
  librealsense2-dkms \
  librealsense2-utils \
  librealsense2-dev \
  librealsense2-gl \
  librealsense2-udev-rules
```

Ubuntu 25.04 is a bit troublesome, but running a few more commands will do.

Use Intel's app to view the camera: 

```bash
realsense-viewer
```

### (3) Camera feed acquisition using python

```bash
pip install pyrealsense2
```
Display color image in real time

Press s to take a screenshot

Press q to quit

```bash
python realsense_test.py
```

A demo for any dual camera setup:

```bash
python realsense_dual_test.py
```

A demo for defining head and wrist cameras:

```bash
python realsense_dual_head_wrist.py
```

### (4) RTDE 

### (4.1) install


RTDE Project: 
https://github.com/UniversalRobots/RTDE_Python_Client_Library

Download zip: 
https://github.com/UniversalRobots/RTDE_Python_Client_Library/releases

```bash
pip install wheel
pip install rtde-2.7.12-release.zip
```

```bash
pip install numpy
pip install matplotlib
```
### (4.2) Read

RTDE basic test: output TCP pose per second:

```bash
python rtde_init_test.py
```

### (4.3) Write

Switch Polyscope to local mode, first run URP, then run the Python script:

```bash
python rtde_control_min.py
```

### (4.4) Servoj

Provides smoother control: 

1 PolyScope: Load and run translation_sample_servoj.urp, it will pop up a popup (blocking=True) and pause at Continue.

2 PC: Run python

```bash
python servoj_rtde_min_urp.py
```

3 PolyScope: Click Continue

project:

https://github.com/danielstankw/Servoj_RTDE_UR5

### (5) Collect action and camera data separately

### (5.1) Collect EE Pose of Arm

Collect action data using RTDE.

Record the TCP pose of the robotic arm and save it:

```bash
python rtde_collect_2_csv.py
```

### (5.2) Collect Camera Data

Basic implementation that saves images to a folder at a certain frequency:

```bash
python realsense_collect_2_folder.py
```

Dual camera ver.:

```bash
python realsense_dual_collect_2_folder.py
```

### (5.3) Encapsulate

### (5.3.1) Camera

Encapsulated Camera Data Collection into Functions: 

realsense_collect_2_folder_func.py

realsense_dual_collect_2_folder_func.py

### (5.3.2) Gripper, RTDE, and Freedrive

Encapsulated All Script About Action Data Collection into Functions: 

rtde_collect_2_csv_func.py (Maybe historical)

gripper_serial.py

rtde_tcp_logger.py

freedrive_urscript.py

### (5.4) Collect Action Data: EE-Pose and Gripper

Notice Swtich Remote Control.

```bash
python collect_arm_gripper.py
```

c + Enter : gripper close

o + Enter : gripper open

q + Enter : quit

### (6) Simultaneous collect action and camera data 

### (6.1) Encapsulated

A class that encapsulates data collection for the arm and gripper:

arm_gripper_collector.py

### (6.2) Collect

Initial pose for data collection and reasoning, go home script (Polyscope remote mode):

```bash
go_home.py
```

Polyscope remote mode.

Note, the B81L laboratory's network requires all five circuit breakers to be turned on.

Arm + Gripper + dual Camera: 

```bash
python collect_data_action_arm_gripper_dual_camera_no_cv.py
```

c: gripper close

o: gripper open

q: quit

History:

```bash
python collect_data_action_camera.py
python collect_data_action_dual_camera.py
```

10hz

### (6.3) Raw Data

The collected raw data is temporarily stored in action_data/ and camera_data/

(LOG:12261816 collect 10 data in RoboTwin_like_data/run_20251226_181456, not delete)

(action_data/ and camera_data/ is raw data, can delete after data is be converted)

### (7) Convert to HDF5 format

Modify the task name in the Python script.

Convert all existing data in folder to HDF5:

```bash
python convert_2_hdf5_output_log_dual_gripper.py
```

History:

```bash
python convert_2_hdf5_output_log.py
python convert_2_hdf5_output_log_dual.py
```

(LOG:12261816 Convert 10 data to hdf5)
(LOG:12311853 Convert 15 data to hdf5, RoboTwin_like_data/run_20251231_185159)

View the contents of an HDF5 file, Example: 

```bash
python preview_hdf5.py /home/zhangw/UR5e_DataCollection/RoboTwin_like_data/run_20251130_194629/torch_cube/simple/data/episode0.hdf5
```

### (8) Convert to ACT format

Include RoboTwin 2.0 as a subfolder of the project, without git: 

UR5e_DataCollection/RoboTwin

In RoboTwin 2.0, each policy has a script that processes data in HDF5 format into the format required for training that policy. Such as: 

UR5e_DataCollection/RoboTwin/policy/ACT/process_data.py

Modify the data processing script for each strategy, using ACT as an example

Enter the task name, path, and data quantity: 

```bash
cd /home/zhangw/UR5e_DataCollection/RoboTwin/policy/ACT
python process_data_real.py \
  /home/zhangw/UR5e_DataCollection/RoboTwin_like_data/run_20251231_190640 \
  pick_block_bowl \
  simple \
  15
```

The converted data (ACT), for example:

```bash
RoboTwin/policy/ACT/processed_data/sim-pick_block_bowl/simple-15
```

(LOG:12261816 Convert 10 data to ACT)
(LOG:12311910 Convert 15 data to ACT, RoboTwin/policy/ACT/processed_data/sim-pick_block_bowl/simple-15)

### (9) Train

Install the ACT-related environment according to the RoboTwin documentation:

```bash
cd /home/zhangw/UR5e_DataCollection/RoboTwin/policy/ACT
pip install pyquaternion pyyaml rospkg pexpect mujoco==2.3.7 dm_control==1.0.14 \
           opencv-python matplotlib einops packaging h5py ipython
cd detr
pip install -e .
cd ..
```

Start Training:

```bash
cd /home/zhangw/UR5e_DataCollection/RoboTwin/policy/ACT
bash train.sh pick_block_bowl simple 15 0 0
#            ^task_name  ^task_config  ^expert_data_num  ^seed  ^gpu_id
```

### (10) Inference

#### (10.1) Encapsulated

rtde_servoj_controller.py

#### (10.2) Run

Local Control.

Return Arm to its initial position, and Warm up:

```bash
python /home/zhangw/UR5e_DataCollection/go_home_servoj.py
```

Run:

```bash
cd /home/zhangw/UR5e_DataCollection/RoboTwin/policy/ACT
python real_eval_rtde_servoj_dualcam_gripper.py
```

#### (10.3) History

Some preliminary phased test scripts: 

```bash
python RoboTwin/policy/ACT/real_eval_stage1_load_act.py
python RoboTwin/policy/ACT/real_eval_stage2_hdf5_forward.py
python RoboTwin/policy/ACT/real_eval_stage3_online_no_ctrl.py
```

(First, adjust the task, settings, and data nums in: real_eval_stage1_load_act.py)

Performing inference on a real UR5e (Remote Control): 

```bash
python RoboTwin/policy/ACT/real_eval.py
```

### TODO

控制：平滑，关节；rt的控制模式

数据：收集关节，数量，随机化，vr

训练：lora, 240, 核桃

更多模型: RoboTwin 2.0

Lerobot框架: https://huggingface.co/docs/lerobot/en/index
适配，或者参考功能，或者复用脚本
https://huggingface.co/docs/lerobot/en/hilserl

仓库管理：数据管理；文件夹

接口统一封装，以适配更多框架


