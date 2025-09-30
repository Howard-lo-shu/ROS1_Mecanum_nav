# Mecanum Wheel-base Autonomous Mobile Robot Based on ROS1(noetic)
  本研究旨在建構一套具備全向移動能力與高導航精度的自主式移動機器人（Autonomous Mobile Robot, AMR）系統，以因應智慧製造環境中物流搬運自動化的需求。系統採用麥克納姆輪（Mecanum Wheel）作為移動平台，使 AMR 能在空間受限或障礙物密集的場域中靈活移動。全域路徑規劃以 Voronoi 演算法進行，提升與障礙物間的安全距離。為解決移動過程中可能發生之輪胎打滑與定位誤差問題，本系統融合深度相機視覺里程計、慣性測量單元（IMU）與輪速里程計資料，透過卡爾曼濾波器進行多感測器資料融合，提升定位準確性與穩定性。最終停靠階段則透過深度相機偵測 AR Track Marker，結合 PID 控制完成靠站位置與姿態的精準修正。整體系統整合感測、導航與控制模組，實現具穩定性、高效率與應用價值的 AMR 解決方案。
###### AR Track Marker
https://wiki.ros.org/ar_track_alvar
###### TEB Local Planner
https://wiki.ros.org/teb_local_planner
###### 進佔導引參考資料
https://github.com/ROBOTIS-GIT/turtlebot3_applications
###### robot_localization
https://docs.ros.org/en/melodic/api/robot_localization/html/index.html
###### IMU Filter
https://wiki.ros.org/imu_filter_madgwick

# 硬體架構
  本文所使用的硬體底層設備為友達光電所開發的AMR自主移動機器人，型號為AMR150m。AMR前後分別裝有一顆Lidar，用於感測周圍環境資訊以及移動時及時避開障礙物。AMR四周裝有深度相機，達成多方向視覺導引功能。

<img width="340" height="220" alt="硬體配置" src="https://github.com/user-attachments/assets/7d3356aa-13d1-498a-bd77-3e29d3132962" /><img width="433" height="220" alt="硬體架構圖" src="https://github.com/user-attachments/assets/e854b2ae-f3f6-4b02-b7c5-29adba7f8c43" />
# 系統架構
  機器人座標轉換使用卡爾曼濾波器結合視覺里程、IMU與傳統輪式里程計的數據當作機器人導航里程計使用，定位方式使用AMCL作為機器人導航定位，最後全局路徑規劃器與區域路徑規劃器使用Voronoi_Planner與TEB Local_Planner。機器人所搭載的三顆鏡頭分別安裝於機體左側、右側與後方。
<img width="1468" height="693" alt="總架構" src="https://github.com/user-attachments/assets/d1df0a53-16ac-455a-8dd3-e54979b181a9" />

  慣性測量單元（Inertial Measurement Unit, IMU）資訊是由Orbbec所開法的Gemini2深度相機中的，線性加速度/camera/accel/sample與角加速度/camera/gyro/sample進行處裡後所發不出來。使用imu_filter_madgwick，從原始 IMU 資料中估算裝置的空間方向（姿態）。該濾波器透過加速度計與陀螺儀的資料進行姿態推算，並可選擇性地加入磁力計資料以提升航向（Yaw）角的準確性。該演算法以四元數（quaternion）格式輸出姿態，具有計算效率高、延遲低、適合即時應用等優點。通過imu_filter_madgwick將會把估測出的姿態給EKF做數據融合處理。
<img width="973" height="644" alt="imu" src="https://github.com/user-attachments/assets/2bb08c10-a7a3-4e79-880e-cea5a598dd3e" />

  視覺里程計（Visual Odometry）Orbbec所開法的Gemini2深度相機透過 RGB-D 相機所擷取之彩色影像與對應深度資訊，可直接從影像中取得空間座標，進而增強特徵匹配的精度與穩定性，相較於僅依賴灰階影像之傳統單目 VO，能提供更準確且穩定的位姿估計結果。視覺里程計模組為 RTAB-Map 套件中提供之 rtabmap_odom，該模組利用 RGB-D 影像進行特徵點提取與匹配，並即時估算相機的相對六自由度（6-DOF）運動。rtabmap_odom 首先從 RGB 影像中提取具辨識度的特徵點，接著利用深度影像提供的深度資訊將這些二維特徵點轉換至三維空間座標，建立三維特徵點雲。
https://wiki.ros.org/rtabmap_odom


# 執行結果
https://github.com/user-attachments/assets/030374f3-ce4b-4fe6-a2bd-173ca7b2b5e5

# Launch command
###### 詳細的指令請參閱ROS launch 指令集，此處僅列出基本使用指令
### Mechatronics system (sencor + odom +tf )
本系統的底盤資料為友達光電所開法MCU Drive，所以沒有提供 robot_mcu 的 Pkg 資料。
```
# Only using Chassis
roslaunch robot_mcu robot_odom.launch

# Only using Lidars
roslaunch robot_mcu scan.launch

# Using Chassis and Sencors
roslaunch robot_mcu robot_odom_laser.launch 
```
### Sensor fusion
```
# After starting the robot chassis, open a new terminal
source /opt/ros/noetic/setup.bash
source ~/<Your workspace>/devel/setup.bash –extend

# Start sensor fusion command
roslaunch robot_nav imu_filter.launch
```
### Mapping
```
# After starting the robot chassis, open a new terminal
source /opt/ros/noetic/setup.bash
source ~/<Your workspace>/devel/setup.bash –extend

# Gmapping
roslaunch cimc_agv_slam cimc_agv_gmapping.launch

# Save map
rosrun map_server map_saver -f <Your map name>
```
### Navigation
```
# After starting the robot chassis, open a new terminal
source /opt/ros/noetic/setup.bash
source ~/<Your workspace>/devel/setup.bash –extend

# Navigation using Sensor fusion odom_ekf
roslaunch robot_nav navigation_teb_voronoi.launch

# Navigation using robor Chassis odom
roslaunch robot_nav navigation_teb.launch
```
