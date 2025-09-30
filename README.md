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
<img width="377" height="220" alt="硬體配置" src="https://github.com/user-attachments/assets/7d3356aa-13d1-498a-bd77-3e29d3132962" /><img width="433" height="220" alt="硬體架構圖" src="https://github.com/user-attachments/assets/e854b2ae-f3f6-4b02-b7c5-29adba7f8c43" />
# 系統架構
機器人座標轉換使用卡爾曼濾波器結合視覺里程、IMU與傳統輪式里程計的數據當作機器人導航里程計使用，定位方式使用AMCL作為機器人導航定位，最後全局路徑規劃器與區域路徑規劃器使用Voronoi_Planner與TEB Local_Planner。機器人所搭載的三顆鏡頭分別安裝於機體左側、右側與後方。
<img width="1468" height="693" alt="總架構" src="https://github.com/user-attachments/assets/d1df0a53-16ac-455a-8dd3-e54979b181a9" />

