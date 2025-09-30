# Mecanum Wheel-base Autonomous Mobile Robot Based on ROS1(noetic)
本研究旨在建構一套具備全向移動能力與高導航精度的自主式移動機器人（Autonomous Mobile Robot, AMR）系統，以因應智慧製造環境中物流搬運自動化的需求。系統採用麥克納姆輪（Mecanum Wheel）作為移動平台，使 AMR 能在空間受限或障礙物密集的場域中靈活移動。全域路徑規劃以 Voronoi 演算法進行，提升與障礙物間的安全距離。為解決移動過程中可能發生之輪胎打滑與定位誤差問題，本系統融合深度相機視覺里程計、慣性測量單元（IMU）與輪速里程計資料，透過卡爾曼濾波器進行多感測器資料融合，提升定位準確性與穩定性。最終停靠階段則透過深度相機偵測 AR Track Marker，結合 PID 控制完成靠站位置與姿態的精準修正。整體系統整合感測、導航與控制模組，實現具穩定性、高效率與應用價值的 AMR 解決方案。
###### AR Track Marker
https://wiki.ros.org/ar_track_alvar
###### TEB Local Planner
https://wiki.ros.org/teb_local_planner
