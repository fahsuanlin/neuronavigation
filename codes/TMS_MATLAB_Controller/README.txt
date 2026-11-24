TMS MATLAB <-> LabVIEW 自動傳送 + 持續接收 + 面板

一、使用前
  - MATLAB 建議 R2020b 以上（configureCallback(tcpclient)）。
  - 檢查 TMS_Start.m 頂端的 ip、port，以及 home_x...home_phi。
  - 尤其 home_phi = 10 是先前測試用的數字，並不是已確認的歸零值！
  - 必須先確認 LabVIEW Main VI 已完成復歸，且實體馬達控制安全。

二、執行
  1. 把這三個 .m 放在同一資料夾，切換 MATLAB Current Folder 至此。
  2. 先執行 LabVIEW Main VI（TCP Listen）。
  3. MATLAB Command Window 輸入：TMS_Start
  4. 視窗面板將開啟；MATLAB 會先傳一次完整 72 Bytes 指令。
  5. MATLAB Command Window 輸入：x_pos = 2
     會在輪詢後自動傳送完整 72 Bytes，其餘軸沿用之前的數值。
     例如：y_pos = 3; psi_pos = 1; 也可修改速度 x_speed = 4。
  6. 回傳的 44 Bytes 每約 100ms 由 LabVIEW 主動送出；MATLAB 自動接收。
     面板顯示最新數值，Command Window 輸入 TMS_Status 可查看快照。
  7. 需要停止 MATLAB 通訊時：stop = 1  或  TMS_Stop
     若要直接在面板停止，可按「停止 TCP」。

三、通訊格式
  TX 72B = [x_speed x_pos 0 y_speed y_pos 0 z_speed z_pos 0
            psi_speed psi_pos 0 tab_mode curve_speed theta_pos 0 phi_pos 0]
  RX 44B = [x_moving x_real y_moving y_real z_moving z_real
            psi_moving psi_real curve_moving theta_real phi_real]
  全部 single，Big Endian；Jog 全部為 0（未實作 Jog）。

四、注意
  - 只在指令內容有改變時傳送，並非每 150ms 重複傳送。
  - MATLAB 回傳接收獨立於傳送，使用 tcpclient configureCallback。
  - MATLAB 的 Timer 不是即時系統，輪詢和 GUI 更新可能延遲。
  - 你目前 Curve Motor VI 依 Tab Mode 只接受 Theta 或 Phi 的其中一組；
    初始化一個72B封包不保證兩組 User Input 都會被 LV 寫入。
  - 停止 MATLAB 連線不能當作馬達急停，必須使用 LV 原本的停機保護。
  - Read/Write 失敗或 LV 斷線，請用 TMS_Stop 關閉後重新連線。
  - 面板的 Real Output 是 LV 回傳數值，不一定是獨立感測器量測的位置。
