# Serial-Robot-Kinematics-Solver
![image](https://github.com/Geng-Hao/Serial-Robot-Kinematics-Solver/blob/main/Algorithm%20Overview.png)

本專案使用循環座標下降演算法(參考文獻：A combined optimization method for solving the inverse kinematics problem of mechanical manipulators)實作6軸機械手臂之運動模型解算器，並以C++物件導向程式設計方式開發。使用者可為自身之機械手臂建立一個Robot Class，並使用內建之public member function計算機械手臂之順向運動學和逆向運動學。目前該C++專案已實作於Meca500機器人，並於Linux環境撰寫socket程式(使用C語言)發送運動指令使機器人繪製圖形。
