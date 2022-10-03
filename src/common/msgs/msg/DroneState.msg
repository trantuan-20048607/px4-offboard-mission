std_msgs/Header header

## 机载电脑是否连接上飞控: true 已连接
bool connected

## 是否解锁: true 为已解锁
bool armed

## 是否降落: true 为已降落, false 为在空中
bool landed

## PX4飞控当前飞行模式
string mode

## 系统启动时间
float32 time_from_start              ## [s]

## 无人机状态量: 位置, 速度, 姿态
float32[3] position                  ## [m]
float32 rel_alt                      ## [m] only for outdoor
float32[3] velocity                  ## [m/s]
float32[3] attitude                  ## [rad]
geometry_msgs/Quaternion attitude_q  ## 四元数
float32[3] attitude_rate             ## [rad/s]
