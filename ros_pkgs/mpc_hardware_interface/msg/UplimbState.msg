Header header                      # 当前时间戳
int8 cmd_num                       # 当前运行状态码
string cmd_name                     # 当前运行状态名称
bool left_arm_is_singular          # 当前左臂是否奇异
bool right_arm_is_singular         # 当前右臂是否奇异
bool iddp_status                   # iddp链接状态







byte STOPPED=0
byte MOVEJ=1
byte MOVEJ_PATH=2
byte MOVEL_NULLSPACE=3
byte MOVEL=4
byte MOVEL_PATH=5
byte SPEEDJ=6
byte SPEEDL=7
byte SPEED_STOP=8
byte MOVECSVFILE=9
byte MOVEFOURIER=10
byte MOVEJ_SPLINE=11
byte TEACH=12
byte SERVOJ=13
byte SERVOL=14


# -----------------------------------------------------------------
# 0为停止状态，
# 1为MOVEJ，
# 2为MOVEJ_PATH，
# 3为MOVEL_NULLSPACE，
# 4为MOVEL， 
# 5为MOVEL_PATH，
# 6为SPEEDJ，
# 7为SPEEDL，
# 8为SPEED_STOP，
# 9为MOVECSVFILE，
# 10为MOVEFOURIER，
# 11为MOVEJ_SPLINE，
# 12为TEACH，
# 13为SERVOJ，
# 14为SERVOL。
# -----------------------------------------------------------------
