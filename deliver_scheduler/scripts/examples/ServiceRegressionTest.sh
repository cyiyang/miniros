# int8 car_no
# # 车辆编号

# int8 request_type
# # request_type字段要求:
# # 0: 取得新的送药需求 GetNewRequest 0 0 drug_type deliver_destination
# # 1: 请求下一个目标 GetNextTarget 0 1 0 0
# # 2: 到达取药区，并取得药物 DrugLoaded 0 2 0 0
# # 3: 到达送药区，并交付药物 Delivered 0 3 0 0

# int8 request_drug_type
# # 0:A, 1:B, 2:C

# int8 request_deliver_destination
# # 1,2,3,4

# ---
# int8 drug_location
# int8 deliver_destination
# # 对于request_type=1 or 2,两个字段的值为-1

rosservice call mission 0 0 0 1
echo =====
rosservice call mission 0 1 0 0
echo =====
rosservice call mission 0 2 0 0
echo =====
echo Sleep 5...
sleep 5
rosservice call mission 0 3 0 0
echo =====
rosservice call mission 0 0 1 2
echo =====
rosservice call mission 0 0 2 4
echo =====
rosservice call mission 0 0 0 1
echo =====
rosservice call mission 0 1 0 0
echo =====
echo Sleep 8...
sleep 8

rosservice call mission 0 3 0 0
echo =====
rosservice call mission 0 0 1 3
echo =====
rosservice call mission 0 1 0 0
echo =====
rosservice call mission 0 2 0 0
echo =====

