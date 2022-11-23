# DistanceTimeCalculator_raw

msg file description

```
float64 distance_remaining            : 목적지까지의 남은 거리 ( Unit : meter )
float64 arrival_time                  : 목적지까지의 잔여 예상 시간 ( Unit : Seconds )
float64 distance_robot_traveled       : 로봇이 움직인 거리 ( Unit : Meter )
float64 remaining_distance_percentage : 목적지까지의 로봇 진행률 ( Unit : Percentage )
uint8 status_info                     : 로봇 명령 상태상황
    - PENDING         = 0   # The goal has yet to be processed by the action server - ( 목적지 이동 불가 )
    - ACTIVE          = 1   # The goal is currently being processed by the action server - ( 목적지로의 명령 수신 완료, 목적지로 진행중 )
    - PREEMPTED       = 2   # The goal received a cancel request after it started executing - ( 목적지로의 일시중지 완료됨 )
                            #   and has since completed its execution (Terminal State)
    - SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State) - ( 목적지 도착 완료 )
    - ABORTED         = 4   # The goal was aborted during execution by the action server due - (목적지 이동 불가)
                            #    to some failure (Terminal State)
    - REJECTED        = 5   # The goal was rejected by the action server without being processed, - ( 목적지 이동 불가)
                            #    because the goal was unattainable or invalid (Terminal State)
    - PREEMPTING      = 6   # The goal received a cancel request after it started executing - ( 목적지 이동 취소됨 )
                            #    and has not yet completed execution
    - RECALLING       = 7   # The goal received a cancel request before it started executing, - ( 목적지 이동 취소됨 )
                            #    but the action server has not yet confirmed that the goal is canceled
    - RECALLED        = 8   # The goal received a cancel request before it started executing - ( 목적지 이동 취소 완료됨 )
                            #    and was successfully cancelled (Terminal State)
    - LOST            = 9   # An action client can determine that a goal is LOST. This should not be  - ( 목적지 명령 통신 이상 발생, 다시 눌러주세요 )
                            #    sent over the wire by an action server
```                            
                            
# Robot pause / resume

```
pause : move_base/cancel
topic msg type : actionlib_msgs/GoalID
description : 별도 값없이 empty array publish 하면 됩니다.

resume : freeway/resume
topic msg type : std_msgs/Empty
description : 별도 값없이 빈값 publish 하면 됩니다.

```
