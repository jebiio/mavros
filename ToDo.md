# ToDo
* arm/disarm
  * PX4 -- (heartbeat) --> mavros -- (arm/disarm) --> middle_node
* throttle/steering
  * PX4 -- (manual_control) --> mavros -- (manual_control) --> middle_node

## arm/disarm
* msg 만들기
  * heartbeat 이랑 동일하게?
* handle_heartbeat()
  * 
* issues
  * mavros에서 처리 or middle_node에서 처리

## manual control
* msg 만들기
* MANUAL_SETPOINT


## 확인하기
* handle_manual_control 으로 동작하는지...
