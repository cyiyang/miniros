from statemachine import State, StateMachine
from statemachine.exceptions import TransitionNotAllowed


class CoolingTime(StateMachine):
    # 定义状态
    speedUpState = State()
    slowDownState = State()
    normalState = State(initial=True)

    # 定义状态转换
    SPEED_UP = slowDownState.to(normalState) | normalState.to(speedUpState)
    SLOW_DOWN = speedUpState.to(normalState) | normalState.to(slowDownState)


sm = CoolingTime()
print(sm.current_state)
sm.send("SPEED_UP")
print(sm.current_state)