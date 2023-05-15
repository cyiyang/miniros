# -*- coding: utf-8 -*-

from statemachine import State, StateMachine
from statemachine.exceptions import TransitionNotAllowed


class CoolingTime(StateMachine):
    # 定义状态
    speedUpState = State("speedUpState")
    slowDownState = State("slowDownState")
    normalState = State("normalState", initial=True)

    # 定义状态转换
    SPEED_UP = slowDownState.to(normalState) | normalState.to(speedUpState)
    SLOW_DOWN = speedUpState.to(normalState) | normalState.to(slowDownState)


sm = CoolingTime()
print(sm.current_state.value)
sm.send("SPEED_UP")
print(sm.current_state.value=="speedUpState")