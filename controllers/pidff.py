from . import BaseController
import numpy as np
from tinyphysics import TinyPhysicsModel

class Controller(BaseController):
    """A simple PID controller with feedforward"""

    def __init__(self):
        self.p = 0.3
        self.i = 0.05
        self.d = -0.1
        self.f = -0.01
        self.error_integral = 0
        self.prev_error = 0
        self.model = TinyPhysicsModel('./models/tinyphysics.onnx',
                debug=True)

    def set_history(self, state_target_futureplans, data):
        self.state_history = [x[0] for x in state_target_futureplans]
        self.action_history = data['steer_command'].values[:20].tolist()
        self.lataccel_history = [x[1] for x in state_target_futureplans]

    def update(
        self,
        target_lataccel,
        current_lataccel,
        state,
        future_plan,
        ):
        
        # PID
        if future_plan.lataccel:
            future_target = future_plan.lataccel[0]
            error = future_target - current_lataccel
        else:
            error = target_lataccel - current_lataccel
        self.error_integral += error
        error_diff = error - self.prev_error
        self.prev_error = error
        pid_term = self.p * error + self.i * self.error_integral \
            + self.d * error_diff

        # FF

        # if future_plan.lataccel:
        #     future_current = self.model.get_current_lataccel(self.state_history[-20:],
        #                 self.action_history[-20:], self.lataccel_history[-20:])
        #     future_target = future_plan.lataccel[0]
        #     future_error = future_target - future_current
        # else:
        #     future_error = 0
        
        # ff_term = self.f * future_error
        
        ff_term = 0

        # Update history

        # self.state_history.append(state)
        # self.action_history.append(pid_term)
        # self.lataccel_history.append(current_lataccel)

        return pid_term + ff_term
        

