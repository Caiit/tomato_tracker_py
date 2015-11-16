from numpy import array
from collections import namedtuple

class TimeInterpolator(object):
    TimeState = namedtuple('TimeState', ['time', 'state'])

    def __init__(self, time, state, max_history=0):
        self.history = [TimeState(time, state)]
        self.max_history = max_history

    def update(self, time, state):
        self.history.append(TimeState(time, state))
        # Truncate history to max size
        if self.max_history and len(self.history) > self.max_history:
            self.history = self.history[:self.max_history]

    def estimate_state(time):
        raise NotImplemented("Implement estimate_state(time) in a derived class.")


class LinearInterpolator(TimeInterpolator):
    def __init__(self, time, state):
        TimeInterpolator.__init__(self, time, state, max_history=2)

    def estimate_state(time):
        if len(self.history) < 2:
            return self.history[0]

        time_diff = self.history[0].time - self.history[1].time
        velocity = (self.history[0].state - self.history[1].state) / time_diff

        future = time - self.history[0].time

        return TimeState(time,
                         self.history[0].state + velocity * future)
