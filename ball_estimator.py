from collections import namedtuple

BallState = namedtuple('BallState', ['time', 'pos_x', 'pos_y'])

class BallEstimator(object):
    """Simple ball position estimator, to aid detection."""
    def __init__(self, time, pos_x, pos_y):
        self.history = [BallState(time, pos_x, pos_y)]

    def update(self, time, pos_x, pos_y):
        self.history.append(BallState(time, pos_x, pos_y))

    def estimate_state(time):
        if len(self.history) < 2:
            return self.history[0]

        time_diff = self.history[0].time - self.history[1].time
        vel_x = (self.history[0].pos_x - self.history[1].pos_x) / time_diff
        vel_y = (self.history[0].pos_y - self.history[1].pos_y) / time_diff

        time_diff = time - self.history[0].time
        return BallState(time,
                         self.history[0].pos_x + vel_x*time,
                         self.history[0].pos_y + vel_y*time)
