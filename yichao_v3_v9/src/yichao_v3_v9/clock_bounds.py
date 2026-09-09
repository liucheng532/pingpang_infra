"""Bound workstation/robot monotonic offset using four timestamp exchanges."""
from .inputs import finite


class ClockBounds:
    def __init__(self):
        self.lower = self.upper = self.established_at = None

    def update(self, workstation_send, robot_receive, robot_send, workstation_receive):
        w0, r1, r2, w3 = [finite(v, 'clock_exchange') for v in
                          (workstation_send, robot_receive, robot_send, workstation_receive)]
        if not w0 <= w3 or not r1 <= r2:
            raise ValueError('clock_exchange_order')
        lower, upper = r2-w3, r1-w0
        if lower > upper or upper-lower > .02:
            raise ValueError('clock_exchange_exceeds_20ms_uncertainty')
        if self.lower is not None and (lower > self.upper+.002 or upper < self.lower-.002):
            raise ValueError('clock_jump_new_session_required')
        self.lower, self.upper, self.established_at = lower, upper, w3
        return {'robot_minus_workstation_lower': lower, 'robot_minus_workstation_upper': upper,
                'workstation_established_at': w3}

    def workstation_interval(self, robot_now):
        if self.lower is None:
            raise ValueError('clock_exchange_missing')
        now = finite(robot_now, 'robot_now')
        early, late = now-self.upper, now-self.lower
        if early < self.established_at-.02 or late-self.established_at > 1.:
            raise ValueError('clock_exchange_lease_expired')
        # 100 ppm drift envelope over a one-second lease; reset on disagreement.
        return (early-.0001, late+.0001)
