#!/usr/bin/env python3
"""Simulation placeholder: acknowledge start immediately, report completion later.

Does not publish vehicle commands or perform an NDT measurement.
"""
import math
import threading


class MeasurementJobs:
    """One active measurement; duplicate starts while active retain its deadline."""
    def __init__(self, duration):
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('duration must be finite and positive')
        self.duration = duration
        self.deadline = None

    def start(self, now):
        if self.deadline is None:
            self.deadline = now + self.duration
        return True

    def due(self, now):
        return self.deadline is not None and now >= self.deadline

    def finish(self):
        self.deadline = None


def main():
    import rospy
    from std_srvs.srv import Trigger, TriggerResponse
    rospy.init_node('mock_ndt_measurement')
    jobs = MeasurementJobs(float(rospy.get_param('~duration_s', 10.0)))
    lock = threading.Lock()
    completing = False
    completion = rospy.ServiceProxy(rospy.get_param(
        '~complete_service', '/ndt/measurement_complete'), Trigger)

    def start(request):
        with lock:
            if completing:
                return TriggerResponse(False, "Previous completion is being delivered; retry start")
            jobs.start(rospy.Time.now().to_sec())
        rospy.loginfo('MOCK NDT start trigger received')
        return TriggerResponse(True, 'Mock measurement accepted')

    def tick(event):
        nonlocal completing
        with lock:
            if not jobs.due(rospy.Time.now().to_sec()):
                return
            completing = True
        try:
            response = completion()
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(5.0, 'Mock completion service unavailable: %s', error)
            with lock:
                completing = False
            return
        with lock:
            jobs.finish()
            completing = False
        rospy.loginfo('MOCK NDT completion: success=%s (%s)', response.success, response.message)

    rospy.Service(rospy.get_param('~start_service', '/ndt/start_measurement'),
                  Trigger, start)
    rospy.Timer(rospy.Duration(0.5), tick)
    rospy.logwarn('MOCK NDT enabled: automatic completion after %.1f seconds of ROS time; no measurement or controller handover', jobs.duration)
    rospy.spin()


if __name__ == '__main__':
    main()
