#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''

import time
import math

from controller import Robot
try:
    import os
    path = os.path.dirname(os.path.realpath(__file__))
    if 'ROS_PACKAGE_PATH' not in os.environ:
        os.environ['ROS_PACKAGE_PATH'] = ''
    if os.environ['ROS_PACKAGE_PATH'].find(path) == -1:
        os.environ['ROS_PACKAGE_PATH'] = ':'.join((path, os.environ['ROS_PACKAGE_PATH']))
    roslib = __import__('roslib', globals(), locals())
    roslib.load_manifest('clockController')
except:
    import logging
    logger = logging.getLogger()
    if logger.handlers:
        logging.getLogger().error(
            'Unable to load roslib, fatal error', exc_info=True)
    else:
        import sys
        import traceback
        print >> sys.stderr, 'Unable to load roslib, fatal error'
        print >> sys.stderr, traceback.format_exc()
    exit(1)
else:
    import rospy
    from rosgraph_msgs.msg import Clock


class ClockSync(Robot):

    def __init__(self):
        super(ClockSync, self).__init__()
        self._timeStep = int(self.getBasicTimeStep())

        self._minHand = self.getMotor('clock_min')
        self._minHand.setPosition(0)

    def run(self):
        # We only want one clock publisher
        rospy.loginfo("Starting clock sync")
        try:
            # ROS Version >= Hydro
            clockPublisher = rospy.Publisher('/clock', Clock, queue_size=2)
        except:
            clockPublisher = rospy.Publisher('/clock', Clock)

        starttime = time.time()
        TWO_PI = math.pi * 2
        OFFSET = math.pi / 2
        convFact = TWO_PI / -60

        while not rospy.is_shutdown() and self.step(self._timeStep) != -1:
            rosTime = rospy.Time(self.getTime() + starttime)
            clock = Clock(clock=rosTime)
            clockPublisher.publish(clock)
            self._minHand.setPosition(((rosTime.to_sec() % 60) * convFact) - OFFSET)
            time.sleep(0.001)

if __name__ == '__main__':
    rospy.init_node('clockSync')
    sf = ClockSync()
    sf.run()
