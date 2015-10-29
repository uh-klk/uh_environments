#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''
from collections import namedtuple

from threading import Thread
import math
import time

from controller import Robot
try:
    import os
    path = os.path.dirname(os.path.realpath(__file__))
    if 'ROS_PACKAGE_PATH' not in os.environ:
        os.environ['ROS_PACKAGE_PATH'] = ''
    if os.environ['ROS_PACKAGE_PATH'].find(path) == -1:
        os.environ['ROS_PACKAGE_PATH'] = ':'.join((path, os.environ['ROS_PACKAGE_PATH']))
    roslib = __import__('roslib', globals(), locals())
    roslib.load_manifest('personController')
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
    import sf_controller_msgs.msg
    import actionlib
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, JointState
    from tf import TransformBroadcaster
    from tf.transformations import quaternion_from_euler

_states = {
    0: 'PENDING',
    'PENDING': 0,
    1: 'ACTIVE',
    'ACTIVE': 1,
    2: 'PREEMPTED',
    'PREEMPTED': 2,
    3: 'SUCCEEDED',
    'SUCCEEDED': 3,
    4: 'ABORTED',
    'ABORTED': 4,
    5: 'REJECTED',
    'REJECTED': 5,
    6: 'PREEMPTING',
    'PREEMPTING': 6,
    7: 'RECALLING',
    'RECALLING': 7,
    8: 'RECALLED',
    'RECALLED': 8,
    9: 'LOST',
    'LOST': 9,
}


class Person(Robot):

    _actionHandles = {}
    Location = namedtuple('Location', ['x', 'y', 'theta', 'timestamp'])
    DistanceScan = namedtuple('DistanceScan', [
                              'min_angle',
                              'max_angle',
                              'min_range',
                              'max_range',
                              'scan_time',
                              'ranges'])

    # TODO: These should be in a config file
    # Speed limits from navigation files
    _translationSpeed = [-0.2, 0.4]
    _rotationSpeed = [-0.8, 0.8]

    def __init__(self, name, namespace='/'):
        super(Person, self).__init__()
        self._namespace = namespace.rstrip('/') + '/'
        self._timeStep = int(self.getBasicTimeStep())
        self._actionName = name
        self._as = actionlib.SimpleActionServer(
            self._actionName,
            sf_controller_msgs.msg.SunflowerAction,
            execute_cb=self.executeCB,
            auto_start=False)
        self._as.start()
        try:
            # ROS Version >= Hydro
            self._cmdVel = rospy.Subscriber(
                self._namespace + 'cmd_vel',
                Twist,
                callback=self.cmdVelCB,
                queue_size=2)
        except:
            self._cmdVel = rospy.Subscriber(
                self._namespace + 'cmd_vel',
                Twist,
                callback=self.cmdVelCB)

        rospy.loginfo(
            'Started Person Controller ActionServer on topic %s',
            self._actionName)
        self._feedback = sf_controller_msgs.msg.SunflowerFeedback()
        self._result = sf_controller_msgs.msg.SunflowerResult()
        self._location = None
        self._lastLocation = None
        self._rosTime = None
        self._servos = {}
        self._sensors = {}
        self._staticJoints = []
        self._sensorValues = {}
        self._leds = {}
        self.initialise()
        self.pos = 0.0

    def _updateLocation(self):
        if self._sensors.get('gps', None) and self._sensors.get('compass', None):
            lX, lY, lZ = self._sensors['gps'].getValues()
            wX, _, wZ = self._sensors['compass'].getValues()

            # http://www.cyberbotics.com/reference/section3.13.php
            bearing = math.atan2(wX, wZ) - (math.pi / 2)
            x, y, _, rotation = self.webotsToRos(lX, lY, lZ, bearing)

            self._lastLocation = self._location
            self._location = Person.Location(x, y, rotation, self.getTime())

    def _updateLaser(self):
        if self._sensors.get('frontLaser', None):
            fov = self._sensors['frontLaser'].getFov()
            ranges = self._sensors['frontLaser'].getRangeImage()
            ranges.reverse()
            maxRange = self._sensors['frontLaser'].getMaxRange()
            sampleRate = self._sensors['frontLaser'].getSamplingPeriod() / 1000

            self._sensorValues['frontLaser'] = Person.DistanceScan(
                fov / -2,
                fov / 2,
                0,
                maxRange,
                sampleRate,
                ranges
            )

        if self._sensors.get('backLaser', None):
            fov = self._sensors['backLaser'].getFov()
            ranges = self._sensors['backLaser'].getRangeImage()
            ranges.reverse()
            maxRange = self._sensors['backLaser'].getMaxRange()
            sampleRate = self._sensors['backLaser'].getSamplingPeriod() / 1000

            self._sensorValues['backLaser'] = Person.DistanceScan(
                fov / -2,
                fov / 2,
                0,
                maxRange,
                sampleRate,
                ranges
            )

    def _publishOdomTransform(self, odomPublisher):
        if self._location:
            odomPublisher.sendTransform(
                (self._location.x, self._location.y, 0),
                quaternion_from_euler(0, 0, self._location.theta),
                self._rosTime,
                self._namespace + 'base_link',
                self._namespace + 'odom')

    def _publishLocationTransform(self, locationPublisher):
        if self._location:
            rospy.loginfo("Location: %s", self._location)
            locationPublisher.sendTransform(
                (0, 0, 0),
                quaternion_from_euler(0, 0, 1.57079),
                self._rosTime,
                self._namespace + 'odom',
                'map',)

    def _publishLaserTransform(self, laserPublisher):
        if self._location:
            laserPublisher.sendTransform(
                (0.0, 0.0, 0.0),
                quaternion_from_euler(0, 0, 0),
                self._rosTime,
                self._namespace + 'scan_front',
                self._namespace + 'base_laser_front_link')
            laserPublisher.sendTransform(
                (0.0, 0.0, 0.0),
                quaternion_from_euler(0, 0, 0),
                self._rosTime,
                self._namespace + 'scan_back',
                self._namespace + 'base_laser_back_link')

    def _publishOdom(self, odomPublisher, frame_id='odom', child_frame='base_link'):
        if self._location:
            msg = Odometry()
            msg.header.stamp = self._rosTime
            msg.header.frame_id = self._namespace + frame_id
            msg.child_frame_id = self._namespace + child_frame

            msg.pose.pose.position.x = self._location.x
            msg.pose.pose.position.y = self._location.y
            msg.pose.pose.position.z = 0

            orientation = quaternion_from_euler(
                0, 0, self._location.theta)
            msg.pose.pose.orientation.x = orientation[0]
            msg.pose.pose.orientation.y = orientation[1]
            msg.pose.pose.orientation.z = orientation[2]
            msg.pose.pose.orientation.w = orientation[3]

            odomPublisher.publish(msg)
        else:
            rospy.logerr('Skipped updating odom! Last: %s, Cur: %s' %
                         (self._location, self._lastLocation))

    def _publishInitialPose(self, posePublisher):
        if self._location:
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self._rosTime
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = -self._location.y
            msg.pose.pose.position.y = self._location.x
            msg.pose.pose.position.z = 0

            orientation = quaternion_from_euler(
                0, 0, self._location.theta + 1.57079)
            msg.pose.pose.orientation.x = orientation[0]
            msg.pose.pose.orientation.y = orientation[1]
            msg.pose.pose.orientation.z = orientation[2]
            msg.pose.pose.orientation.w = orientation[3]

            posePublisher.publish(msg)
            return True
        return False

    def _publishPose(self, posePublisher, frame_id='odom', child_frame='base_link'):
        if self._location:
            msg = Odometry()
            msg.header.stamp = self._rosTime
            msg.header.frame_id = self._namespace + frame_id
            msg.child_frame_id = self._namespace + child_frame

            msg.pose.pose.position.x = self._location.x
            msg.pose.pose.position.y = self._location.y
            msg.pose.pose.position.z = 0

            orientation = quaternion_from_euler(
                0, 0, self._location.theta)
            msg.pose.pose.orientation.x = orientation[0]
            msg.pose.pose.orientation.y = orientation[1]
            msg.pose.pose.orientation.z = orientation[2]
            msg.pose.pose.orientation.w = orientation[3]

            if self._lastLocation:
                dt = (
                    self._location.timestamp - self._lastLocation.timestamp) / 1000
                msg.twist.twist.linear.x = (
                    self._location.x - self._lastLocation.x) / dt
                msg.twist.twist.linear.y = (
                    self._location.y - self._lastLocation.y) / dt
                msg.twist.twist.angular.x = (
                    self._location.theta - self._lastLocation.theta) / dt

            posePublisher.publish(msg)

    def _publishLaser(self, laserPublisher, laserName):
        if self._sensorValues.get(laserName, None):
            laser_frequency = 40
            msg = LaserScan()
            msg.header.stamp = self._rosTime
            msg.header.frame_id = laserPublisher.resolved_name

            msg.ranges = self._sensorValues[laserName].ranges
            msg.angle_min = self._sensorValues[laserName].min_angle
            msg.angle_max = self._sensorValues[laserName].max_angle
            msg.angle_increment = abs(
                msg.angle_max - msg.angle_min) / len(msg.ranges)
            msg.range_min = self._sensorValues[laserName].min_range
            msg.range_max = self._sensorValues[laserName].max_range
            msg.scan_time = self._timeStep
            msg.time_increment = (msg.scan_time / laser_frequency /
                                  len(self._sensorValues[laserName].ranges))
            laserPublisher.publish(msg)

    def _publishJoints(self, jointPublisher):
        # header:
        #   seq: 375
        #   stamp:
        #     secs: 1423791124
        #     nsecs: 372004985
        #   frame_id: ''
        # name: ['head_pan_joint', 'neck_lower_joint', 'swivel_hubcap_joint', 'base_swivel_joint', 'head_tilt_joint', 'neck_upper_joint']
        # position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # velocity: []
        # effort: []
        if (self._servos or self._staticJoints) and jointPublisher:
            msg = JointState()
            msg.header.stamp = self._rosTime
            names = []
            positions = []
            for (name, servo) in self._servos.iteritems():
                names.append('%s_joint' % name)
                # 'or 0.0' to prevent null from being published
                positions.append(servo.getPosition() or 0.0)
            for name in self._staticJoints:
                names.append('%s_joint' % name)
                # 'or 0.0' to prevent null from being published
                positions.append(0.0)

            msg.name = names
            msg.position = positions
            jointPublisher.publish(msg)

    def run(self):
        try:
            # ROS Version >= Hydro
            odomPublisher = rospy.Publisher(self._namespace + 'odom', Odometry, queue_size=2)
            posePublisher = rospy.Publisher(self._namespace + 'pose', Odometry, queue_size=2)
            frontLaserPublisher = rospy.Publisher(self._namespace + 'scan_front', LaserScan, queue_size=2)
            backLaserPublisher = rospy.Publisher(self._namespace + 'scan_back', LaserScan, queue_size=2)
            jointPublisher = rospy.Publisher(self._namespace + 'joint_states', JointState, queue_size=2)
            initialPosePublisher = rospy.Publisher(self._namespace + 'initialpose', PoseWithCovarianceStamped, queue_size=2)
        except:
            odomPublisher = rospy.Publisher(self._namespace + 'odom', Odometry)
            posePublisher = rospy.Publisher(self._namespace + 'pose', Odometry)
            frontLaserPublisher = rospy.Publisher(self._namespace + 'scan_front', LaserScan)
            backLaserPublisher = rospy.Publisher(self._namespace + 'scan_back', LaserScan)
            jointPublisher = rospy.Publisher(self._namespace + 'joint_states', JointState)
            initialPosePublisher = rospy.Publisher(self._namespace + 'initialpose', PoseWithCovarianceStamped)
        odomTransform = TransformBroadcaster()
        # laserTransform = TransformBroadcaster()
        initialposepublished = False

        while not rospy.is_shutdown() and self.step(self._timeStep) != -1:
            # self._rosTime = rospy.Time(self.getTime())
            self._rosTime = rospy.Time.now()

            # Give time for ros to initialise
            if not initialposepublished and self._rosTime.secs >= 5:
                initialposepublished = self._publishInitialPose(initialPosePublisher)

            self._updateLocation()
            self._updateLaser()

            self._publishPose(posePublisher)
            self._publishOdom(odomPublisher)
            self._publishLaser(frontLaserPublisher, 'frontLaser')
            self._publishLaser(backLaserPublisher, 'backLaser')
            self._publishJoints(jointPublisher)

            self._publishOdomTransform(odomTransform)
            # self._publishLaserTransform(laserTransform)

            # It appears that we have to call sleep for ROS to process messages
            time.sleep(0.0001)

    def webotsToRos(self, x, y, z, theta):
        rX = -z
        rY = -x
        rZ = y
        theta = -1 * theta
        return (rX, rY, rZ, theta)

    def initialise(self):
        self._leds = {
            'body': self.getLED('light')
        }
        self._wheels = {
            'left': {
                'front': self.getMotor('fl_wheel'),
                'back': self.getMotor('rl_wheel'),
            },
            'right': {
                'front': self.getMotor('fr_wheel'),
                'back': self.getMotor('rr_wheel'),
            }
        }

        for location in self._wheels.itervalues():
            for wheel in location.itervalues():
                wheel.setPosition(float('+inf'))
                wheel.setVelocity(0)

        self._lasers = ['frontLaser', 'baclLaser']
        self._sensors = {
            'frontLaser': self.getCamera('front_laser'),
            'backLaser': self.getCamera('back_laser'),
            'gps': self.getGPS('person_gps'),
            'compass': self.getCompass('person_compass'),
            'camera': self.getCamera('head_camera'),
        }
        self._staticJoints = ["base_front_right_wheel", "base_front_left_wheel", "base_rear_right_wheel", "base_rear_left_wheel"]

        for sensor in self._sensors.itervalues():
            if(sensor):
                sensor.enable(self._timeStep)

    def park(self):
        pass

    def executeCB(self, goal):
        # rospy.loginfo('Got goal: %s' % goal)
        if goal.component == 'light':
            result = self.setLight(goal.jointPositions)
        elif goal.action == 'move':
            result = self.move(goal)
        elif goal.action == 'init':
            result = self.init(goal.component)
        elif goal.action == 'stop':
            self.stop(goal.component)
            result = True
        elif goal.action == 'park':
            self.park()
            result = True
        else:
            rospy.logwarn('Unknown action %s', goal.action)
            self._result.result = -1
            self._as.set_aborted(self._result)

        if result:
            self._result.result = 0
            self._as.set_succeeded(self._result)
        else:
            self._result.result = -1
            self._as.set_aborted(self._result)

    def init(self, name):
        return True

    def stop(self, name):
        rospy.loginfo('%s: Stopping %s', self._actionName, name)
        if name == 'base':
            client = actionlib.SimpleActionClient(self._namespace + 'move_base', MoveBaseAction)
            client.wait_for_server()
            client.cancel_all_goals()
        else:
            pass

    def setLight(self, color):
        # Webots selects the color as an array index of available colors
        # 3-bit color array is arranged in ascending binary order
        try:
            r = 0x4 if color[0] else 0
            g = 0x2 if color[1] else 0
            b = 0x1 if color[2] else 0
            # Webots color array is 1-indexed
            colorIndex = r + g + b + 1
            if self._leds['body']:
                self._leds['body'].set(colorIndex)
                return True
            else:
                rospy.logerr('Unable to set color.  Body LED not found.')
                return False
        except Exception:
            rospy.logerr('Error setting color to: %s' % (color), exc_info=True)
            return False

    def move(self, goal):
        joints = goal.jointPositions

        if(goal.namedPosition != '' and goal.namedPosition is not None):
            param = self._namespace + 'sf_controller/' + \
                goal.component + '/' + goal.namedPosition
            if(rospy.has_param(param)):
                joints = rospy.get_param(param)[0]

        rospy.loginfo('%s: Setting %s to %s',
                      self._actionName,
                      goal.component,
                      goal.namedPosition or joints)

        try:
            if goal.component == 'base':
                result = self.navigate(goal, joints)
            elif goal.component == 'base_direct':
                result = self.moveBase(goal, joints)
            else:
                result = 4  # Aborted

            rospy.logdebug('%s: "%s to %s" Result:%s',
                           self._actionName,
                           goal.component,
                           goal.namedPosition or joints,
                           result)
        except Exception as e:
            rospy.logerr('Error occurred: %s' % e)
            return False

        return result == _states['SUCCEEDED']

    def moveBase(self, goal, positions):
        maxTransNeg, maxTransPos = self._translationSpeed
        maxRotNeg, maxRotPos = self._rotationSpeed

        LINEAR_RATE = math.pi / 2  # [rad/s]
        WHEEL_RADIUS = 0.10
        # AXEL_LENGTH = 0.36
        # WHEEL_ROTATION = AXEL_LENGTH / (2 * WHEEL_RADIUS)
        # Logically, this feels like it should be axle_length / (2 * wheel_radius), but that doesn't work
        # rotation_factor from guess and check
        WHEEL_ROTATION = 9.5

        rotation = round(positions[0], 4)
        linear = round(positions[1], 4)

        if not isinstance(rotation, (int, float)):
            rospy.logerr('Non-numeric rotation in list, aborting moveBase')
            return _states['ABORTED']
        elif not isinstance(linear, (int, float)):
            rospy.logerr('Non-numeric translation in list, aborting moveBase')
            return _states['ABORTED']
        if linear > maxTransPos:
            rospy.logerr(
                'Maximal relative translation step exceeded(max: %sm, requested: %sm), '
                'aborting moveBase' % (maxTransPos, linear))
            return _states['ABORTED']
        if linear < maxTransNeg:
            rospy.logerr(
                'Minimal relative translation step exceeded(min: %sm, requested: %sm), '
                'aborting moveBase' % (maxTransNeg, linear))
            return _states['ABORTED']
        if rotation > maxRotPos:
            rospy.logerr(
                'Maximal relative rotation step exceeded(max: %srad, requested: %sm), '
                'aborting moveBase' % (maxRotPos, rotation))
            return _states['ABORTED']
        if rotation < maxRotNeg:
            rospy.logerr(
                'Maximal relative rotation step exceeded(max: %srad, requested: %sm), '
                'aborting moveBase' % (maxRotNeg, rotation))
            return _states['ABORTED']

        rotRads = rotation * WHEEL_ROTATION
        linearRads = linear / WHEEL_RADIUS

        leftDuration = (rotRads + linearRads) / LINEAR_RATE
        rightDuration = ((-1 * rotRads) + linearRads) / LINEAR_RATE

        leftRate = LINEAR_RATE
        rightRate = LINEAR_RATE
        if leftDuration < 0:
            leftRate = -1 * leftRate
            leftDuration = abs(leftDuration)
        if rightDuration < 0:
            rightRate = -1 * rightRate
            rightDuration = abs(rightDuration)
        if leftDuration != rightDuration:
            if leftDuration < rightDuration:
                leftRate = leftRate * (leftDuration / rightDuration)
            else:
                rightRate = rightRate * (rightDuration / leftDuration)

        rospy.loginfo('Setting rates: L=%s, R=%s' % (leftRate, rightRate))
        duration = max(leftDuration, rightDuration)
        start_time = self.getTime()
        end_time = start_time + duration
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._actionName)
                for wheel in self._wheels['right'].itervalues():
                    wheel.setVelocity(0)
                for wheel in self._wheels['left'].itervalues():
                    wheel.setVelocity(0)
                # self._as.set_preempted()
                return _states['PREEMPTED']

            for wheel in self._wheels['right'].itervalues():
                wheel.setVelocity(rightRate)
            for wheel in self._wheels['left'].itervalues():
                wheel.setVelocity(leftRate)

            if self.getTime() >= end_time:
                break

        for wheel in self._wheels['right'].itervalues():
            wheel.setVelocity(0)
        for wheel in self._wheels['left'].itervalues():
            wheel.setVelocity(0)

        return _states['SUCCEEDED']

    def cmdVelCB(self, msg):
        WHEEL_RADIUS = 0.10
        AXLE_LENGTH = 0.36
        BASE_RADIUS = AXLE_LENGTH / 2

        vR = (msg.linear.x + (msg.angular.z * BASE_RADIUS * math.pi)) / WHEEL_RADIUS
        vL = (msg.linear.x - (msg.angular.z * BASE_RADIUS * math.pi)) / WHEEL_RADIUS

        rospy.logdebug('Setting rates: L=%s, R=%s' % (vL, vR))
        for wheel in self._wheels['right'].itervalues():
            wheel.setVelocity(vR)
        for wheel in self._wheels['left'].itervalues():
            wheel.setVelocity(vL)

    def navigate(self, goal, positions):
        pose = PoseStamped()
        pose.header.stamp = self._rosTime
        pose.header.frame_id = 'map'
        pose.pose.position.x = positions[0]
        pose.pose.position.y = positions[1]
        pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, positions[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        client = actionlib.SimpleActionClient(self._namespace + 'move_base', MoveBaseAction)
        client_goal = MoveBaseGoal()
        client_goal.target_pose = pose

        client.wait_for_server()
        rospy.loginfo('%s: Navigating to (%s, %s, %s)',
                      self._actionName,
                      positions[0],
                      positions[1],
                      positions[2])

        handle = _ActionHandle(client)
        client.send_goal(client_goal)
        handle.wait()
        return handle.result


class _ActionHandle(object):
    # ------------------- action_handle section ------------------- #
    # Action handle class.
    #
    # The action handle is used to implement asynchronous behaviour within the
    # script.

    def __init__(self, simpleActionClient):
        # Initialises the action handle.
        self._client = simpleActionClient
        self._waiting = False
        self._result = None

    @property
    def result(self):
        if self._waiting:
            return None

        return self._result

    def wait(self, duration=None):
        t = self.waitAsync(duration)
        t.join()

    def waitAsync(self, duration=None):
        thread = Thread(target=self._waitForFinished, args=(duration,))
        thread.setDaemon(True)
        thread.start()
        return thread

    def _waitForFinished(self, duration):
        self._waiting = True
        if duration is None:
            self._client.wait_for_result()
        else:
            self._client.wait_for_result(rospy.Duration(duration))

        self._result = self._client.get_state()
        self._waiting = False

    def cancel(self):
        self._client.cancel_all_goals()


if __name__ == '__main__':
    rospy.init_node('person_controller')
    sf = Person(rospy.get_name(), namespace="person")
    sf.run()
