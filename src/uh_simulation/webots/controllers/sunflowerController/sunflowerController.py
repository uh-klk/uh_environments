#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''
from collections import namedtuple

from threading import Thread, current_thread, RLock
import math
import time

from controller import Robot

import rospy
from sf_controller_msgs.msg import SunflowerAction, SunflowerFeedback, SunflowerResult
import actionlib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
# from p2os_msgs.msg import SonarArray
from sensor_msgs.msg import LaserScan, JointState
from dynamixel_msgs.msg import JointState as DynJointState
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from sf_controller_msgs.msg._SunflowerGoal import SunflowerGoal
from actionlib.server_goal_handle import ServerGoalHandle

ros_param_lock = RLock()


def setupPyDevTrace():
    pydevdPath = "/opt/eclipse/plugins/org.python.pydev_4.4.0.201510052309/pysrc/"
    import sys
    if not pydevdPath in sys.path:
        sys.path.append(pydevdPath)
    try:
        import pydevd
        pydevd.settrace()
    except ImportError as err:
        print err
        exit(1)
        pass  # Most probably, pydev is not installed on the system


class Sunflower(Robot):

    _actionHandles = {}
    Location = namedtuple('Location', ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'timestamp'])
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
        super(Sunflower, self).__init__()
        self._namespace = namespace.rstrip('/') + '/'
        self._timeStep = int(self.getBasicTimeStep())
        self._actionName = name
        self._actions = {
            'init': self.init,
            'move': self.move,
            'park': self.park,
            'stop': self.stop}
        self._as = actionlib.ActionServer(
            self._actionName,
            SunflowerAction,
            goal_cb=self._goalCB,
            cancel_cb=self._cancelCB,
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
            'Started Sunflower Controller ActionServer on topic %s',
            self._actionName)
        self._location = None
        self._lastLocation = None
        self._rosTime = None
        self._servos = {}
        self._staticJoints = []
        self._sensors = {}
        self._sensorValues = {}
        self._leds = {}
        self.initialise()

    def _updateLocation(self):
        if self._sensors.get('gps', None) and self._sensors.get('compass', None):
            lX, lY, lZ = self._sensors['gps'].getValues()

            if 'imu' in self._sensors:
                roll, pitch, yaw = self._sensors['imu'].getRollPitchYaw()

                # certain 'right handed' rotations don't seem to be matching up between ros and webots
                pitch = -pitch
            else:
                wX, wY, wZ = self._sensors['compass'].getValues()
                roll = pitch = 0
                # http://www.cyberbotics.com/reference/compass.php
                # certain 'right handed' rotations don't seem to be matching up between ros and webots
                yaw = -math.atan2(wX, wZ)

            # Convert to ros coordinates
            rX = lX
            rY = -lZ
            rZ = lY

            self._lastLocation = self._location
            self._location = Sunflower.Location(rX, rY, rZ, roll, pitch, yaw, self.getTime())

    def _updateSonar(self):
        if self._sensors.get('sonar', None):
            self._sensorValues['sonar'] = map(
                lambda x: x.getValue(), self._sensors['sonar'])

    def _updateLaser(self):
        if self._sensors.get('frontLaser', None):
            fov = self._sensors['frontLaser'].getFov()
            ranges = self._sensors['frontLaser'].getRangeImage()
            ranges.reverse()
            maxRange = self._sensors['frontLaser'].getMaxRange()
            sampleRate = self._sensors['frontLaser'].getSamplingPeriod() / 1000

            self._sensorValues['frontLaser'] = Sunflower.DistanceScan(
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
                (self._location.x, self._location.y, self._location.z),
                quaternion_from_euler(self._location.roll, self._location.pitch, self._location.yaw),
                self._rosTime,
                self._namespace + 'base_link',
                self._namespace + 'odom')

    def _publishLocationTransform(self, locationPublisher):
        if self._location:
            rospy.logdebug("Location: %s", self._location)
            locationPublisher.sendTransform(
                (0, 0, 0),
                quaternion_from_euler(0, 0, 0),
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

    def _publishOdom(self, odomPublisher):
        if self._location:
            msg = Odometry()
            msg.header.stamp = self._rosTime
            msg.header.frame_id = self._namespace + 'odom'
            msg.child_frame_id = self._namespace + 'base_link'

            msg.pose.pose.position.x = self._location.x
            msg.pose.pose.position.y = self._location.y
            msg.pose.pose.position.z = self._location.z

            orientation = quaternion_from_euler(
                self._location.roll, self._location.pitch, self._location.yaw)
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
            msg.pose.pose.position.x = self._location.x
            msg.pose.pose.position.y = self._location.y
            msg.pose.pose.position.z = self._location.z

            orientation = quaternion_from_euler(self._location.roll, self._location.pitch, self._location.yaw)
            msg.pose.pose.orientation.x = orientation[0]
            msg.pose.pose.orientation.y = orientation[1]
            msg.pose.pose.orientation.z = orientation[2]
            msg.pose.pose.orientation.w = orientation[3]

            posePublisher.publish(msg)
            return True
        return False

    def _publishPose(self, posePublisher):
        if self._location:
            msg = Odometry()
            msg.header.stamp = self._rosTime
            msg.header.frame_id = self._namespace + 'odom'
            msg.child_frame_id = self._namespace + 'base_link'

            msg.pose.pose.position.x = self._location.x
            msg.pose.pose.position.y = self._location.y
            msg.pose.pose.position.z = self._location.z

            orientation = quaternion_from_euler(self._location.roll, self._location.pitch, self._location.yaw)
            msg.pose.pose.orientation.x = orientation[0]
            msg.pose.pose.orientation.y = orientation[1]
            msg.pose.pose.orientation.z = orientation[2]
            msg.pose.pose.orientation.w = orientation[3]

            if self._lastLocation:
                dt = (self._location.timestamp - self._lastLocation.timestamp) / 1000
                msg.twist.twist.linear.x = (self._location.x - self._lastLocation.x) / dt
                msg.twist.twist.linear.y = (self._location.y - self._lastLocation.y) / dt
                msg.twist.twist.linear.z = (self._location.z - self._lastLocation.y) / dt
                msg.twist.twist.angular.x = (self._location.roll - self._lastLocation.roll) / dt
                msg.twist.twist.angular.y = (self._location.pitch - self._lastLocation.pitch) / dt
                msg.twist.twist.angular.z = (self._location.yaw - self._lastLocation.yaw) / dt

            posePublisher.publish(msg)

    def _publishSonar(self, sonarPublisher):
        if self._sensorValues.get('sonar', None):
            # msg = SonarArray()
            # msg.header.stamp = self._rosTime

            # msg.ranges = self._sensorValues['sonar']
            # msg.ranges_count = len(self._sensorValues['sonar'])
            # sonarPublisher.publish(msg)
            pass

    def _publishLaser(self, laserPublisher):
        if self._sensorValues.get('frontLaser', None):
            laser_frequency = 40
            msg = LaserScan()
            msg.header.stamp = self._rosTime
            msg.header.frame_id = self._namespace + 'scan_front'

            msg.ranges = self._sensorValues['frontLaser'].ranges
            msg.angle_min = self._sensorValues['frontLaser'].min_angle
            msg.angle_max = self._sensorValues['frontLaser'].max_angle
            msg.angle_increment = abs(
                msg.angle_max - msg.angle_min) / len(msg.ranges)
            msg.range_min = self._sensorValues['frontLaser'].min_range
            msg.range_max = self._sensorValues['frontLaser'].max_range
            msg.scan_time = self._timeStep
            msg.time_increment = (msg.scan_time / laser_frequency /
                                  len(self._sensorValues['frontLaser'].ranges))
            laserPublisher.publish(msg)

    def _publishJoints(self, jointPublisher, dynamixelPublishers=None):
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
        if self._servos and dynamixelPublishers:
            # getComponentState of the sunflower class uses the dynamixel contoller status
            for (name, servo) in self._servos.iteritems():
                if name in dynamixelPublishers:
                    msg = DynJointState()
                    msg.header.stamp = self._rosTime
                    msg.name = name
                    msg.goal_pos = servo.getTargetposition()
                    msg.current_pos = servo.getPosition()
                    msg.load = servo.getTorqueFeedback()
                    dynamixelPublishers[name].publish(msg)

    def run(self):
        try:
            # ROS Version >= Hydro
            odomPublisher = rospy.Publisher(self._namespace + 'odom', Odometry, queue_size=2)
            posePublisher = rospy.Publisher(self._namespace + 'pose', Odometry, queue_size=2)
            laserPublisher = rospy.Publisher(self._namespace + 'scan_front', LaserScan, queue_size=2)
            jointPublisher = rospy.Publisher(self._namespace + 'joint_states', JointState, queue_size=2)
            initialPosePublisher = rospy.Publisher(self._namespace + 'initialpose', PoseWithCovarianceStamped, queue_size=2)
            dynamixelPublishers = {name: rospy.Publisher(
                self._namespace + '%s_controller/state' % name, DynJointState, queue_size=2) for name in self._servos.iterkeys()}
        except:
            odomPublisher = rospy.Publisher(self._namespace + 'odom', Odometry)
            posePublisher = rospy.Publisher(self._namespace + 'pose', Odometry)
            laserPublisher = rospy.Publisher(self._namespace + 'scan_front', LaserScan)
            jointPublisher = rospy.Publisher(self._namespace + 'joint_states', JointState)
            initialPosePublisher = rospy.Publisher(self._namespace + 'initialpose', PoseWithCovarianceStamped)
            dynamixelPublishers = {name: rospy.Publisher(self._namespace + '%s_controller/state' % name, DynJointState) for name in self._servos.iterkeys()}
        odomTransform = TransformBroadcaster()

        initialPosePublishCount = 5
        initialPosePublishedAt = 0

        while not rospy.is_shutdown() and self.step(self._timeStep) != -1:
            self._rosTime = rospy.Time.now()

            # Give time for ROS to initialise
            if initialPosePublishCount >= 0 and abs(initialPosePublishedAt - self._rosTime.secs) >= 5:
                initialPosePublishCount -= int(self._publishInitialPose(initialPosePublisher))
                initialPosePublishedAt = self._rosTime.nsecs

            self._updateLocation()
            self._updateSonar()
            self._updateLaser()

            self._publishPose(posePublisher)

            self._publishOdom(odomPublisher)
            self._publishOdomTransform(odomTransform)

            self._publishLaser(laserPublisher)

            self._publishJoints(jointPublisher, dynamixelPublishers)

            # It appears that we have to call sleep for ROS to process messages
            time.sleep(0.0001)

    def initialise(self):
        numLeds = 3
        # numSonarSensors = 16

        self._leftWheel = self.getMotor('left_wheel')
        self._leftWheel.setPosition(float('+inf'))
        self._leftWheel.setVelocity(0)

        self._rightWheel = self.getMotor('right_wheel')
        self._rightWheel.setPosition(float('+inf'))
        self._rightWheel.setVelocity(0)

        self._staticJoints = [
            "base_swivel",
            "base_swivel_wheel",
            "base_left_wheel",
            "base_right_wheel"
        ]

        self._servos = {
            'tray': self.getMotor('tray'),
            'neck_lower': self.getMotor('neck_lower'),
            'neck_upper': self.getMotor('neck_upper'),
            'head_tilt': self.getMotor('head_tilt'),
            'head_pan': self.getMotor('head_pan'),
        }

        for servo in self._servos.values():
            servo.enablePosition(self._timeStep)

        self._sensors['frontLaser'] = self.getCamera('front_laser')
        self._sensors['frontLaser'].enable(self._timeStep)

        self._sensors['gps'] = self.getGPS('gps')
        self._sensors['gps'].enable(self._timeStep)

        self._sensors['compass'] = self.getCompass('compass')
        self._sensors['compass'].enable(self._timeStep)

        self._sensors['imu'] = self.getInertialUnit('imu')
        self._sensors['imu'].enable(self._timeStep)

        self._sensors['camera'] = self.getCamera('head_camera')
        if self._sensors['camera']:
            self._sensors['camera'].enable(self._timeStep)

        self._leds['light'] = self.getLED('light')

        self._leds['base'] = []
        for i in range(0, numLeds):
            led = self.getLED('red_led%s' % (i + 1))
            led.set(0)
            self._leds['base'].append(led)

        # self._sensors['sonar'] = []
        # for i in range(0, numSonarSensors):
        #    sensor = self.getDistanceSensor('so%s' % i)
        #    sensor.enable(self._timeStep)
        #    self._sensors['sonar'].append(sensor)

    def park(self):
        pass

    def _cancelCB(self, goalHandle):
        rospy.logwarn("Got Cancel msg")

        def statusCB(status, msg='', *args, **kwargs):
            feedback = SunflowerFeedback()
            feedback.status = status
            feedback.msg = msg if type(msg) == str else ''
            goalHandle.publish_feedback(feedback)

        def doneCB(result, msg=""):
            res = SunflowerResult(result, msg)
            goalHandle.set_canceled(res)

        self.stop(goalHandle, statusCB, doneCB)

    def _goalCB(self, goalHandle):
        goalHandle.set_accepted()
        goal = goalHandle.get_goal()

        def updateStatus(status, msg='', *args, **kwargs):
            if msg:
                rospy.logdebug("Status: %s", msg)
            feedback = SunflowerFeedback()
            feedback.status = status
            feedback.msg = msg if type(msg) == str else str(msg)
            goalHandle.publish_feedback(feedback)

        def completed(result, msg=''):
            status = goalHandle.get_goal_status().status
            msg = msg or "Completed component %s" % (goal.component, )
            updateStatus(status, msg)
            if not isinstance(result, SunflowerResult):
                r = SunflowerResult()
                if type(result) == int:
                    r.result = result
                else:
                    r.msg = str(result)
                result = r

            if status == actionlib.GoalStatus.ACTIVE:
                goalHandle.set_succeeded(result)
            elif status == actionlib.GoalStatus.PREEMPTED:
                goalHandle.set_preempted(result)
            else:
                goalHandle.set_canceled(result)

        if goal.action in self._actions:
            updateStatus(goalHandle.get_goal_status().status, "Starting action %s" % (goal.action, ))
            Thread(target=self._actions[goal.action], args=(goalHandle, updateStatus, completed)).start()
        else:
            rospy.logwarn("Unknown action %s", goal.action)
            goalHandle.set_rejected()

    def init(self, goalHandle, statusCB, doneCB):
        doneCB(actionlib.GoalStatus.SUCCEEDED)

    def stop(self, goalHandle, statusCB, doneCB):
        name = goalHandle.get_goal().component
        rospy.loginfo('%s: Stopping %s', self._actionName, name)
        if name == 'base':
            client = actionlib.SimpleActionClient(self._namespace + 'move_base', MoveBaseAction)
            client.wait_for_server()
            client.cancel_all_goals()

        doneCB(actionlib.GoalStatus.SUCCEEDED)

    def setlight(self, goalHandle, statusCB, doneCB):
        goal = goalHandle.get_goal()
        color = goal.jointPositions
        # Sunflower hardware only supports on/off states for RGB array
        # Webots selects the color as an array index of available colors
        # 3-bit color array is arranged in ascending binary order
        try:
            r = 0x4 if color[0] else 0
            g = 0x2 if color[1] else 0
            b = 0x1 if color[2] else 0
            # Webots color array is 1-indexed
            colorIndex = r + g + b + 1
            if self._leds['light']:
                self._leds['light'].set(colorIndex)
                doneCB(actionlib.GoalStatus.SUCCEEDED)
            else:
                rospy.logerr('Unable to set color.  Body LED not found.')
                doneCB(actionlib.GoalStatus.REJECTED)
        except Exception:
            rospy.logerr('Error setting color to: %s' % (color), exc_info=True)
            doneCB(actionlib.GoalStatus.ABORTED)

    def move(self, goalHandle, statusCB, doneCB):
        goal = goalHandle.get_goal()
        joints = goal.jointPositions

        if(goal.namedPosition != '' and goal.namedPosition is not None):
            param = self._namespace + 'sf_controller/' + \
                goal.component + '/' + goal.namedPosition
            if(rospy.has_param(param)):
                global ros_param_lock
                with ros_param_lock:
                    joints = rospy.get_param(param)[0]

        rospy.logdebug('%s: Setting %s to %s',
                       self._actionName,
                       goal.component,
                       goal.namedPosition or joints)

        def onDone(state=actionlib.GoalStatus.ABORTED, result=None):
            rospy.logdebug('%s: "%s to %s" Result:%s',
                           self._actionName,
                           goal.component,
                           goal.namedPosition or joints,
                           result)
            doneCB(result)

        try:
            if goal.component == 'base_pan':
                goal.component = 'base_direct'
                goal.jointPositions = [goal.jointPositions[0], 0.0]
            if goal.component == 'base':
                self.navigate(goalHandle, statusCB, onDone)
            elif goal.component == 'base_direct':
                self.moveBase(goalHandle, statusCB, onDone)
            elif goal.component in self._leds:
                self.setlight(goalHandle, statusCB, doneCB)
            else:
                self.moveJoints(goalHandle, joints, onDone)
        except Exception as e:
            rospy.logerr('Error occurred: %s' % e)
            import traceback
            traceback.print_exc()
            onDone(actionlib.GoalStatus.ABORTED, 'Error occurred: %s' % e)

    def moveBase(self, goalHandle, statusCB, doneCB):
        goal = goalHandle.get_goal()
        positions = goal.jointPositions

        LINEAR_RATE = math.pi / 2  # [rad/s]
        # WHEEL_DIAMETER = 0.195  # [m] From the manual
        WHEEL_RADIUS = 0.0975
        # BASE_SIZE = 0.3810  # [m] From the manual
        AXLE_LENGTH = 0.33  # [m] From WeBots Definition
        WHEEL_ROTATION = AXLE_LENGTH / (2 * WHEEL_RADIUS)

        rotation = round(positions[0], 4)
        linear = round(positions[1], 4)

        if not isinstance(rotation, (int, float)):
            rospy.logerr('Non-numeric rotation in list, aborting moveBase')
            doneCB(actionlib.GoalStatus.REJECTED)
            return

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

        duration = max(leftDuration, rightDuration)
        rospy.logdebug('Setting rates: L=%s, R=%s for %ss' % (leftRate, rightRate, duration))
        start_time = self.getTime()
        end_time = start_time + duration
        while not rospy.is_shutdown():
            if goalHandle.get_goal_status().status != actionlib.GoalStatus.ACTIVE:
                rospy.loginfo('%s: Preempted, status was %s' % (self._actionName, goalHandle.get_goal_status().status))
                self._rightWheel.setVelocity(0)
                self._leftWheel.setVelocity(0)
                doneCB(actionlib.GoalStatus.PREEMPTED)
                return

            self._rightWheel.setVelocity(rightRate)
            self._leftWheel.setVelocity(leftRate)

            if self.getTime() >= end_time:
                break

        self._rightWheel.setVelocity(0)
        self._leftWheel.setVelocity(0)

        doneCB(actionlib.GoalStatus.SUCCEEDED)

    def cmdVelCB(self, msg):
        # rospy.loginfo("cmdVelCB called on thread: %s", current_thread().ident)
        MAX_VEL = 5.24

        WHEEL_RADIUS = 0.0975
        AXLE_LENGTH = 0.33
        BASE_RADIUS = AXLE_LENGTH / 2

        vR = (msg.linear.x + (msg.angular.z * BASE_RADIUS * math.pi)) / WHEEL_RADIUS
        vL = (msg.linear.x - (msg.angular.z * BASE_RADIUS * math.pi)) / WHEEL_RADIUS

        if abs(vR) > MAX_VEL or abs(vL) > MAX_VEL:
            # scale vR/vL
            maxVel = max(abs(vR), abs(vL))
            scale = MAX_VEL / maxVel
            vR *= scale
            vL *= scale

        rospy.logdebug('Setting rates: L=%s, R=%s' % (vL, vR))
        self._rightWheel.setVelocity(vR)
        self._leftWheel.setVelocity(vL)

    def navigate(self, goalHandle, statusCB, doneCB):
        positions = goalHandle.get_goal().jointPositions
        rospy.logdebug("navigate called on thread: %s", current_thread().ident)
        pose = PoseStamped()
        pose.header.stamp = self._rosTime
        pose.header.frame_id = 'map'
        if len(positions) < 2:
            try:
                raise ValueError("Positions too short")
            except:
                import traceback
                traceback.format_exc()

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
        rospy.logdebug('%s: Navigating to (%s, %s, %s)',
                       self._actionName,
                       positions[0],
                       positions[1],
                       positions[2])

        client.send_goal(client_goal, done_cb=doneCB)
        activeStates = [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]
        while True:
            status = client.get_state()
            rospy.logdebug("NavClient (%s): %s", positions, status)
            if status not in activeStates:
                if status == actionlib.GoalStatus.ABORTED:
                    goalHandle.set_aborted()
                elif status == actionlib.GoalStatus.PREEMPTED:
                    rospy.logdebug("NavClient Preempted")
                    goalHandle.set_canceled()
                    rospy.logdebug("NavClient Set_Canceled")
                elif status == actionlib.GoalStatus.REJECTED:
                    goalHandle.set_rejected()
                rospy.logdebug("NavClient Done")
                doneCB(status)
                break
            sstatus = goalHandle.get_goal_status().status
            print "Server (%s): %s" % (positions, sstatus, )
            if sstatus not in activeStates:
                print "Cancelling"
                client.cancel_goal()
            else:
                rospy.sleep(0.1)

    def moveJoints(self, goalHandle, statusCB, doneCB):
        component = goalHandle.get_goal().component
        positions = goalHandle.get_goal().jointPositions

        try:
            param_name = self._namespace + 'sf_controller/%s/joint_names' % component
            global ros_param_lock
            with ros_param_lock:
                joint_names = rospy.get_param(param_name)
        except KeyError:
            # TODO: we're not publishing the dynamixel yaml configs for webots
            if component == 'head':
                joint_names = ['head_pan', 'head_tilt', 'neck_upper', 'neck_lower']
            else:
                # assume component is a named joint
                joint_names = [component, ]
        for i in range(0, len(joint_names)):
            servoName = joint_names[i]
            if servoName not in self._servos:
                rospy.logerr('Undefined joint %s', servoName)
                doneCB(actionlib.GoalStatus.ABORTED)
                return
            self._servos[servoName].setPosition(positions[i])

        inpos_threshold = 0.1
        waittime = 5
        waitstart = rospy.Time.now()
        while (rospy.Time.now() - waitstart).to_sec() <= waittime:
            done = True
            for i in range(0, len(joint_names)):
                current = self._servos[joint_names[i]].getPosition()
                target = positions[i]
                done = done and abs(current - target) <= inpos_threshold
            if done:
                doneCB(actionlib.GoalStatus.SUCCEEDED)
                return

        doneCB(actionlib.GoalStatus.ABORTED)

if __name__ == '__main__':
    rospy.init_node('sf_controller')
    sf = Sunflower(rospy.get_name(), namespace="sunflower1_1")
    sf.run()
