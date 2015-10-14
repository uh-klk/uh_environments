#! /usr/bin/env python

import sys
import copy

from math import pi
from collections import namedtuple

import rospy

from PyKDL import Rotation
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
import visualization_msgs


class LocationPublisher(object):

    def __init__(self, rate, topic, sources):
        self._rate = rate
        self._publisher = rospy.Publisher(topic, MarkerArray, queue_size=2)
        self._sourceTopics = {}
        self._sourceParams = {}
        self._sourceFixed = {}
        self._markers = self._processSources(sources)

    def __del__(self):
        if self._publisher:
            self._publisher.unregister()

    def _processSources(self, sources):
        i = 0
        markers = {}
        for source in sources:
            i += 1
            if not isinstance(source, dict):
                rospy.logerr("Sources must be a dictionary list")
                return None
            sourceType = source.get('type', None)
            if not sourceType:
                rospy.logwarn("Source missing type param, skipping.  %s", source)
                continue
            if sourceType == 'topic':
                topic = source.get('topic', None)
                if not topic:
                    rospy.logwarn("Topic source missing topic, skipping.  %s", source)
                    continue

                key = 'topic_%s' % (topic, )
                self._sourceTopics[topic] = rospy.Subscriber(topic, PoseStamped, callback=self._updatePoseSource, callback_args=key)
            elif sourceType == 'param':
                param = source.get('param', None)
                if not param:
                    rospy.logwarn("Param source missing param, skipping.  %s", source)
                    continue

                key = 'param_%s' % (param, )
                self._sourceParams[key] = param
            elif sourceType == 'fixed':
                key = 'fixed_%s' % (i, )

            marker = Marker()

            marker.header.stamp = rospy.Time(0)
            marker.header.frame_id = source.get('marker_frame', 'map')

            marker.ns = source.get('marker_ns', '')
            marker.id = int(source.get('marker_id', i))
            mType = source.get('marker_type', Marker.SPHERE)
            if isinstance(mType, str):
                mType = getattr(Marker, mType.upper())
            marker.type = mType

            mAction = source.get('marker_action', Marker.ADD)
            if isinstance(mType, str):
                mType = getattr(Marker, mType.upper())
            marker.action = mAction

            marker.pose.position.x = source.get('marker_position', {}).get('x', 0.0)
            marker.pose.position.y = source.get('marker_position', {}).get('y', 0.0)
            marker.pose.position.z = source.get('marker_position', {}).get('z', 0.0)

            marker.pose.orientation = self._getOrientation(source.get('marker_orientation', None))

            marker.scale.x = source.get('marker_scale', {}).get('x', 0.1)
            marker.scale.y = source.get('marker_scale', {}).get('y', 0.1)
            marker.scale.z = source.get('marker_scale', {}).get('z', 0.1)

            marker.color.r = source.get('marker_color', {}).get('r', 1.0)
            marker.color.g = source.get('marker_color', {}).get('g', 1.0)
            marker.color.b = source.get('marker_color', {}).get('b', 1.0)
            marker.color.a = source.get('marker_color', {}).get('a', 1.0)

            marker.lifetime = rospy.Duration(source.get('marker_lifetime', 0))
            marker.frame_locked = bool(source.get('marker_framelocked', False))

            # TODO: points
            # TODO: colors

            marker.text = source.get('marker_text', '')

            marker.mesh_resource = source.get('marker_mesh', '')
            marker.mesh_use_embedded_materials = bool(source.get('marker_mesh_resources', False))
            markers[key] = marker

            if marker.text and marker.type != Marker.TEXT_VIEW_FACING:
                # Add second marker as a label
                m2 = copy.deepcopy(marker)
                m2.ns += ('/' if m2.ns else '') + 'labels'
                m2.scale.z = 0.1
                m2.pose.position.z += marker.scale.z + m2.scale.z * 0.5
                m2.type = Marker.TEXT_VIEW_FACING
                m2.color.r = source.get('label_color', {}).get('r', 1.0)
                m2.color.g = source.get('label_color', {}).get('g', 1.0)
                m2.color.b = source.get('label_color', {}).get('b', 1.0)
                m2.color.a = source.get('label_color', {}).get('a', 1.0)
            markers[key + '_label'] = m2

        return markers

    def _getOrientation(self, data):
        q = Quaternion()
        if data:
            if 'theta' in data:
                # Assume Yaw Orientation
                orientation = Rotation.RotZ(data.get('theta'))
            elif any(k in ['roll', 'pitch', 'yaw'] for k in data.iterkeys()):
                # Assume RPY Orientation
                orientation = Rotation.RPY(data.get('roll', 0),
                                           data.get('pitch', 0),
                                           data.get('yaw', 0))
            elif any(k in ['w', 'x', 'y', 'z'] for k in data.iterkeys()):
                orientation = Rotation.Quaternion(data.get('x', 0),
                                                  data.get('y', 0),
                                                  dat.get('z', 0),
                                                  data.get('w', 0))
            else:
                orientation = Rotation()

            orientation = orientation.GetQuaternion()
            q.x, q.y, q.z, q.w = orientation
        return q

    def _updatePoseSource(self, msg, key):
        marker = self._markers[key]
        marker.pose = msg.pose
        marker.header = msg.header
        marker.header.frame_id = 'map'
        if self._markers[key + '_label']:
            label = self._markers[key + '_label']
            label.pose = copy.deepcopy(msg.pose)
            label.pose.position.z += marker.scale.z + label.scale.z * 0.5
            label.header.stamp = marker.header.stamp
            label.header.frame_id = 'map'

    def _updateParams(self):
        for key, param in self._sourceParams.iteritems():
            data = rospy.get_param(param, None)
            if data:
                self._markers[key].pose.position.x = data.get('position', {'x': self._markers[key].pose.position.x}).get('x')
                self._markers[key].pose.position.y = data.get('position', {'y': self._markers[key].pose.position.y}).get('y')
                self._markers[key].pose.position.z = data.get('position', {'z': self._markers[key].pose.position.z}).get('z')

                self._markers[key].pose.orientation = self._getOrientation(data.get('orientation', None))

                self._markers[key].scale.x = data.get('scale', {'x': self._markers[key].scale.x}).get('x')
                self._markers[key].scale.y = data.get('scale', {'y': self._markers[key].scale.y}).get('y')
                self._markers[key].scale.z = data.get('scale', {'z': self._markers[key].scale.z}).get('z')

                self._markers[key].color.r = data.get('color', {'r': self._markers[key].color.r}).get('r')
                self._markers[key].color.g = data.get('color', {'g': self._markers[key].color.g}).get('g')
                self._markers[key].color.b = data.get('color', {'b': self._markers[key].color.b}).get('b')
                self._markers[key].color.a = data.get('color', {'a': self._markers[key].color.a}).get('a')

    def run(self):
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            try:
                self._updateParams()
                self._publisher.publish(MarkerArray(self._markers.values()))
                rate.sleep()
            except rospy.ROSInterruptException:
                break


if __name__ == '__main__':
    rospy.init_node('location_publisher', anonymous=True)

    if len(sys.argv) == 1:
        # For debugging in the IDE
        import yaml
        with open('../config/locations.yaml') as config:
            configDict = yaml.load(config)
            rospy.set_param('~sources', configDict)

    try:
        sources = rospy.get_param('~sources')
        topic = rospy.get_param('~topic', 'locations')
        rate = rospy.get_param('~rate', 10)
    except KeyError as e:
        rospy.logfatal("iha_main Missing param: %s" % (e.message,))
        exit(0)

    a = LocationPublisher(rate,
                          topic,
                          sources)
    a.run()
