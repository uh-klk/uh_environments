#! /usr/bin/env python

import os
import sys
import copy

from math import pi
from collections import namedtuple

import rospy

hydro = False
try:
    from PyKDL import Rotation
    hydro = True
except:
    from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
import visualization_msgs
from copy import deepcopy

baseDir = os.path.dirname(os.path.realpath(__file__))


class LocationPublisher(object):

    def __init__(self, rate, topic, sources):
        global hydro
        self._rate = rate
        if hydro:
            self._publisher = rospy.Publisher(topic, MarkerArray, queue_size=2)
        else:
            self._publisher = rospy.Publisher(topic, MarkerArray)
        self._sourceTopics = {}
        self._sourceParams = {}
        self._sourceFixed = {}
        self._markers = self._processSources(sources)

    def __del__(self):
        if self._publisher:
            self._publisher.unregister()

    def getMarker(self, source, id):
        mType = source.get('marker_type', Marker.SPHERE)
        mType = mType if not isinstance(mType, str) else getattr(Marker, mType.upper())
        mAction = source.get('marker_action', Marker.ADD)
        mAction = mAction if not isinstance(mType, str) else getattr(Marker, mType.upper())

        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = source.get('marker_frame', 'map')
        marker.ns = source.get('marker_ns', '')
        marker.id = int(source.get('marker_id', id))
        marker.type = mType
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

        # Second (default hidden) marker as a label
        label = copy.deepcopy(marker)
        label.action = Marker.DELETE
        label.ns += ('/' if label.ns else '') + 'labels'
        label.scale.z = 0.1
        label.pose.position.z += marker.scale.z + label.scale.z * 0.5
        label.type = Marker.TEXT_VIEW_FACING
        label.color.r = source.get('label_color', {}).get('r', 1.0)
        label.color.g = source.get('label_color', {}).get('g', 1.0)
        label.color.b = source.get('label_color', {}).get('b', 1.0)
        label.color.a = source.get('label_color', {}).get('a', 1.0)

        return marker, label

    def _processSources(self, sources):
        i = 0
        markers = {}
        for source in sources:
            i += 1
            marker, label = self.getMarker(source, i)
            if not isinstance(source, dict):
                rospy.logerr("Sources must be a dictionary list")
                return None

            sourceType = source.get('type', None)
            if not sourceType:
                rospy.logwarn("Source missing type param, skipping.  %s", source)
                continue

            if sourceType in ['topic', 'param']:
                data = source.get(sourceType, None)
                if not data:
                    rospy.logwarn("Source missing %s, skipping.  %s", source, sourceType)
                    continue

                key = '%s_%s' % (sourceType, data)
                if sourceType == 'topic':
                    self._sourceTopics[data] = rospy.Subscriber(data, PoseStamped, callback=self._updatePoseSource, callback_args=key)
                    markers[key] = marker
                    markers[key + '_label'] = label
                elif sourceType == 'param':
                    self._sourceParams[data] = (marker, label)
            elif sourceType == 'fixed':
                key = 'fixed_%s' % (i, )
                marker.action = label.action = Marker.ADD
                markers[key] = marker
                markers[key + '_label'] = label

        return markers

    def _getOrientation(self, data):
        q = Quaternion()
        if data:
            global hydro
            if hydro:
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
            else:
                if 'theta' in data:
                    # Assume Yaw Orientation
                    orientation = quaternion_from_euler(0, 0, data.get('theta'))
                elif any(k in ['roll', 'pitch', 'yaw'] for k in data.iterkeys()):
                    # Assume RPY Orientation
                    orientation = quaternion_from_euler(data.get('roll', 0), data.get('pitch', 0), data.get('yaw', 0))
                elif any(k in ['w', 'x', 'y', 'z'] for k in data.iterkeys()):
                    orientation = (data.get('x', 0),
                                   data.get('y', 0),
                                   dat.get('z', 0),
                                   data.get('w', 0))
                else:
                    orientation = (0, 0, 0, 0)

            q.x, q.y, q.z, q.w = orientation
        return q

    def _updatePoseSource(self, msg, key):
        marker = self._markers[key]
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.header = msg.header
        if marker.text and marker.type != Marker.TEXT_VIEW_FACING:
            label = self._markers[key + '_label']
            label.action = Marker.ADD
            label.pose = deepcopy(marker.pose)
            label.pose.position.z += marker.scale.z + label.scale.z * 0.5
            label.header.stamp = self._markers[key].header.stamp
        elif key + '_label' in self._markers:
            self._markers[key + '_label'].action = Marker.DELETE

    def _updateParams(self):
        global baseDir
        for param, (defaultMarker, defaultLabel) in self._sourceParams.iteritems():
            param_key = 'param_' + param
            data = rospy.get_param(param, [])
            data = data if isinstance(data, (list, tuple)) else (data, )
            for key in [k for k in self._markers.keys() if k.startswith(param_key)]:
                self._markers[key].action = Marker.DELETE

            for markerDict in data:
                key = param_key + '_' + str(markerDict['id'])
                marker = self._markers.get(key, copy.deepcopy(defaultMarker))
                marker.action = Marker.ADD
                marker.id = int(markerDict['id'])
                marker.pose.position.x = markerDict.get('position', {}).get('x', marker.pose.position.x)
                marker.pose.position.y = markerDict.get('position', {}).get('y', marker.pose.position.y)
                marker.pose.position.z = markerDict.get('position', {}).get('z', marker.pose.position.z)

                marker.pose.orientation = self._getOrientation(markerDict.get('orientation', None))

                marker.scale.x = markerDict.get('scale', {}).get('x', marker.scale.x)
                marker.scale.y = markerDict.get('scale', {}).get('y', marker.scale.y)
                marker.scale.z = markerDict.get('scale', {}).get('z', marker.scale.z)

                marker.color.r = markerDict.get('color', {}).get('r', marker.color.r)
                marker.color.g = markerDict.get('color', {}).get('g', marker.color.g)
                marker.color.b = markerDict.get('color', {}).get('b', marker.color.b)
                marker.color.a = markerDict.get('color', {}).get('a', marker.color.a)

                icon = markerDict.get('model', None)
                if icon and os.path.exists(os.path.join(os.path.join(baseDir, '../objects'), str(icon) + '.stl')):
                    marker.type = Marker.MESH_RESOURCE
                    marker.mesh_resource = 'package://uh_environments/objects/%s.stl' % (icon, )

                marker.text = markerDict.get('text', '')

                self._markers[key] = marker

                if marker.text and marker.type != Marker.TEXT_VIEW_FACING:
                    label = self._markers.get(key + '_label', deepcopy(defaultLabel))
                    label.id = marker.id
                    label.action = Marker.ADD
                    label.text = marker.text
                    label.pose = deepcopy(marker.pose)
                    label.pose.position.z += marker.scale.z + label.scale.z * 0.5
                    label.header.stamp = marker.header.stamp
                    self._markers[key + '_label'] = label

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
        with open(os.path.join(baseDir, '../config/sources.yaml')) as config:
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
