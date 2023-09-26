#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#  Copyright 2022 Fraunhofer FKIE - All Rights Reserved
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from typing import Dict
from typing import Set

import rospy
import tf2_ros
import threading

from std_msgs.msg import Int32
from fkie_measurement_msgs.msg import Measurement, MeasurementArray, MeasurementLocated, MeasurementValue


class MeasurementCollectorNode(rospy.SubscribeListener):
    def __init__(self):
        super(MeasurementCollectorNode, self).__init__()
        rospy.loginfo('Launch parameter:')
        self.param_topic_pub_measurement_array = rospy.get_param(
            '~topic_pub_measurement_array', 'measurement_array_agg')
        rospy.loginfo(
            f"  topic_pub_measurement_array: {self.param_topic_pub_measurement_array}")
        self.param_topic_sub_measurement_array = rospy.get_param(
            '~topic_sub_measurement_array', 'measurement_array')
        rospy.loginfo(
            f"  topic_sub_measurement_array: {self.param_topic_sub_measurement_array}")

        self.global_frame = rospy.get_param('~frame_global', "map")
        rospy.loginfo(f"  frame_global: {self.global_frame}")

        self.utm_zone_number = rospy.get_param('~utm_zone_number', "")
        rospy.loginfo(f"  utm_zone_number: {self.utm_zone_number}")
        self.utm_zone_letter = rospy.get_param('~utm_zone_letter', "")
        rospy.loginfo(f"  utm_zone_letter: {self.utm_zone_letter}")

        # unique_serial_id: MeasurementArray with full_history
        self.sensor_histories: Dict[str, MeasurementArray] = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # create subscribers for all registered sensors
        self.pub_measurement_array = rospy.Publisher(
            self.param_topic_pub_measurement_array, MeasurementArray, subscriber_listener=self, queue_size=5)
        rospy.loginfo(f"advertised to {self.pub_measurement_array.name}")
        self.sub_m = rospy.Subscriber(
            '/in_measurement', Measurement, self.callback_measurement, queue_size=5)
        rospy.loginfo(f"subscriberd to {self.sub_m.name}")
        self.sub_m_located = rospy.Subscriber(
            '/in_measurement_located', MeasurementLocated, self.callback_measurement_located, queue_size=5)
        rospy.loginfo(f"subscriberd to {self.sub_m_located.name}")
        self.sub_m_array = rospy.Subscriber(
            self.param_topic_sub_measurement_array, MeasurementArray, self.callback_measurement_array, queue_size=5)
        rospy.loginfo(f"subscriberd to {self.sub_m_array.name}")
        self.sub_client_count = rospy.Subscriber(
            '/client_count', Int32, self.callback_client_count, queue_size=5)
        rospy.loginfo(f"subscriberd to {self.sub_m_array.name}")

    def callback_measurement(self, msg):
        # type: (Measurement) -> None
        if not msg.unique_serial_id:
            rospy.logerr("[callback_measurement] empty [unique_serial_id].")
            return

        if msg.unique_serial_id not in self.sensor_histories:
            ma = MeasurementArray()
            ma.full_history = True
            self.sensor_histories[msg.unique_serial_id] = ma

        s_history: MeasurementArray = self.sensor_histories[msg.unique_serial_id]

        msgl = MeasurementLocated()
        msgl.measurement = msg

        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, msg.header.frame_id, rospy.Time())
            # get current position
            msgl.pose.pose.position.x = trans.transform.translation.x
            msgl.pose.pose.position.y = trans.transform.translation.y
            msgl.pose.pose.position.z = trans.transform.translation.z
            msgl.pose.header.frame_id = trans.header.frame_id
            msgl.pose.header.stamp = trans.header.stamp
            s_history.located_measurements.append(msgl)
            if self.utm_zone_number and self.utm_zone_letter:
                msgl.utm_zone_number = self.utm_zone_number
                msgl.utm_zone_letter = self.utm_zone_letter
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("[callback_measurement] Could not find TF2 lookup between frames [{0}] and [{1}]".format(
                self.global_frame, msg.header.frame_id
            ))
            return
        msga = MeasurementArray()
        msga.full_history = False
        msga.located_measurements.append(msgl)
        self.pub_measurement_array.publish(msga)

    def callback_measurement_located(self, msg):
        # type: (MeasurementLocated) -> None
        if not msg.measurement.unique_serial_id:
            rospy.logerr(
                "[callback_measurement_located] empty [unique_serial_id].")
            return

        if msg.measurement.unique_serial_id not in self.sensor_histories:
            ma = MeasurementArray()
            ma.full_history = True
            self.sensor_histories[msg.measurement.unique_serial_id] = ma

        s_history: MeasurementArray = self.sensor_histories[msg.measurement.unique_serial_id]
        s_history.located_measurements.append(msg)

        msga = MeasurementArray()
        msga.full_history = False
        msga.located_measurements.append(msg)
        self.pub_measurement_array.publish(msga)

    def callback_measurement_array(self, msg):
        # clear if this message has full history
        if msg.full_history:
            ids: Set[str] = set()
            for msmnt in msg.measurements:
                ids.add(msmnt.unique_serial_id)
            for msmnt in msg.located_measurements:
                ids.add(msmnt.measurement.unique_serial_id)
            for id in ids:
                if id in self.sensor_histories:
                    del self.sensor_histories[id]

        # add to history
        for msmnt in msg.measurements:
            self.callback_measurement(msmnt)

        for msmnt in msg.located_measurements:
            self.callback_measurement_located(msmnt)
        self.pub_measurement_array.publish(msg)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        rospy.loginfo(f"New subscription for {topic_name}")
        msg = MeasurementArray()
        msg.full_history = True
        for _id, ma in self.sensor_histories.items():
            msg.measurements.extend(ma.measurements)
            msg.located_measurements.extend(ma.located_measurements)
        self.pub_measurement_array.publish(msg)

    def callback_client_count(self, msg):
        if (msg.data > 0):
            threading.Timer(3., self.peer_subscribe, ('new_client_count', None, None)).start()

# Main function
if __name__ == '__main__':
    rospy.init_node('measurement_collector_node')
    mes = MeasurementCollectorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt as e:
        pass
