#!/usr/bin/env python3

import rospy
import numpy as np
import random
import struct
import subprocess

from pypozyx import *
from pypozyx.tools.device_list import *
from pypozyx.tools.discovery import *
from pypozyx.tools.version_check import *
from pozyx_ros.msg import DeviceRange as Range
from pozyx_ros.msg import DeviceRangeArray
from std_msgs.msg import Bool

POZYX_PORT = {0x6a46:'369839493436', 0x6a66:'36A4396F3436', 0x6a0c:'365E39493436', \
                0x6a57:'367A39533436', 0x6744:'325F34893037', 0x6a48:'368C395F3436', \
		        0x6a41:'366139913436', 0x6a78:'36A6396F3436', 0x6a60:'367439823436'}


class pozyx_node(object):
    def __init__(self):
        super(pozyx_node, self).__init__()

        self.all_ids = rospy.get_param(rospy.get_param("~id_param_name"))
        print('----------------------------------------')
        print("my id: ", self.all_ids)

        # port of UWB
        self.port_list = get_pozyx_ports()
        self.all_pozyx = {}
        if len(self.port_list) == 0:
            rospy.logfatal("no device attached")
            raise RuntimeError("No pozyx device attached")

        
        all_port = {}
        for port in self.port_list:
            st, output = subprocess.getstatusoutput("udevadm info -a "+port+" | grep serial")
            print('----------------------------------------')
            print(output)
            print('----------------------------------------')
            for tag_p in self.all_ids:
                if POZYX_PORT[self.all_ids[tag_p]] == output.split('"')[1]:
                    rospy.loginfo("Found one port at: "+port)
                    all_port[tag_p] = port
                    break
        
        # prepare pub msg
        self.pub_msg = {}

        for tag_p in all_port:
            try:
                rospy.loginfo("trying " + str(all_port[tag_p]))
                self.all_pozyx[tag_p] = PozyxSerial(all_port[tag_p])
                network_id = NetworkID()
                self.all_pozyx[tag_p].getNetworkId(network_id)
                rospy.loginfo("Double checking the network id.")
                if network_id == self.all_ids[tag_p]:
                    rospy.loginfo("found my port: 0x%0.4x" % self.all_ids[tag_p])
                    rospy.loginfo(tag_p + " is ready.")
                    self.pub_msg[tag_p] = DeviceRangeArray()
                    # break
                else:
                    rospy.logwarn("not my port")
                    self.all_pozyx[tag_p].ser.close()
                    raise RuntimeError("The id is wrong!!!!!!")
            except SerialException:
                rospy.logwarn("Got Serial Exception")

            if self.all_pozyx[tag_p] == None:
                rospy.logerr("No device found with certain ID")
                raise RuntimeError("No device found with certain ID")

            self.all_pozyx[tag_p].printDeviceInfo()
            self.all_pozyx[tag_p].setRangingProtocol(PozyxConstants.RANGE_PROTOCOL_PRECISION, None)

        self.device_list = None
        self.dest_device_list = rospy.get_param(rospy.get_param("~dest_ids_name"))
        print("self.device: ", self.dest_device_list)

        # Publisher
        self.pub_anchor = rospy.Publisher('/ranges', DeviceRangeArray, queue_size=5)

    def getDeviceRange(self):
        for device in self.dest_device_list:
            time_for_arr = rospy.Time.now()
            for tag_p in self.all_pozyx:
                self.pub_msg[tag_p] = DeviceRangeArray()

                device_range = DeviceRange()
                status = self.all_pozyx[tag_p].doRanging(self.dest_device_list[device], device_range, None)
                if status == POZYX_SUCCESS:
                    r = Range()

                    r.tag_id = self.dest_device_list[device]
                    r.header.stamp = rospy.Time.now()
                    r.distance = device_range.distance
                    r.RSS = device_range.RSS

                    rospy.loginfo("0x%0.4x\n --------------------\n Distance: %d\n RSSI: %d" %(self.dest_device_list[device], r.distance, r.RSS))
                    self.pub_msg[tag_p].rangeArray.append(r)
                else:
                    pass

                self.pub_msg[tag_p].header.stamp = time_for_arr

            if 'anchor' in self.pub_msg:
                self.pub_anchor.publish(self.pub_msg['anchor'])

    def printErrorCode(self):
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code)
        if status == POZYX_SUCCESS:
            rospy.logerr(self.pozyx.getErrorMessage(error_code))
        else:
            rospy.logerr("error getting error msg")

if __name__ == '__main__':
    rospy.init_node('radiation_demo',anonymous=False)
    freq = int(rospy.get_param("~ranging_freq"))

    timer = rospy.timer.Rate(freq)
    try:
        pozyx_node = pozyx_node()
        last_t = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            pozyx_node.getDeviceRange()
            # timer.sleep()
            this_t = rospy.Time.now().to_sec()
            if this_t - last_t > 1:
                rospy.loginfo("Still Alive")
                last_t = this_t
    except RuntimeError:
        rospy.logfatal("get runtime error")
    except rospy.ROSInterruptException:
        pass