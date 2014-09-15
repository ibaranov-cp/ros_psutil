#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Clearpath Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Clearpath Robotics nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import psutil
import rospy
import sys

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

# Import custom message data and dynamic reconfigure variables.
from ros_psutil.msg import Process
from ros_psutil.cfg import rosPsutilConfig as ConfigType

#print psutil.cpu_times()
#print psutil.pids()

class ROSPsutil():
    def __init__(self):
        # Get the ~private namespace parameters from command line or launch file.
        #init_message = rospy.get_param('~message', 'hello')
        rate = float(rospy.get_param('~rate', '1.0'))
        rospy.loginfo('rate = %d', rate)
        # Create a dynamic reconfigure server.
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)
        # Create a publisher for our custom message.
        pub = rospy.Publisher('Processes', Process, queue_size=1)
        # Set the message to publish as our custom message.
        msg = Process()

        # Main while loop.
        while not rospy.is_shutdown():
            # Fill in custom message variables with values from dynamic reconfigure server.
	    msg.pids = psutil.pids()

            for pids in msg.pids:
                p = psutil.Process(pids)
                msg.nice.append(p.nice())
                msg.num_threads.append(p.num_threads())
                msg.name.append(p.name())
                msg.exe.append(p.exe())
                msg.cwd.append(p.cwd())
                msg.status.append(p.status())
                msg.username.append( p.username())
                msg.terminal.append(p.terminal())
                msg.create_time.append(p.create_time())
                msg.memory_percent.append(p.memory_percent())
                msg.cpu_percent.append(p.cpu_percent())

            # Publish our custom message.
            pub.publish(msg)
            # Sleep for a while before publishing new messages. Division is so rate != period.
            if rate:
                rospy.sleep(1/rate)
            else:
                rospy.sleep(1.0)

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
        # Return the new variables.
        return config

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ros_psutil')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        new = ROSPsutil()
    except rospy.ROSInterruptException: pass
