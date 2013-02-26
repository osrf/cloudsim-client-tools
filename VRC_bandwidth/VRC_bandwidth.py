#!/usr/bin/env python

# Program that creates a ROS node and subscribes to the simulation state topics.
# When the 'start' topic arrives, the program starts counting the inbound and
# outbound bandwidth, as well as logging the data into a file. The counting and
# logging ends when the 'stop' topic is received.

#import roslib; roslib.load_manifest('VRC_bandwidth')
import rospy
from std_msgs.msg import String
import time
import datetime
import subprocess
import os

class BandwidthCount:

    def __init__(self, _freq):
        REPLACELOG = True
        self.freq = _freq
        rospy.init_node('VRC_bandwidth', anonymous=True)

        if REPLACELOG:
            self.logfile = 'bandwidth.log'
        else:
            filename = str(datetime.datetime.now())
            filename = 'bandwidth_' + filename.replace(' ', '-') + '.log'
            self.logfile = filename
  
        rospy.Subscriber("state/start", String, self.startCounting)
        rospy.Subscriber("state/stop", String, self.stopCounting)

        # delete all chains 
        self.runExternalCommand('sudo iptables -X')

        # create iptables chains
        self.runExternalCommand('sudo iptables -N Inbound')
        self.runExternalCommand('sudo iptables -N Outbound')

        # link with default traffic chains
        self.runExternalCommand('sudo iptables -I INPUT -j Inbound')
        self.runExternalCommand('sudo iptables -I OUTPUT -j Outbound')

        # select traffic to measure [tcp|udp|all]
        self.runExternalCommand('sudo iptables -A Inbound -p all')
        self.runExternalCommand('sudo iptables -A Outbound -p all')

        rospy.spin()

    def runExternalCommand(self, cmd):
        try:
            devnull = open('/dev/null', 'w')
            output = subprocess.check_output(cmd.split())
            return output
        except subprocess.CalledProcessError as e:
            print e.output
            return -1

    def getBandwidthStats(self):
        # Get inbound bandwidth
        cmd = 'sudo iptables -L Inbound -n -v -x'
        output = self.runExternalCommand(cmd)
        inbound = output.split('\n')[2].split()[1]

        # Get outbound bandwidth
        cmd = 'sudo iptables -L Outbound -n -v -x'
        output = self.runExternalCommand(cmd)
        outbound = output.split('\n')[2].split()[1]
        
        return inbound, outbound

    def resetCounting(self):
        self.runExternalCommand('sudo iptables -Z')

    def startCounting(self, data):
        self.resetCounting()
        # rospy.loginfo('I heard the start signal')
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.freq), self.updateCounting)

    def stopCounting(self, data):
        # rospy.loginfo('I heard the stop signal')
        self.timer.shutdown()

    def updateCounting(self, data):
        #print 'updateCounting()'
        inbound, outbound = self.getBandwidthStats()
        self.logCounting(inbound, outbound)

    def logCounting(self, _inbound, _outbound):
        #print 'logCounting()'
        f = open(self.logfile, 'a')
        #ToDo: timestamp from simulation time (subscribed?)
        timestamp = str(time.time())
        f.write(timestamp + ' ' + str(_inbound) + ' ' + str(_outbound) + '\n')
        f.close()

if __name__ == '__main__':
    bandwidthCount = BandwidthCount(1.0)

