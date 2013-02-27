#!/usr/bin/env python

# Program that creates a ROS node and subscribes to the sim. state topics.
# When the 'start' topic arrives, the program starts counting the inbound and
# outbound bandwidth, as well as logging the data into a file. The counting and
# logging ends when the 'stop' topic is received.

import time
import datetime
import subprocess
import argparse
import os
import sys

#import roslib; roslib.load_manifest('VRC_bandwidth')
import rospy
from std_msgs.msg import String


class BandwidthCount:
    START = 'state/start'
    STOP = 'state/stop'

    def __init__(self, _freq, _dir, _prefix, _replace):
        self.freq = _freq
        self.dir = _dir
        self.prefix = _prefix
        self.replace = _replace

        rospy.init_node('VRC_bandwidth', anonymous=True)

        # Create the name of the file containing the log
        self.logFileName = self.prefix
        if not self.replace:
            timestamp = str(datetime.datetime.now())
            self.logFileName += '-' + timestamp.replace(' ', '-')
        self.logFileName += '.log'
        self.fullPathName = os.path.join(self.dir, self.logFileName)

        # Remove any previous log session
        if os.path.exists(self.fullPathName):
            os.remove(self.fullPathName)

        # Subscribe to the topics to start and stop the counting/logging
        rospy.Subscriber(BandwidthCount.START, String, self.startCounting)
        rospy.Subscriber(BandwidthCount.STOP, String, self.stopCounting)

        try:
            # flush all chains
            cmd = 'sudo iptables -F'
            subprocess.check_call(cmd.split())
            # delete all chains
            cmd = 'sudo iptables -X'
            subprocess.check_call(cmd.split())

            # create iptables chains
            cmd = 'sudo iptables -N Inbound'
            subprocess.check_call(cmd.split())
            cmd = 'sudo iptables -N Outbound'
            subprocess.check_call(cmd.split())

            # link with default traffic chains
            cmd = 'sudo iptables -I INPUT -j Inbound'
            subprocess.check_call(cmd.split())
            cmd = 'sudo iptables -I OUTPUT -j Outbound'
            subprocess.check_call(cmd.split())

            # select traffic to measure [tcp|udp|all]
            cmd = 'sudo iptables -A Inbound -p all'
            subprocess.check_call(cmd.split())
            cmd = 'sudo iptables -A Outbound -p all'
            subprocess.check_call(cmd.split())
        except subprocess.CalledProcessError as e:
            print e.output
            print 'iptables initialization commands failed'
            sys.exit(1)

        rospy.spin()

    def resetCounting(self):
        cmd = 'sudo iptables -Z'
        subprocess.check_call(cmd.split())

    def getBandwidthStats(self):
        # Get inbound bandwidth
        cmd = 'sudo iptables -L Inbound -n -v -x'
        output = subprocess.check_output(cmd.split())
        inbound = output.split('\n')[2].split()[1]

        # Get outbound bandwidth
        cmd = 'sudo iptables -L Outbound -n -v -x'
        output = subprocess.check_output(cmd.split())
        outbound = output.split('\n')[2].split()[1]

        return inbound, outbound

    def logCounting(self, _inbound, _outbound):
        with open(self.fullPathName, 'a') as f:
            #ToDo: timestamp from simulation time (subscribed?)
            timestamp = str(time.time())
            f.write(timestamp + ' ' + str(_inbound) + ' ' +
                    str(_outbound) + '\n')

    def updateCounting(self, data):
        try:
            inbound, outbound = self.getBandwidthStats()
            self.logCounting(inbound, outbound)
        except subprocess.CalledProcessError as e:
            print e.output

    def startCounting(self, data):
        #rospy.loginfo('I heard the start signal')
        try:
            self.resetCounting()
            period = rospy.Duration(1.0 / self.freq)
            self.timer = rospy.Timer(period, self.updateCounting)
        except subprocess.CalledProcessError as e:
            print e.output
            sys.exit(1)

    def stopCounting(self, data):
        #rospy.loginfo('I heard the stop signal')
        self.timer.shutdown()


def check_negative(value):
    fvalue = float(value)
    if fvalue <= 0:
        raise argparse.ArgumentTypeError("%s is not a positive float value"
                                         % value)
    return fvalue

if __name__ == '__main__':
    # Specify command line arguments
    parser = argparse.ArgumentParser(description='Counts/log bandwidth usage.')
    parser.add_argument('-f', '--frequency', metavar='FREQ',
                        type=check_negative, default=1,
                        help='frequency of counting and logging (Hz)')
    parser.add_argument('-d', '--dir', metavar='DESTINATION', default='.',
                        help='path to the log file')
    parser.add_argument('-p', '--prefix', metavar='FILENAME-PREFIX',
                        default='bandwidth', help='prefix of the logfile')
    parser.add_argument('-r', '--replace', action='store_true',
                        help='Override files. Do not add any suffix')
    args = parser.parse_args()

    # Parse command line arguments
    freq = args.frequency
    dir = args.dir
    if (not os.path.exists(dir)):
        print 'Directory (', dir, ') does not exists'
        sys.exit(1)
    prefix = args.prefix
    replace = args.replace

    # Run the node
    bandwidthCount = BandwidthCount(freq, dir, prefix, replace)
