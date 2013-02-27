#!/usr/bin/env python

""" Program that creates a ROS node and subscribes to the sim. state topics.
 When the 'start' topic arrives, the program starts counting the inbound and
 outbound bandwidth, as well as logging the data into a file. The counting and
 logging ends when the 'stop' topic is received.
 """

 # author: caguero

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
    """
    Class for counting and logging bandwith usage
    """

    #  ROS topics to activate/deactivate the counting and log
    START = 'state/start'
    STOP = 'state/stop'

    def __init__(self, freq, dir, prefix, isincremental):
        """
        Constructor.

        @param _freq: frequency of counting and logging (Hz.)
        @type _freq: float
        @param _dir: directory where the log file will be contained
        @type _dir: string
        @param _prefix: prefix of the log filename
        @type _prefix: string
        @param _isincremental: True if incremental logging is selected
        @type _isincremental: boolean
        """
        self.freq = freq
        self.dir = dir
        self.prefix = prefix
        self.isincremental = isincremental

        rospy.init_node('VRC_bandwidth', anonymous=True)

        # Create the name of the file containing the log
        self.logFileName = self.prefix
        if self.isincremental:
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
        """
        Reset bandwidth stats.
        """
        cmd = 'sudo iptables -Z'
        subprocess.check_call(cmd.split())

    def getBandwidthStats(self):
        """
        Returns the inbound and outbound packet size since the last reset.

        @raise subprocess.CalledProcessError: if the external commands
        (iptables) does not return 0
        """
        # Get inbound bandwidth
        cmd = 'sudo iptables -L Inbound -n -v -x'
        output = subprocess.check_output(cmd.split())
        inbound = output.split('\n')[2].split()[1]

        # Get outbound bandwidth
        cmd = 'sudo iptables -L Outbound -n -v -x'
        output = subprocess.check_output(cmd.split())
        outbound = output.split('\n')[2].split()[1]

        return inbound, outbound

    def logCounting(self, inbound, outbound):
        """
        Log current bandwidth stats on disk.

        @param inbound: bytes received since the last reset
        @type inbound: int
        @param outbound: bytes sent since the last reset
        @type outbound: int
        """
        with open(self.fullPathName, 'a') as f:
            #ToDo: timestamp from simulation time (subscribed?)
            timestamp = str(time.time())
            f.write(timestamp + ' ' + str(inbound) + ' ' +
                    str(outbound) + '\n')

    def updateCounting(self, data):
        """
        Callback periodically called by ROS to update the counting/logging.

        @param data Not used but necessary to match the rospy.Timer signature
        """
        try:
            inbound, outbound = self.getBandwidthStats()
            self.logCounting(inbound, outbound)
        except subprocess.CalledProcessError as e:
            print e.output

    def startCounting(self, data):
        """
        Reset the bandwidth stats and starts counting/logging periodically.

        @param data Not used but needs to match the rospy.Subscriber signature
        """
        #rospy.loginfo('I heard the start signal')
        try:
            self.resetCounting()
            period = rospy.Duration(1.0 / self.freq)
            self.timer = rospy.Timer(period, self.updateCounting)
        except subprocess.CalledProcessError as e:
            print e.output
            sys.exit(1)

    def stopCounting(self, data):
        """
        Stop the bandwidth counting and logging.

        @param data Not used but needs to match the rospy.Subscriber signature
        """
        #rospy.loginfo('I heard the stop signal')
        self.timer.shutdown()


def check_negative(value):
    """
    Checks if a parameter is a non negative float number.

    @param value: argument to verify
    """
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
    parser.add_argument('-i', '--incremental', action='store_true',
                        help='Do not override logs adding a timestamp suffix')
    args = parser.parse_args()

    # Parse command line arguments
    freq = args.frequency
    dir = args.dir
    if (not os.path.exists(dir)):
        print 'Directory (', dir, ') does not exists'
        sys.exit(1)
    prefix = args.prefix
    isincremental = args.incremental

    # Run the node
    bandwidthCount = BandwidthCount(freq, dir, prefix, isincremental)
