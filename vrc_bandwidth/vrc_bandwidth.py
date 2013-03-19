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
from multiprocessing import Lock

# If ROS is not installed && sourced, that will fail
try:
    import rospy
    from std_msgs.msg import String
except ImportError:
    print ('ROS is not installed or its environment is not ready')
    sys.exit(1)


class BandwidthCount:
    """
    Class for counting and logging bandwith usage
    """

    #  ROS topics to activate/deactivate the counting and log
    START = 'vrc/state/start'
    STOP = 'vrc/state/stop'
    TO_KB = 1.0 / 1000.0

    def __init__(self, freq, directory, prefix, mode):
        """
        Constructor.

        @param freq: frequency of counting and logging (Hz.)
        @type freq: float
        @param directory: directory where the log file will be contained
        @type directory: string
        @param prefix: prefix of the log filename
        @type prefix: string
        @param isincremental: True if incremental logging is selected
        @type isincremental: boolean
        """
        self.freq = freq
        self.dir = directory
        self.prefix = prefix
        self.mode = mode

        rospy.init_node('VRC_bandwidth', anonymous=True)

        # Default name of the file containing the log
        self.logfilename = self.prefix + '.log'

        # Restricts only one start_counting() at the same time
        self.mutex = Lock()
        self.running = False

        # Subscribe to the topics to start and stop the counting/logging
        rospy.Subscriber(BandwidthCount.START, String, self.start_counting)
        rospy.Subscriber(BandwidthCount.STOP, String, self.stop_counting)

        try:
            # Flush all chains
            cmd = 'sudo iptables -F'
            subprocess.check_call(cmd.split())
            # Delete all chains
            cmd = 'sudo iptables -X'
            subprocess.check_call(cmd.split())

            # Create iptables chains
            cmd = 'sudo iptables -N Inbound'
            subprocess.check_call(cmd.split())
            cmd = 'sudo iptables -N Outbound'
            subprocess.check_call(cmd.split())

            # Link with default traffic chains
            cmd = 'sudo iptables -I INPUT -j Inbound'
            subprocess.check_call(cmd.split())
            cmd = 'sudo iptables -I OUTPUT -j Outbound'
            subprocess.check_call(cmd.split())

            # Select traffic to measure [tcp|udp|all]
            cmd = 'sudo iptables -A Inbound -p all'
            subprocess.check_call(cmd.split())
            cmd = 'sudo iptables -A Outbound -p all'
            subprocess.check_call(cmd.split())
        except subprocess.CalledProcessError as ex:
            print ex.output
            print 'iptables initialization commands failed'
            sys.exit(1)

        rospy.spin()

    @staticmethod
    def reset_counting():
        """
        Reset bandwidth stats.
        """
        cmd = 'sudo iptables -Z'
        subprocess.check_call(cmd.split())

    @staticmethod
    def get_bandwidth_stats():
        """
        Returns the inbound and outbound packet size since the last reset.

        @raise subprocess.CalledProcessError: if the external commands
        (iptables) does not return 0
        """
        # Get inbound bandwidth (KBytes)
        cmd = 'sudo iptables -L Inbound -n -v -x'
        output = str(subprocess.check_output(cmd.split()))
        inbound = float(output.split('\n')[2].split()[1]) * BandwidthCount.TO_KB

        # Get outbound bandwidth (Kbytes)
        cmd = 'sudo iptables -L Outbound -n -v -x'
        output = str(subprocess.check_output(cmd.split()))
        outbound = float(output.split('\n')[2].split()[1]) * BandwidthCount.TO_KB

        return inbound, outbound

    def log_counting(self, inbound, outbound):
        """
        Log current bandwidth stats on disk.

        @param inbound: KBytes received since the last reset
        @type inbound: int
        @param outbound: KBytes sent since the last reset
        @type outbound: int
        """
        print self.fullpathname
        with open(self.fullpathname, 'a') as logf:
            tstamp = str(time.time())
            simclock = str(rospy.get_time())
            logf.write(tstamp + ' ' + simclock + ' ' + str(inbound) +
                       ' ' + str(outbound) + '\n')

    def update_counting(self, data):
        """
        Callback periodically called by ROS to update the counting/logging.

        @param data Not used but necessary to match the rospy.Timer signature
        """
        try:
            inbound, outbound = self.get_bandwidth_stats()
            self.log_counting(inbound, outbound)
        except subprocess.CalledProcessError as ex:
            print ex.output

    def start_counting(self, data):
        """
        Reset the bandwidth stats and starts counting/logging periodically.

        @param data Not used but needs to match the rospy.Subscriber signature
        """
        #rospy.loginfo('I heard the start signal')
        with self.mutex:
            if self.running:
                return
            else:
                try:
                    # Update filename
                    if self.mode == 'new':
                        timestamp = str(datetime.datetime.now())
                        self.logfilename = self.prefix + '-' + timestamp.replace(' ', '-') + '.log'
                    self.fullpathname = os.path.join(self.dir, self.logfilename)

                    # Create header unless mode is resume and file exists
                    if ((self.mode == 'replace') or
                       ((self.mode == 'resume') and (not os.path.exists(self.fullpathname))) or
                       (self.mode == 'new')):
                        with open(self.fullpathname, 'w') as logf:
                            header = '''
# This is a log of bandwidth usage,
# including time stamps and a summation of inbound and
# outbound kilobytes of bandwidth.\n
# Router_time /clock inKB outKB\n
'''
                            logf.write(header)

                    self.reset_counting()
                    period = rospy.Duration(1.0 / self.freq)
                    self.timer = rospy.Timer(period, self.update_counting)
                except subprocess.CalledProcessError as ex:
                    print ex.output
                    sys.exit(1)
                finally:
                    self.running = True

    def stop_counting(self, data):
        """
        Stop the bandwidth counting and logging.

        @param data Not used but needs to match the rospy.Subscriber signature
        """
        #rospy.loginfo('I heard the stop signal')
        with self.mutex:
            if not self.running:
                return
            else:
                self.timer.shutdown()
                self.running = False


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
    parser.add_argument('-m', '--mode', choices=['replace', 'resume', 'new'],
                        default='new', help='''
Defines the behavior after a new start/stop cycle. 'replace' overrides the log.
'resume' appends data to the current log. 'new' does not override log by creating
 a new file adding a timestamp suffix
'''
                        )

    # Parse command line arguments
    args = parser.parse_args()
    arg_freq = args.frequency
    arg_dir = args.dir
    if (not os.path.exists(arg_dir)):
        print 'Directory (', arg_dir, ') does not exists'
        sys.exit(1)
    arg_prefix = args.prefix
    arg_mode = args.mode

    # Run the node
    bandwidth_count = BandwidthCount(arg_freq, arg_dir, arg_prefix, arg_mode)
