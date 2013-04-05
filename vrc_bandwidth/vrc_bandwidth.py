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
import redis
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

    def __init__(self, freq, directory, prefix, mode,
                 dev, fc_ip, uplink, downlink, topic_uplink, topic_downlink):
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
        print 'Starting a new bandwidth log'
        self.freq = freq
        self.dir = directory
        self.prefix = prefix
        self.mode = mode
        self.dev = dev
        self.fc_ip = fc_ip
        self.uplink = uplink
        self.downlink = downlink
        self.topic_uplink = topic_uplink
        self.topic_downlink = topic_downlink

        rospy.init_node('VRC_bandwidth', anonymous=True)

        # Default name of the file containing the log
        self.logfilename = self.prefix + '.log'

        # Restricts only one start_counting() at the same time
        self.mutex = Lock()
        self.running = False

        # Redis Database handler
        self.db = redis.Redis()

        # Flags to know if the links are active
        self.is_uplink_active = True
        self.is_downlink_active = True

        # Publishers for the remaining bits allowed
        self.pub_uplink = rospy.Publisher(self.topic_uplink, String)
        self.pub_downlink = rospy.Publisher(self.topic_downlink, String)

        # Subscribe to the topics to start and stop the counting/logging
        rospy.Subscriber(BandwidthCount.START, String, self.start_counting)
        rospy.Subscriber(BandwidthCount.STOP, String, self.stop_counting)

        rospy.spin()

    def reset_counting(self):
        """
        Reset bandwidth stats.
        """
        # Reset the accounting. That's also done by the vrc_accounter.
        self.db.set('VRC_BytesFromFC', 0)
        self.db.set('VRC_BytesToFC', 0)

        cmd = 'sudo stop vrc_bitcounter'
        try:
            subprocess.check_call(cmd.split())
            print 'vrc_bitcounter stopped'
        except:
            None
        cmd = 'sudo start vrc_bitcounter'
        subprocess.check_call(cmd.split())
        print 'vrc_bitcounter started'

    def get_bandwidth_stats(self):
        """
        Returns the inbound and outbound packet size since the last reset.

        @raise subprocess.CalledProcessError: if the external commands
        (iptables) does not return 0
        """

        inbound = self.db.get('VRC_BytesToFC')
        outbound = self.db.get('VRC_BytesFromFC')

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
            logf.write(tstamp + ' ' + simclock + ' ' + inbound +
                       ' ' + outbound + '\n')

    def check_limits(self, inbound, outbound):
        """
        Check if the allocated inbound/outbound bits have been reached the
        maximum limit allowed. In afirmative case, the associated link
        will be disabled for communication.

        @param inbound: current number of bits sent to the field computer
        @param outbound: current number of bits sent from the field computer
        """
        print 'inbound:', inbound
        print 'inbound limit:', self.uplink
        if long(inbound) > long(self.uplink) and self.is_uplink_active:
            print 'Uplink limit reached'
            cmd = 'sudo iptables -I FORWARD -i tun0 -o eth0 -j DROP'
            subprocess.check_call(cmd.split())
            self.is_uplink_active = False

        print 'outbound:', outbound
        print 'outbound limit:', self.downlink
        if long(outbound) > long(self.downlink) and self.is_downlink_active:
            print 'Downlink limit reached'
            cmd = 'sudo iptables -I FORWARD -i eth0 -o tun0 -j DROP'
            subprocess.check_call(cmd.split())
            self.is_downlink_active = False

    def publish_remaining_bits(self, inbound, outbound):
        remaining_uplink = String(str(long(self.uplink) - long(inbound)))
        remaining_downlink = String(str(long(self.downlink) - long(outbound)))

        self.pub_uplink.publish(remaining_uplink)
        self.pub_downlink.publish(remaining_downlink)

    def update_counting(self, data):
        """
        Callback periodically called by ROS to update the counting/logging.

        @param data Not used but necessary to match the rospy.Timer signature
        """
        try:
            inbound, outbound = self.get_bandwidth_stats()
            self.log_counting(inbound, outbound)
            self.check_limits(inbound, outbound)
            self.publish_remaining_bits(inbound, outbound)
        except subprocess.CalledProcessError as ex:
            print ex.output

    def start_counting(self, data):
        """
        Reset the bandwidth stats and starts counting/logging periodically.

        @param data Not used but needs to match the rospy.Subscriber signature
        """
        rospy.loginfo('I heard the start signal')
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
        rospy.loginfo('I heard the stop signal')
        with self.mutex:
            if not self.running:
                return
            else:
                # Reset the accounting. That's also done by the vrc_accounter.
                self.db.set('VRC_BytesFromFC', 0)
                self.db.set('VRC_BytesToFC', 0)

                # Resume communications
                try:
                    if not self.is_uplink_active:
                        cmd = 'sudo iptables -D FORWARD -i tun0 -o eth0 -j DROP'
                        subprocess.check_call(cmd.split())
                        self.is_uplink_active = True
                except:
                    None
                try:
                    if not self.is_downlink_active:
                        cmd = 'sudo iptables -D FORWARD -i eth0 -o tun0 -j DROP'
                        subprocess.check_call(cmd.split())
                        self.is_downlink_active = True
                except:
                    None

                # Stop the accouning
                cmd = 'sudo stop vrc_bitcounter'
                try:
                    subprocess.check_call(cmd.split())
                    print 'vrc_bitcounter stopped'
                except:
                    None

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
'resume' appends data to the current log. 'new' does not override log by
creating a new file adding a timestamp suffix
'''
                        )
    parser.add_argument('-tu', '--topic-uplink', metavar='ROSTOPIC',
                        default='vrc/bits/remaining/uplink',
                        help='Number of uplink bits remaining')
    parser.add_argument('-td', '--topic-downlink', metavar='ROSTOPIC',
                        default='vrc/bits/remaining/downlink',
                        help='Number of downlink bits remaining')
    parser.add_argument('dev', metavar='DEVICE',
                        help='Device to attach the bit accounter (ex. eth0)')
    parser.add_argument('fc_ip', metavar='FIELD-COMPUTER-IP',
                        help='IP Address of the field computer')
    parser.add_argument('uplink_bits_limit', metavar='UPLINK-BITS-LIMIT',
                        help='Number of Uplink bits alotted')
    parser.add_argument('downlink_bits_limit', metavar='DOWNLINK-BITS-LIMIT',
                        help='Number of Downlink bits alotted')

    # Parse command line arguments
    args = parser.parse_args()
    print args

    arg_freq = args.frequency
    arg_dir = args.dir
    if (not os.path.exists(arg_dir)):
        print 'Directory (', arg_dir, ') does not exists'
        sys.exit(1)
    arg_prefix = args.prefix
    arg_mode = args.mode
    arg_rostopic_uplink = args.topic_uplink
    arg_rostopic_downlink = args.topic_downlink
    arg_dev = args.dev
    args_fc_ip = args.fc_ip
    arg_uplink = args.uplink_bits_limit
    arg_downlink = args.downlink_bits_limit

    # Run the node
    bandwidth_count = BandwidthCount(arg_freq, arg_dir, arg_prefix,
                                     arg_mode, arg_dev, args_fc_ip,
                                     arg_uplink, arg_downlink,
                                     arg_rostopic_uplink, arg_rostopic_downlink)
