#!/usr/bin/env python

""" Program that creates a ROS node and subscribes to the sim. state topics.
 When the 'start' topic arrives, the program starts counting the inbound and
 outbound bytes, as well as logging the data into a file. The counting and
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
import logging
from multiprocessing import Lock

#  ROS/Redis default topics/keys
DEFAULT_START = 'vrc/state/start'
DEFAULT_STOP = 'vrc/state/stop'
DEFAULT_REMAINING_UPLINK = 'vrc/bytes/remaining/uplink'
DEFAULT_REMAINING_DOWNLINK = 'vrc/bytes/remaining/downlink'
DEFAULT_CURRENT_UPLINK = 'vrc/bytes/current/uplink'
DEFAULT_CURRENT_DOWNLINK = 'vrc/bytes/current/downlink'
DEFAULT_MAX_UPLINK = 'vrc/bytes/limit/uplink'
DEFAULT_MAX_DOWNLINK = 'vrc/bytes/limit/downlink'
DEFAULT_SCORE = '/vrc_score'
DEFAULT_ELAPSED_TIME = 'vrc/seconds/wt_elapsed'
DEFAULT_REMAINING_TIME = 'vrc/seconds/wt_remaining'

# Tasks
DEFAULT_TASK_MAX_TIME = 1800


# To check if netwatcher does a good job
INTERNAL_LOG_FILE = '/tmp/vrc_netwatcher.log'
FALLBACK_LOG_FILE = '~/vrc_netwatcher.log'

# Range of IP addresses to enable/disable
INITIAL_IP_RANGE = '10.0.0.51'
END_IP_RANGE = '10.0.0.53'

# Score file name
NETWORK_SCORE_FILE = 'network_score.log'


# If ROS is not installed && sourced, that will fail
try:
    import roslib; roslib.load_manifest('atlas_msgs')
    import rospy
    from std_msgs.msg import String, Empty, Int32
    from atlas_msgs.msg import VRCScore
except ImportError:
    logging.basicConfig(filename=INTERNAL_LOG_FILE, level=logging.INFO)
    logging.error("ROS is not installed or its environment is not ready")
    sys.exit(1)


def stop_vrc_program():
    """
    Stop the network accounting program.

    @raises subprocess.CalledProcessError If vrc_bytecounter is not running
    """
    cmd = 'sudo stop vrc_bytecounter'
    subprocess.check_call(cmd.split())


class Netwatcher:
    """
    Class for accounting and logging network usage.
    """

    def __init__(self, freq, directory, prefix, mode, score_file, topic_start,
                 topic_stop, topic_uplink, topic_downlink, topic_score,
                 topic_wt_elapsed, topic_wt_remaining, max_task_time, max_uplink_key,
                 max_downlink_key, current_uplink_key, current_downlink_key,
                 outage, log_level):
        """
        Constructor.

        @param freq: frequency of counting and logging (Hz.)
        @param directory: directory where the log file will be contained
        @param prefix: prefix of the log filename
        @param mode: log behavior after a new start/stop (replace, resume, new)'
        @param score_file: Score file name
        @param topic_start: ROS topic that starts a new log session
        @param topic_stop: ROS topic that stops the current log session
        @param topic_uplink: ROS topic to publish remaining uplink bytes
        @param topic_downlink: ROS topic to publish remaining downlink bytes
        @param topic_score: ROS topic where all the scoring data is published
        @param topic_wt_elapsed: ROS topic showing the elapsed task wall time
        @param topic_wt_remaining: ROS topic showing the remaining task wall time
        @param max_task_time: Max. number of seconds to finish the task
        @param max_uplink_key: DB key that stores the maximum uplink bytes
        @param max_downlink_key: DB key that stores the maximum downlink bytes
        @param current_uplink_key: DB key that stores the current uplink bytes
        @param current_downlink_key: DB key that stores the current downl. bytes
        @param outage: Never deactivate the comms if True
        @param log_level: Level of severity of the events to track
        """
        self.freq = freq
        self.dir = directory
        self.prefix = prefix
        self.mode = mode
        self.score_file = score_file
        self.topic_start = topic_start
        self.topic_stop = topic_stop
        self.topic_uplink = topic_uplink
        self.topic_downlink = topic_downlink
        self.topic_score = topic_score
        self.wt_elapsed = topic_wt_elapsed
        self.wt_remaining = topic_wt_remaining
        self.max_task_time = max_task_time
        self.max_uplink_key = max_uplink_key
        self.max_downlink_key = max_downlink_key
        self.current_uplink_key = current_uplink_key
        self.current_downlink_key = current_downlink_key
        self.outage = outage

        # Wall clockwatch
        self.start_time = rospy.Time.from_sec(time.time())

        self.inbound = 0
        self.outbound = 0

        # iptables rules
        self.UPLINK_IPTABLES_DROP = ('sudo iptables -A FORWARD -m iprange '
                                     '--dst-range ' + INITIAL_IP_RANGE + '-' +
                                     END_IP_RANGE + ' -j DROP')
        self.DOWNLINK_IPTABLES_DROP = ('sudo iptables -A FORWARD -m iprange '
                                       '--src-range ' + INITIAL_IP_RANGE + '-' +
                                       END_IP_RANGE + ' -j DROP')
        self.UPLINK_IPTABLES_REMOVE = ('sudo iptables -D FORWARD -m iprange '
                                       '--dst-range ' + INITIAL_IP_RANGE + '-' +
                                       END_IP_RANGE + ' -j DROP')
        self.DOWNLINK_IPTABLES_REMOVE = ('sudo iptables -D FORWARD -m iprange '
                                         '--src-range ' + INITIAL_IP_RANGE + '-' +
                                         END_IP_RANGE + ' -j DROP')

        rospy.init_node('VRC_netwatcher', anonymous=True)

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

        # Publishers
        self.pub_uplink = rospy.Publisher(self.topic_uplink, String)
        self.pub_downlink = rospy.Publisher(self.topic_downlink, String)
        self.pub_elapsed = rospy.Publisher(topic_wt_elapsed, Int32)
        self.pub_remaining = rospy.Publisher(topic_wt_remaining, Int32)

        # Subscribe to the topics to start and stop the counting/logging
        rospy.Subscriber(self.topic_score, VRCScore, self.start_counting)

        # Initialize internal log file
        self.init_internal_log(log_level)

        # Publish ROS topics every second
        self.pub_timer = rospy.Timer(rospy.Duration(1.0), self.publish_ros_topics)

        rospy.spin()

    def init_internal_log(self, log_level):
        """
        Initialize the format and filename of the internal log.

        @param log_level: Level of severity of the events to track
        """
        self.logger = logging.getLogger('internal_log')
        try:
            fh = logging.FileHandler(INTERNAL_LOG_FILE)
        except Exception:
            # If the file cannot be open, create the log on your $HOME
            fh = logging.FileHandler(FALLBACK_LOG_FILE)
        formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s',
                                      datefmt='%m/%d/%Y %I:%M:%S %p')
        fh.setFormatter(formatter)
        self.logger.addHandler(fh)
        self.logger.setLevel(log_level)

        self.logger.debug('New vrc_netwatcher started with the next options:\n'
                          'Logging\n'
                          '\tFrequency: %s Hz.\n'
                          '\tDestination directory: %s\n'
                          '\tLog file prefix: %s\n'
                          '\tLog mode: %s\n'
                          'ROS\n'
                          '\tROS start topic: %s\n'
                          '\tROS stop topic: %s\n'
                          '\tROS remaining uplink topic: %s\n'
                          '\tROS remaining downlink topic: %s\n'
                          'Accounting\n'
                          '\tRedis key for the max uplink bytes allowed: %s\n'
                          '\tRedis key for the max downlink bytes allowed: %s\n'
                          '\tRedis key for the current uplink bytes: %s\n'
                          '\tRedis key for the current downlink bytes: %s\n'
                          'Comms outage\n'
                          '\tCommunications outage activated? %s\n'
                          'Internal log\n'
                          '\tLog level: %s\n'
                          % (self.freq, self.dir, self.prefix, self.mode,
                             self.topic_start, self.topic_stop,
                             self.topic_uplink, self.topic_downlink,
                             self.max_uplink_key, self.max_downlink_key,
                             self.current_uplink_key, self.current_downlink_key,
                             self.outage, log_level))

    def reset_counting(self):
        """
        Reset accounting stats.

        @raise subprocess.CalledProcessError If vrc_bytecounter cannot start
        """
        # Reset the accounting. That's also done by the vrc_accounter.
        self.db.set(self.current_uplink_key, 0)
        self.db.set(self.current_downlink_key, 0)

        # vrc_bytecounter should be stopped but why not to be sure
        try:
            stop_vrc_program()
            self.logger.warn('Reset_counting() vrc_bytecounter stopped.'
                             'It should not be running')
        except Exception, excep:
            # Normal situation, cause vrc_bytecounter should not be running
            None

        cmd = 'sudo start vrc_bytecounter'
        try:
            subprocess.check_call(cmd.split())
            self.logger.debug('Reset_counting() vrc_bytecounter started')
        except Exception, excep:
            self.logger.error('Reset_counting() Unable to start vrc_bytecounter'
                              ': %s' % (excep))
            raise

    def check_rediskey_long(self, key, value, inside_function):
        """
        Checks that a (key, value) from redis exists and it's long.
        @param key: A redis key
        @param value: A redis value associated to 'key'
        @param inside_function: The function in which the value was read.

        @raise TypeError If the value is not a long
        """
        try:
            long(value)
        except TypeError:
            self.logger.error(inside_function + ' Current ' + key +
                              ' key does not exist or is not a long value (%s)'
                              % (value))
            raise

    def get_network_stats(self, key):
        """
        Returns the current network value looking at redis

        @raise TypeError If db[key] is not a long
        """
        value = self.db.get(key)
        self.check_rediskey_long(key, value, 'get_network_stats()')
        return value

    def log_counting(self):
        """
        Log current network stats on disk.
        """
        with open(self.fullpathname, 'a') as logf:
            tstamp = str(time.time())
            simclock = str(rospy.get_time())
            new_line = (tstamp + ' ' + simclock + ' ' + self.inbound +
                        ' ' + self.outbound + '\n')
            logf.write(new_line)
            logf.flush()
            self.logger.debug('New line written in %s:\n\t%s'
                              % (self.fullpathname, new_line))

    def is_limit_reached(self, label, current, max_key, is_active, outage_rule):
        """
        Check if the allocated inbound/outbound bytes have been reached the
        maximum limit allowed. In afirmative case, the associated link
        will be disabled for communication.

        @param label: Name of the comms link
        @param current: Current number of bytes
        @param max_key: Redis key for the max number of bytes allowed to the link
        @param is_active: True if the current link is active
        @param outage_rule: Command to deactivate the link

        @raise TypeError If db[max_key] is not a long
        """
        # Get the maximum bytes allowed
        max_link = self.db.get(max_key)
        try:
            self.check_rediskey_long(max_key, max_link, 'is_limit_reached()')
        except Exception, excep:
            max_link = sys.maxint

        self.logger.debug('is_limit_reached():\n'
                          '\t%s: %s / %s\n' % (label, current, max_link))

        if is_active and long(current) > long(max_link):
            if self.outage:
                try:
                    cmd = outage_rule
                    subprocess.check_call(cmd.split())
                    self.logger.info('%s disabled' % (label))
                except Exception, excep:
                    self.logger.error('Error disabling %s comms:\n\t%s'
                                      % (label, excep))
            else:
                self.logger.info('%s would be disabled (no outage mode)'
                                 % (label))
            return True
        return False

    def publish_remaining_bytes(self, current, max_key, publisher, topic):
        """
        Publish as a ROS topic the remaining bytes of a link.

        @param current: current number of bytes of a specific link
        @param max_key: redis key containing the max. allowed bytes for this link
        @param publisher: ROS publisher associated to this link
        @param topic: ROS topic associated to this link

        @raise TypeError If db[max_key] is not long
        """
        # Get the maximum bytes allowed
        max_link = self.db.get(max_key)
        try:
            self.check_rediskey_long(max_key, max_link, 'publish_remaining_bytes()')
        except Exception, excep:
            max_link = sys.maxint

        remaining = max(0, long(max_link) - long(current))

        publisher.publish(String(str(remaining)))
        self.logger.info('New ROS messages published:\n'
                         '\t%s: %s\n' % (topic, remaining))

    def publish_ros_topics(self, data):
        with self.mutex:
            # Remaining bytes
            self.publish_remaining_bytes(self.inbound, self.max_uplink_key,
                                         self.pub_uplink, self.topic_uplink)
            self.publish_remaining_bytes(self.outbound, self.max_downlink_key,
                                         self.pub_downlink, self.topic_downlink)

            # Time elapsed since the beginning og the task
            now = rospy.Time.from_sec(time.time())
            elapsed = now - self.start_time
            self.pub_elapsed.publish(elapsed.secs)

            # Time remaining since the beginning og the task
            remaining = self.max_task_time - elapsed.secs
            self.pub_remaining.publish(remaining)

    def update_counting(self, data):
        """
        Callback periodically called by ROS to update the counting/logging.

        @param data Not used but necessary to match the rospy.Timer signature
        """
        try:
            with self.mutex:
                self.inbound = self.get_network_stats(self.current_uplink_key)
                self.outbound = self.get_network_stats(self.current_downlink_key)

                # Save data into the usage log
                self.log_counting()

                # Activate the comms outage if the limits are reached
                if self.is_limit_reached('uplink', self.inbound,
                                         self.max_uplink_key,
                                         self.is_uplink_active,
                                         self.UPLINK_IPTABLES_DROP):
                    self.is_uplink_active = False

                if self.is_limit_reached('downlink', self.outbound,
                                         self.max_downlink_key,
                                         self.is_downlink_active,
                                         self.DOWNLINK_IPTABLES_DROP):
                    self.is_downlink_active = False

                '''# Publish comms stats as ROS topics
                self.publish_remaining_bytes(self.inbound, self.max_uplink_key,
                                             self.pub_uplink,
                                             self.topic_uplink)
                self.publish_remaining_bytes(self.outbound,
                                             self.max_downlink_key,
                                             self.pub_downlink,
                                             self.topic_downlink)'''

        except Exception, excep:
            self.logger.error('%s(): Exception captured:\n\t%s'
                              % ('update_counting()', excep))

    def start_counting_wrapper(self, data):
        """
        Create a non zero fake Time argument and calls the start_counting()

        @param data Not used but needs to match the rospy.Subscriber signature
        """
        fake_data = VRCScore()
        fake_data.sim_time_elapsed = rospy.Time(1)
        self.start_counting(fake_data)

    def start_counting(self, data):
        """
        Reset the network usage stats and starts counting/logging periodically.

        @param data VRCScore type with the current score
        """
        if data.sim_time_elapsed > rospy.Time(0):

            self.logger.info('I heard the start signal')
            with self.mutex:
                if not self.running:
                    # Update filename
                    if self.mode == 'new':
                        timestamp = str(datetime.datetime.now())
                        self.logfilename = (self.prefix + '-' +
                                            timestamp.replace(' ', '-') +
                                            '.log')
                    self.fullpathname = os.path.join(self.dir, self.logfilename)

                    # Create header unless mode is resume and file exists
                    if ((self.mode == 'replace') or
                       ((self.mode == 'resume') and
                        (not os.path.exists(self.fullpathname))) or
                       (self.mode == 'new')):
                        with open(self.fullpathname, 'w') as logf:
                            header = ('# This is a log of network usage,\n'
                                      '# including time stamps and a\n'
                                      '# summation of inbound and\n'
                                      '# outbound bytes of usage.\n'
                                      '# Router_time /clock inBytes outBytes\n')
                            logf.write(header)
                    try:
                        self.reset_counting()
                    except Exception, excep:
                        self.logger.error('%s(): Exception captured:\n\t%s\n'
                                          'Exit because reset_counting() failed'
                                          % (self.start_counting.__name__, excep))
                        sys.exit(1)
                    period = rospy.Duration(1.0 / self.freq)
                    self.timer = rospy.Timer(period, self.update_counting)
                    self.running = True

    def resume_comms(self, label, is_active, outage_restore_cmd):
        """
        Resume communications of a given link.

        @param label Name of the link to restore
        @param is_active True if the given link is active
        @param outage_restore_cmd Command line that restores the comms
        """
        try:
            if not is_active:
                if self.outage:
                    cmd = outage_restore_cmd
                    subprocess.check_call(cmd.split())
                else:
                    self.logger.info('%s enabled (no outage mode)' % (label))

        except Exception, excep:
            self.logger.error('%s(): Exception captured:\n\t%s\n'
                              'Probably the iptables rule did not exist:\n %s'
                              % ('resume_comms()', excep, outage_restore_cmd))

    def stop_counting(self, data):
        """
        Stop the current counting and logging session.

        @param data Not used but needs to match the rospy.Subscriber signature
        """
        self.logger.info('I heard the stop signal')
        with self.mutex:
            if self.running:
                self.logger.debug('stop_counting() Teardown the session')

                # Resume communications
                self.resume_comms('uplink', self.is_uplink_active,
                                  self.UPLINK_IPTABLES_REMOVE)
                self.is_uplink_active = True

                self.resume_comms('downlink', self.is_downlink_active,
                                  self.DOWNLINK_IPTABLES_REMOVE)
                self.is_downlink_active = True

                # Stop the accounting
                try:
                    stop_vrc_program()
                    self.logger.info('vrc_bytecounter stopped')
                except Exception, excep:
                    self.logger.warn('%s() Unable to stop vrc_bytecounter: '
                                     '%s' % ('stop_counting()', excep))

                # Do not call anymore to update_counting()
                self.timer.shutdown()

                # Write the networking score into a file
                self.create_score_file()

                self.running = False

    def create_score_file(self):
        """
        Write a single line text file with the network score.
        Format: #inBytes #outBytes
        """
        # Write the score file
        with open(self.score_file, 'w') as scoref:
            scoref.write(self.inbound + ' ' + self.outbound)


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
    parser = argparse.ArgumentParser(
        description=('Counts/log network usage. It uses the vrc_bytecounter  '
                     'tool to periodically account the uplink and downlink '
                     'bytes. The maximum uplink/downlink limit is stored in a '
                     'redis database. The current uplink/downlink values are '
                     'saved in redis by vrc_bytecounter. If a limit is reached,'
                     ' the appropriate link is deactivated. The remaining bytes'
                     ' for each link are pubished using ROS messages.'))

    # Logging options
    parser.add_argument('-f', '--frequency', metavar='FREQ',
                        type=check_negative, default=1,
                        help='frequency of counting and logging (Hz)')
    parser.add_argument('-d', '--dir', metavar='DESTINATION', default='.',
                        help='path to the log file')
    parser.add_argument('-p', '--prefix', metavar='FILENAME-PREFIX',
                        default='network-usage', help='prefix of the logfile')
    parser.add_argument('-m', '--mode', choices=['replace', 'resume', 'new'],
                        default='new',
                        help=('Defines the behavior after a new start/stop '
                              'cycle. "replace" overrides the log. "resume" '
                              'appends data to the current log. "new" does not '
                              'override log, it creates a new file adding a '
                              'timestamp suffix'))
    parser.add_argument('-s', '--score-file', metavar='SCORE-FILE',
                        default=NETWORK_SCORE_FILE, help='Score file name')

    # ROS options
    parser.add_argument('-start', '--topic-start', metavar='ROSTOPIC',
                        default=DEFAULT_START,
                        help='ROS topic to start a new log session')
    parser.add_argument('-stop', '--topic-stop', metavar='ROSTOPIC',
                        default=DEFAULT_STOP,
                        help='ROS topic to stop the current log session')
    parser.add_argument('-tu', '--topic-uplink', metavar='ROSTOPIC',
                        default=DEFAULT_REMAINING_UPLINK,
                        help='ROS topic to publish remaining uplink bytes')
    parser.add_argument('-td', '--topic-downlink', metavar='ROSTOPIC',
                        default=DEFAULT_REMAINING_DOWNLINK,
                        help='ROS topic to publish remaining downlink bytes')
    parser.add_argument('-ts', '--topic-score', metavar='ROSTOPIC',
                        default=DEFAULT_SCORE,
                        help='ROS topic where the scoring data is published')
    parser.add_argument('-te', '--topic-wt-elapsed', metavar='ROSTOPIC',
                        default=DEFAULT_ELAPSED_TIME,
                        help='ROS topic showing the elapsed task wall time')
    parser.add_argument('-tr', '--topic-wt-remaining', metavar='ROSTOPIC',
                        default=DEFAULT_REMAINING_TIME,
                        help='ROS topic showing the remaining task wall time')

    # Tasks
    parser.add_argument('-tt', '--max-task-time', metavar='SECONDS',
                        default=DEFAULT_TASK_MAX_TIME,
                        help='Max numbers of seconds to finish the task')

    # Byte accounting options
    parser.add_argument('-kmu', '--max-uplink-key',
                        metavar='MAXIMUM-UPLINK-REDIS-KEY',
                        default=DEFAULT_MAX_UPLINK,
                        help='Redis key that stores the maximum uplink bytes')
    parser.add_argument('-kmd', '--max-downlink-key',
                        default='vrc/bytes/limit/downlink',
                        metavar=DEFAULT_MAX_DOWNLINK,
                        help='Redis key that stores the maximum downlink bytes')
    parser.add_argument('-kcu', '--current-uplink-key',
                        metavar='CURRENT-UPLINK-REDIS-KEY',
                        default=DEFAULT_CURRENT_UPLINK,
                        help='Redis key that stores the current uplink bytes')
    parser.add_argument('-kcd', '--current-downlink-key',
                        metavar='CURRENT-DOWNLINK-REDIS-KEY',
                        default=DEFAULT_CURRENT_DOWNLINK,
                        help='Redis key that stores the current downlink bytes')
    parser.add_argument('-o', '--outage', action='store_true',
                        default=False,
                        help='Deactivate the comms if the limits are reached')

    # Log level
    parser.add_argument('--log', metavar='LOGTYPE',
                        choices=['INFO', 'DEBUG', 'WARNING',
                                 'ERROR', 'CRITICAL'],
                        default='INFO',
                        help=('Level of severity of the events to track '
                              '(INFO, DEBUG, WARNING, ERROR, CRITICAL)'))

    # Parse command line arguments
    args = parser.parse_args()

    arg_freq = args.frequency
    arg_dir = args.dir
    if (not os.path.exists(arg_dir)):
        print 'Directory (', arg_dir, ') does not exist'
        sys.exit(1)
    arg_prefix = args.prefix
    arg_mode = args.mode
    arg_score_file = args.score_file
    arg_rostopic_start = args.topic_start
    arg_rostopic_stop = args.topic_stop
    arg_rostopic_uplink = args.topic_uplink
    arg_rostopic_downlink = args.topic_downlink
    arg_rostopic_score = args.topic_score
    arg_rostopic_wt_elapsed = args.topic_wt_elapsed
    arg_rostopic_wt_remaining = args.topic_wt_remaining
    arg_max_task_time = args.max_task_time
    arg_max_uplink_key = args.max_uplink_key
    arg_max_downlink_key = args.max_downlink_key
    arg_current_uplink_key = args.current_uplink_key
    arg_current_downlink_key = args.current_downlink_key
    arg_outage = args.outage
    arg_log = args.log

    # Run the netwatcher node
    netwatcher = Netwatcher(arg_freq, arg_dir, arg_prefix, arg_mode,
                            arg_score_file,
                            arg_rostopic_start, arg_rostopic_stop,
                            arg_rostopic_uplink, arg_rostopic_downlink,
                            arg_rostopic_score, arg_rostopic_wt_elapsed,
                            arg_rostopic_wt_remaining, arg_max_task_time,
                            arg_max_uplink_key, arg_max_downlink_key,
                            arg_current_uplink_key, arg_current_downlink_key,
                            arg_outage, arg_log)
