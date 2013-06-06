#!/usr/bin/env python

""" Program that monitors a list of machines. If the monitor finds some abnormal
situation, it will notify by email.
 """

import argparse
import subprocess
import logging
import smtplib
import time
import socket


DEFAULT_EMAILS = ['drcsim-support@osrfoundation.org']
MAX_PING_TIME_MS = 1500
INTERNAL_LOG_FILE = '/tmp/vrc_monitor.log'
FALLBACK_LOG_FILE = '~/vrc_monitor.log'


class Monitor:
    def __init__(self, machines, email):
        self.machines = machines
        self.email = email

        self.init_internal_log('DEBUG')

        # State of the machines (True=Alive, False=Dead)
        self.states = [True for machine in machines]
        self.prev_states = list(self.states)

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

        self.logger.info('New vrc_monitor started with the next options:\n'
                         'Machines: %s\nEmail: %s\nLog level: %s'
                         % (self.machines, self.email, log_level))

    def is_machine_alive(self, host, npackages):
        """
        Send a ping command to a given host to determine if it is alive.

        @param host: host destination
        @type host: string
        @param npackages: number of packages sent on each measurement
        @type npackages: int
        """
        cmd = 'fping {host} -C {npacket} -q -t {maxping}'.format(host=host, npacket=npackages, maxping=MAX_PING_TIME_MS)
        try:
            subprocess.check_output(cmd.split(), stderr=subprocess.STDOUT)
            return True
        except subprocess.CalledProcessError:
            return False

    def mail(self, fromaddr, toaddr_list, subject, text):
        '''
        Send email using a gmail account.

        @param fromaddr: Remitent
        @param toaddr_list: destination email list
        @param subject: email's subject
        @param text: email's body message
        '''

        content = ('From: %s\r\nTo: %s\r\nSubject: %s\n\n%s'
                   % (fromaddr, ", ".join(toaddr_list), subject, text))

        server = smtplib.SMTP('localhost')
        server.set_debuglevel(1)
        server.sendmail(fromaddr, toaddr_list, content)
        server.quit()

    def notify(self, machine, value):
        '''
        Notify a machine's event (problem or recovery).
        @param machine Machine that triggered the alert
        @param value True (Back to live) or False (Not responding)
        '''
        try:
            machine_type = socket.gethostname().split('-')[0]
            constellation = socket.gethostname().split('-')[1]

            if machine_type == 'router':
                if value:
                    content = ('The machine %s of const. %s is now up and running'
                               % (machine, constellation))
                else:
                    content = ('The machine %s of const %s is not responding'
                               % (machine, constellation))

                subject = 'VRC_Monitor (constellation %s)' % constellation
                self.mail('VRC_Monitor', self.email, subject, content)

                self.logger.info('New change in the state of a machine detected: %s'
                                 ' - %s' % (machine, content))
        except Exception, excep:
                    self.logger.error('notify(): %s' % repr(excep))

    def monitor(self):
        '''
        Monitor the state of a set of machines. It notifies by email if a
        machine does not response to ping or it is recovered.
        '''
        while True:

            # Ping the machines
            print 'Monitoring machines'
            for i in range(len(self.machines)):
                try:
                    self.states[i] = self.is_machine_alive(self.machines[i], 1)
                except Exception, excep:
                    self.logger.error('monitor() Exception running ping: %s'
                                      % repr(excep))

            # Notify if something has changed
            print 'Looking for changes'
            for i in range(len(self.machines)):
                if self.states[i] != self.prev_states[i]:
                    self.notify(self.machines[i], self.states[i])

            time.sleep(60)

            self.prev_states = list(self.states)


if __name__ == '__main__':
    # Specify command line arguments
    parser = argparse.ArgumentParser(
        description=('Monitor machines and notify updates by email'))

    parser.add_argument('machines', metavar='MACHINES',
                        nargs='+', help='List of machines to be monitored')

    # Parse command line arguments
    args = parser.parse_args()

    vrc_monitor = Monitor(args.machines, DEFAULT_EMAILS)
    vrc_monitor.monitor()
