#!/usr/bin/env python

""" This script runs as a daemon and tries to maintain the current latency as
close to a target latency. The current and target latencies are available
in redis database. The script acts as a controller injecting an extra
amount of latency every cycle.
"""

import sys
import time
import redis
import subprocess
import argparse
#import daemon

from cloudsim import pid


class TS_Controller:
    """
    Class for adding extra latency to a network interface. The amount of
    latency will be different in each cycle depending on the target
    latency and current latency.
    """

    # Constants
    STATIC = 'static'
    DYNAMIC = 'dynamic'
    PID = 'pid'
    STEP = 10.0

    def __init__(self, freq, typec, device, current_latency_label,
                 target_latency_label, max_lat, verbose):
        """
        Constructor.

        @param freq: frequency of latency injections (Hz.)
        @type freq: float
        @param typec: type of controller to run the shaping
        @type typec: string
        @param device: device in which the shaping will be injected
        @type device: string
        @param current_latency_label: redis key label for the current latency
        @type current_latency_label: string
        @param target_latency_label: redis key label for the target latency
        @type target_latency_label: string
        @param max_lat: maximum injection value (ms.)
        @type max_lat: float
        @param verbose: show information of the controller status
        @type verbose: boolean
        """
        self.freq = freq
        self.typec = typec
        self.device = device
        self.current_lat_lab = current_latency_label
        self.target_lat_lab = target_latency_label
        self.max_lat = max_lat
        self.verbose = verbose

        self.database = redis.Redis('localhost')
        self.current = 0.0
        self.pid = pid.PID('TS', 0, 1, 0, 100)

        cmd = "vrc_init_tc.py {dev}".format(dev=self.device)
        try:
            subprocess.check_call(cmd.split())
        except subprocess.CalledProcessError as ex:
            print ex.output
            sys.exit(1)

    def update(self):
        """
        Function that periodically runs and calculate the new latency
        injection based on the target and current latency values.
        """
        has_target_key = self.database.exists(self.target_lat_lab)
        has_current_key = self.database.exists(self.current_lat_lab)

        if has_target_key and has_current_key:
            target_lat = float(self.database.get(self.target_lat_lab))
            current_lat = float(self.database.get(self.current_lat_lab))

            if self.typec == TS_Controller.STATIC:  # Type: Static increment/decrement
                if current_lat < target_lat:
                    self.current += TS_Controller.STEP
                else:
                    self.current -= TS_Controller.STEP
            elif self.typec == TS_Controller.DYNAMIC:  # Type: Dynamic increment/decrement
                self.current += target_lat - current_lat
            elif self.typec == TS_Controller.PID:  # Type: PID
                self.pid.setReference((target_lat - current_lat) / self.max_lat)
                self.current += (self.pid.getOutput() / 100.0) * self.max_lat
            else:
                print '[TS_Controller::update()] Wrong controller type (', str(self.typec, ')')
                return

            self.current = min(max(0, self.current), self.max_lat)

            cmd = "vrc_configure_tc.py {dev} {latency}ms {loss}%".format(dev=self.device, latency=self.current, loss=0)
            if self.verbose:
                print 'Command to run: ', cmd
                print 'Target: ', target_lat
                print 'Current: ', current_lat
                print 'To inject: ', self.current
            try:
                status = subprocess.check_call(cmd.split())
            except subprocess.CalledProcessError as ex:
                print ex.output
                print status, '[TS_Controller::update()] Error using tc'
                return
        else:
            if self.verbose:
                print '[TS_Controller::update()] Latency keys not available'


def run_daemon(freq, typec, device, current_latency_label,
               target_latency_label, max_lat, verbose):
    """
    Run the latency injection controller periodically at a given frequency.

    @param freq: frequency of latency injections (Hz.)
    @type freq: float
    @param typec: type of controller to run the shaping
    @type typec: string
    @param device: device in which the shaping will be injected
    @type device: string
    @param current_latency_label: redis key label for the current latency
    @type current_latency_label: string
    @param target_latency_label: redis key label for the target latency
    @type target_latency_label: string
    @param max_lat: maximum injection value (ms.)
    @type max_lat: float
    @param verbose: show information of the controller status
    @type verbose: boolean
    """
    #with daemon.DaemonContext(stdout=sys.stdout, stderr=sys.stdout):
    shaping = TS_Controller(freq, typec, device, current_latency_label,
                            target_latency_label, max_lat, verbose)
    if freq <= 0.0:
        print 'Negative frequency specified:', str(freq)
        sys.exit(1)

    period = 1.0 / freq
    while True:
        shaping.update()
        time.sleep(period)


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


if __name__ == "__main__":
    # Specify command line arguments
    parser = argparse.ArgumentParser(description='Injects extra latency into an interface')
    parser.add_argument('-f', '--frequency', metavar='FREQ',
                        type=check_negative, default=1.0,
                        help='frequency of measurements (Hz)')
    parser.add_argument('-t', '--type', metavar='CONTROLLER_TYPE',
                        choices=[TS_Controller.STATIC,
                        TS_Controller.DYNAMIC, TS_Controller.PID],
                        default=TS_Controller.DYNAMIC,
                        help='type of controller')
    parser.add_argument('-cl', '--current_latency_label', metavar='CURRENT_LATENCY_LABEL',
                        default='ts_currentLatency',
                        help='redis key associated to the current measurement')
    parser.add_argument('-tl', '--target_latency_label', metavar='TARGET_LATENCY_LABEL',
                        default='ts_targetLatency',
                        help='redis key associated to the target measurement')
    parser.add_argument('-m', '--max', metavar='LATENCY',
                        type=check_negative, default=500.0,
                        help='maximum injection value (ms.)')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='show information of the controller status')
    parser.add_argument('-d', '--device', metavar='DEVICE', required='True',
                        help='num of packages sent on every measurement')

    # Parse command line arguments
    args = parser.parse_args()
    arg_freq = args.frequency
    arg_type = args.type
    arg_device = args.device
    arg_current_latency_label = args.current_latency_label
    arg_target_latency_label = args.target_latency_label
    arg_max_latency = args.max
    arg_verbose = args.verbose

    run_daemon(arg_freq, arg_type, arg_device, arg_current_latency_label,
               arg_target_latency_label, arg_max_latency, arg_verbose)
