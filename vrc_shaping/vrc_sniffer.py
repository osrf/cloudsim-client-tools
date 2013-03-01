#!/usr/bin/env python

""" This script runs as a daemon and measures the latency to reach a host.
Every second, this value is updated and the result is saved in a redis
database with a specific key.
"""

import sys
import time
import redis
import subprocess
import argparse
import unittest

UNREACHABLE = 99999


def get_ping_time(host, npackages):
    """
    Get latency time to a given host.

    @param host: host destination
    @type host: string
    @param npackages: number of packages sent on each measurement
    @type npackages: int
    """
    cmd = 'fping {host} -C {npacket} -q'.format(host=host, npacket=npackages)
    try:
        output = str(subprocess.check_output(cmd.split(),
                                             stderr=subprocess.STDOUT))
        print 'Output: ', output
    except subprocess.CalledProcessError as ex:
        print ex.output
        return UNREACHABLE

    # Calculate the mean of all the latencies measured for the destination host
    latencies = [float(latency) for latency in output.strip().split(':')[-1].split() if latency != '-']

    if len(latencies) > 0:
        return sum(latencies) / len(latencies)
    return UNREACHABLE


def run_daemon(freq, host, npackages, redis_label):
    """
    Run the latency sniffer periodically at a given frequency
    @param freq: frequency of latency measurements (Hz.)
    @type freq: float
    @param host: target host
    @type host: string
    @param npackages: number of packages sent on each measurement
    @type npackages: int
    @param redis_label: label to be used as key in the redis database
    @type redis_label: string
    """
    if freq <= 0:
        print 'Negative frequency specified:', str(freq)
        sys.exit(1)

    database = redis.Redis('localhost')
    period = 1.0 / freq

    while True:
        start_time = time.time()
        current_latency = get_ping_time(host, npackages)
        print 'Latency: ', str(current_latency)
        if current_latency >= 0:
            database.set(redis_label, current_latency)
        else:
            database.set(redis_label, UNREACHABLE)

        elapsed_time = time.time() - start_time
        time.sleep(max(period - elapsed_time, 0))


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


class TestSnifferFunctions(unittest.TestCase):

    def setUp(self):
        self.reachable_hosts = ['localhost']
        self.unreachable_hosts = ['nonexistenhost']

    def test_get_ping_time(self):
        for host in self.reachable_hosts:
            latency = get_ping_time(host, 1)
            self.assertTrue(latency > 0 and latency != UNREACHABLE)

        for host in self.unreachable_hosts:
            latency = get_ping_time(host, 1)
            self.assertTrue(latency == UNREACHABLE)


if __name__ == "__main__":

    # Specify command line arguments
    parser = argparse.ArgumentParser(description='Measures latency to a host.')
    parser.add_argument('-f', '--frequency', metavar='FREQ',
                        type=check_negative, default=1,
                        help='frequency of measurements (Hz)')
    parser.add_argument('-n', '--npackages', metavar='NUM',
                        type=int,
                        default=1,
                        help='num of packages sent on every measurement')
    parser.add_argument('-l', '--label', metavar='LABEL',
                        default='ts_currentLatency',
                        help='redis key associated to the measurement')
    parser.add_argument('-t', '--targethost', metavar='HOST', required='True',
                        help='target host of the measurements')

    # Parse command line arguments
    args = parser.parse_args()
    arg_freq = args.frequency
    arg_host = args.targethost
    if args.npackages <= 0:
        raise argparse.ArgumentTypeError("%s is not a positive int value"
                                         % args.npackages)
    arg_npackages = args.npackages
    arg_label = args.label

    run_daemon(arg_freq, arg_host, arg_npackages, arg_label)
