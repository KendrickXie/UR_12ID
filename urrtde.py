import argparse
import logging
import sys
import threading
sys.path.append('..')
import rtde as rtde
import rtde.rtde_config as rtde_config
import os

#parameters
parser = argparse.ArgumentParser()
parser.add_argument('--host', default='164.54.122.96', help='name of host to connect to (localhost)')
parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
#parser.add_argument('--samples', type=int, default=2, help='number of samples to record')
parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Herz')
parser.add_argument('--config', default='record_configuration.xml', help='data configuration file to use (record_configuration.xml)')
#parser.add_argument('--output', default='robot_data.csv', help='data output file to write to (robot_data.csv)')
parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
parser.add_argument("--buffered", help="Use buffered receive which doesn't skip data", action="store_true")
parser.add_argument("--binary", help="save the data in binary format", action="store_true")
args = parser.parse_args()

if args.verbose:
    logging.basicConfig(level=logging.INFO)

dirname = os.path.dirname(os.path.abspath(__file__))
conf = rtde_config.ConfigFile(os.path.join(dirname, args.config))
output_names, output_types = conf.get_recipe('out')

con = rtde.RTDE(args.host, args.port)


class URRTMonitor(threading.Thread):

    def __init__(self, urHost):
        threading.Thread.__init__(self)
        self.logger = logging.getLogger(self.__class__.__name__)
        self._stop_event = True
        self._csys_lock = threading.Lock()

    def set_csys(self, csys):
        with self._csys_lock:
            self._csys = csys

    def q_actual(self, wait=False, timestamp=False):
        """ Get the actual joint position vector."""
        if timestamp:
            return self.state.timestamp, self.state.actual_q
        else:
            return self.state.actual_q
    getActual = q_actual

    def q_target(self, wait=False, timestamp=False):
        """ Get the target joint position vector."""
        if timestamp:
            return self.state.timestamp, self.state.target_q
        else:
            return self.state.target_q
    getTarget = q_target

    def tcf_pose(self, wait=False, timestamp=False, ctrlTimestamp=False):
        """ Return the tool pose values."""

        tcf = self.state.actual_TCP_pose
        if ctrlTimestamp or timestamp:
            ret = [tcf]
            if timestamp:
                ret.insert(-1, self.state.timestamp)
            if ctrlTimestamp:
                ret.insert(-1, self.state.actual_execution_time)
            return ret
        else:
            return tcf
    getTCF = tcf_pose

    def tcf_force(self, wait=False, timestamp=False):
        """ Get the tool force. The returned tool force is a
        six-vector of three forces and three moments."""
        tcf_force = self.state.actual_TCP_force
        if timestamp:
            return self.state.timestamp, tcf_force
        else:
            return tcf_force
    getTCFForce = tcf_force

    def stop(self):
        # print(self.__class__.__name__+': Stopping')
        self._stop_event = True

    def close(self):
        self.stop()
        self.join()

    def run(self):
        #start data synchronization
        self._stop_event = False

        con.connect()

        # get controller version
        con.get_controller_version()

        # setup recipes
        if not con.send_output_setup(output_names, output_types, frequency = args.frequency):
            logging.error('Unable to configure output')
            sys.exit()

        if not con.send_start():
            logging.error('Unable to start synchronization')
            sys.exit()

        while not self._stop_event:
            if args.buffered:
                self.state = con.receive_buffered(args.binary)
            else:
                self.state = con.receive(args.binary)
        con.send_pause()
        con.disconnect()
