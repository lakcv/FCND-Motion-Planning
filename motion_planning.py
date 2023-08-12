import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import  heuristic
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from graph_utils import path_pruning, prepare_graph,  a_star_graph, find_start_goal_in_graph

import re
import numpy.linalg as LA

from os.path import exists
import pickle



class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, data, grid, G, north_offset, east_offset):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        self.TARGET_ALTITUDE = 5
        self.SAFETY_DISTANCE = 5
        self.data = data
        self.grid = grid
        self.G = G
        self.north_offset = north_offset
        self.east_offset = east_offset

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 5.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                          self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        print(self.waypoints)
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        saved_waypoints_pkl_fn = 'saved_waypoints_7993ebf.pkl'
        self.target_position[2] = self.TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline()
        [lat0, lon0] = [np.float(s) for s in re.findall(r"\s[-+]?(?:\d*\.*\d+)", first_line)]
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        if not exists(saved_waypoints_pkl_fn):
            # TODO: retrieve current global position
            global_position = np.array([self._longitude, self._latitude, self._altitude])

            # TODO: convert to current local position using global_to_local()
            local_position = global_to_local(global_position, self.global_home)

            print('local_position is ', local_position)
            print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                             self.local_position))

            # Set goal as some arbitrary position on the grid
            # TODO: adapt to set goal as latitude / longitude position and convert
            global_goal = (-122.398974, 37.794297, 0)
            local_goal = global_to_local(global_goal, self.global_home)

            _start = (int(local_position[0]) - self.north_offset, int(local_position[1]) - self.east_offset)
            _goal = (int(local_goal[0]) - self.north_offset, int(local_goal[1]) - self.east_offset)

            node_start, node_goal = find_start_goal_in_graph(self.G, _start, _goal)

            path, _ = a_star_graph(self.G, heuristic, node_start, node_goal)

            # Convert path to waypoints
            # waypoints = [[p[0] + self.north_offset, p[1] + self.east_offset, TARGET_ALTITUDE, 0] for p in path]
            waypoints = [[int(p[0]+ self.north_offset), int(p[1]+ self.east_offset), self.TARGET_ALTITUDE, 0] for p in path]
            with open(saved_waypoints_pkl_fn, 'wb') as f:
                pickle.dump(waypoints, f)
        else:
            with open(saved_waypoints_pkl_fn, 'rb') as f:
                waypoints = pickle.load(f)
        # TODO: prune path to minimize number of waypoints
        print('Len(waypoints) = ',len(waypoints))
        pruned_waypoints = path_pruning(self.grid,waypoints , self.north_offset , self.east_offset)
        for i , _ in enumerate(pruned_waypoints):
            angle = np.arctan2(pruned_waypoints[i][1],pruned_waypoints[i][0])
            pruned_waypoints[i][3] = angle
        print('Len(pruned_waypoints) = ', len(pruned_waypoints))

        # Set self.waypoints
        self.waypoints = pruned_waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()
    saved_graph_fn = 'saved_graph_v1.pkl'
    if exists(saved_graph_fn):
        with open(saved_graph_fn, 'rb') as f:
            data, grid, G, north_offset, east_offset = pickle.load(f)
    else:
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5
        data, grid, G, north_offset, east_offset = prepare_graph('colliders.csv', TARGET_ALTITUDE, SAFETY_DISTANCE)
        with open(saved_graph_fn, 'wb') as f:
            pickle.dump([data, grid, G, north_offset, east_offset], f)

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, data, grid, G, north_offset, east_offset)
    time.sleep(1)

    drone.start()
