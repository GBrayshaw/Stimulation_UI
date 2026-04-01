import argparse
import json
import time

#import Pyro5.api
from cri.controller import MG400Controller as Controller
from cri.robot import AsyncRobot, SyncRobot

class RobotController:
    def __init__(self, connect: bool = True):
        self.base_frame = (350, 12.1, -35, 0, 0, 165)	# base frame: x->front, y->left, z->up, rz->anticlockwise
        self.work_frame = (350, 12.1, -80, 0, 0, 165)
        self.tap_move = [0, 0, 0, 0, 0, 0], [0, 0, -(2 + 0), 0, 0, 0]
        self.tcp = (0, 0, -50, 0, 0, 0)
        self.linear_speed = 0

        # In-memory metadata dictionary describing the robot configuration
        self.meta = {
            "base_frame": list(self.base_frame),
            "work_frame": list(self.work_frame),
            "tap_move": self.tap_move,
            "tcp": list(self.tcp),
            "linear_speed": self.linear_speed,
        }

        # Optional hardware connection. For UI meta operations we may
        # construct this controller with connect=False to avoid blocking
        # if robot hardware is unavailable.
        self.robot = None
        if connect:
            try:
                self.robot = self._make_robot()
            except Exception:
                self.robot = None

            # If the robot was created successfully, try to apply the
            # configured frames and speed, but do not discard the robot
            # object if any of these assignments fail.
            if self.robot is not None:
                try:
                    self.robot.tcp = self.tcp
                except Exception:
                    pass
                try:
                    self.robot.coord_frame = self.base_frame
                except Exception:
                    pass
                try:
                    self.robot.speed = self.linear_speed
                except Exception:
                    pass

    def _make_robot(self) -> AsyncRobot:
        return AsyncRobot(SyncRobot(Controller()))

    def close_robot(self):
        if self.robot is not None:
            try:
                self.robot.close()
            except Exception:
                pass
            self.robot = None

    def ping(self):
        return "ok"

    def get_state(self):
        return {
            "connected": self.robot is not None,
            "meta": {
                "base_frame": list(self.base_frame),
                "work_frame": list(self.work_frame),
                "tap_move": self.tap_move,
                "tcp": list(self.tcp),
                "linear_speed": self.linear_speed,
            },
        }

    def make_meta(self, filename):
        """Save the current robot configuration to a JSON meta file."""
        # Refresh meta from current attributes
        self.meta = {
            "base_frame": list(self.base_frame),
            "work_frame": list(self.work_frame),
            "tap_move": self.tap_move,
            "tcp": list(self.tcp),
            "linear_speed": self.linear_speed,
        }
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(self.meta, f, indent=2)

    def get_meta(self, filename):
        """Load robot configuration from a JSON meta file into this instance."""
        with open(filename, "r", encoding="utf-8") as f:
            data = json.load(f)

        # Update attributes from the loaded meta, falling back to existing
        # values when keys are missing.
        if "base_frame" in data:
            self.base_frame = tuple(data["base_frame"])
        if "work_frame" in data:
            self.work_frame = tuple(data["work_frame"])
        if "tap_move" in data:
            self.tap_move = data["tap_move"]
        if "tcp" in data:
            self.tcp = tuple(data["tcp"])
        if "linear_speed" in data:
            self.linear_speed = data["linear_speed"]

        # Keep meta in sync
        self.meta = {
            "base_frame": list(self.base_frame),
            "work_frame": list(self.work_frame),
            "tap_move": self.tap_move,
            "tcp": list(self.tcp),
            "linear_speed": self.linear_speed,
        }

        # If a robot instance exists, update its runtime configuration
        if getattr(self, "robot", None) is not None:
            try:
                self.robot.tcp = self.tcp
            except Exception:
                pass
            try:
                self.robot.coord_frame = self.base_frame
            except Exception:
                pass
            try:
                self.robot.speed = self.linear_speed
            except Exception:
                pass

    def move_home(self):
        """Move the robot to the home pose in the base frame."""
        if self.robot is None:
            raise RuntimeError("Robot not connected")
        # Ensure we are in the correct frame, then move to origin
        try:
            self.robot.coord_frame = self.base_frame
        except Exception:
            pass
        self.robot.move_linear((0, 0, 0, 0, 0, 0))

    def perform_tap(self, hold_time: int = 0):
        """Perform a tap motion using the configured tap_move poses.

        Expects tap_move to be a 2-element sequence of poses
        [up_pose, down_pose]. The robot moves home, into the work frame,
        then down to the tap, holds for hold_time seconds, returns up,
        and finally goes home again.
        """
        if self.robot is None:
            raise RuntimeError("Robot not connected")

        tap_move = self.meta.get("tap_move", None)
        if tap_move is None:
            raise RuntimeError("Tap move not configured")
        if not isinstance(tap_move, (list, tuple)) or len(tap_move) != 2:
            raise RuntimeError("tap_move must be a sequence of two poses [up, down]")

        up_pose, down_pose = tap_move

        # Go to home, then into work frame at origin
        self.move_home()
        try:
            self.robot.coord_frame = self.work_frame
        except Exception:
            pass
        self.robot.move_linear((0, 0, 0, 0, 0, 0))

        # Execute tap: down, hold, up
        self.robot.move_linear(tuple(down_pose))
        if hold_time > 0:
            time.sleep(hold_time)
        self.robot.move_linear(tuple(up_pose))

        # Return home when finished
        self.move_home()


def serve_robot_controller(
    *,
    name: str = "robot.controller",
    ns_host: str | None = None,
    ns_port: int | None = None,
    bind_host: str | None = None,
    bind_port: int = 0,
    connect: bool = True,
):
    """Expose RobotController on a Pyro5 nameserver."""
    controller = RobotController(connect=connect)
    daemon = Pyro5.api.Daemon(host=bind_host, port=bind_port)
    uri = daemon.register(controller)

    try:
        ns = Pyro5.api.locate_ns(host=ns_host, port=ns_port)
        ns.register(name, uri)
        print(f"RobotController registered as '{name}' at {uri}")
        daemon.requestLoop()
    finally:
        try:
            ns.remove(name)
        except Exception:
            pass
        daemon.close()
        controller.close_robot()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Expose RobotController via Pyro5")
    parser.add_argument("--name", default="robot.controller", help="Pyro nameserver object name")
    parser.add_argument("--ns-host", default=None, help="Pyro nameserver host")
    parser.add_argument("--ns-port", type=int, default=None, help="Pyro nameserver port")
    parser.add_argument("--bind-host", default=None, help="Local interface/IP for the Pyro daemon")
    parser.add_argument("--bind-port", type=int, default=0, help="Local port for the Pyro daemon")
    parser.add_argument(
        "--no-connect",
        action="store_true",
        help="Start without connecting to the physical robot",
    )
    args = parser.parse_args()

    serve_robot_controller(
        name=args.name,
        ns_host=args.ns_host,
        ns_port=args.ns_port,
        bind_host=args.bind_host,
        bind_port=args.bind_port,
        connect=not args.no_connect,
    )
