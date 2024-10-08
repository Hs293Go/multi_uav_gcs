import collections.abc

import rospy
from fsc_autopilot_msgs.msg import PositionControllerReference
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandInt, FileRead, SetMode
from nav_msgs.msg import Odometry
from python_qt_binding import QtCore
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Int8, UInt32
from std_srvs.srv import Trigger


class VehicleNode(QtCore.QObject):
    update_local_position = QtCore.pyqtSignal(float, float, float)
    update_rpy = QtCore.pyqtSignal(float, float, float)
    update_battery = QtCore.pyqtSignal(float, float)
    update_lla = QtCore.pyqtSignal(float, float, float)
    update_local_velocity = QtCore.pyqtSignal(float, float, float)
    update_state = QtCore.pyqtSignal(bool, bool, str)
    update_nsats = QtCore.pyqtSignal(int)
    update_setpoints = QtCore.pyqtSignal(int, float, float, float, float)
    update_traj_progress = QtCore.pyqtSignal(int)

    def __init__(self, prefix="", odom_topic=""):
        super().__init__()
        self._odom_topic = odom_topic
        self._prefix = prefix

        self._arming_client = rospy.ServiceProxy(
            "%s/mavros/cmd/arming" % self._prefix, CommandBool
        )
        self._set_mode_client = rospy.ServiceProxy(
            "%s/mavros/set_mode" % self._prefix, SetMode
        )
        self._set_home_client = rospy.ServiceProxy(
            "%s/state_estimator/override_set_home" % self._prefix, CommandInt
        )
        self._start_exectraj_client = rospy.ServiceProxy(
            "%s/trajectory_publisher/start" % self._prefix, Trigger
        )
        self._stop_exectraj_client = rospy.ServiceProxy(
            "%s/trajectory_publisher/stop" % self._prefix, Trigger
        )
        self._load_trajectory_file_client = rospy.ServiceProxy(
            "%s/trajectory_publisher/read_trajectory_file" % self._prefix, FileRead
        )

        self._subs = {
            "imu": rospy.Subscriber(
                "%s/mavros/imu/data" % self._prefix, Imu, self._imu_cb, queue_size=1
            ),
            "lla": rospy.Subscriber(
                "%s/mavros/global_position/global" % self._prefix,
                NavSatFix,
                self._lla_cb,
                queue_size=1,
            ),
            "state": rospy.Subscriber(
                "%s/mavros/state" % self._prefix, State, self._state_cb, queue_size=1
            ),
            "nsats": rospy.Subscriber(
                "%s/mavros/global_position/raw/satellites" % self._prefix,
                UInt32,
                self._nsat_cb,
                queue_size=1,
            ),
            "sp": rospy.Subscriber(
                "%s/mavros/setpoint_raw/attitude" % self._prefix,
                AttitudeTarget,
                self._setpoint_cb,
                queue_size=1,
            ),
            "batt": rospy.Subscriber(
                "%s/mavros/battery" % self._prefix,
                BatteryState,
                self._battery_cb,
                queue_size=1,
            ),
            "trajectory_prog": rospy.Subscriber(
                "%s/trajectory_publisher/progress" % self._prefix,
                Int8,
                self._trajectory_cb,
                queue_size=1,
            ),
        }
        self.update_odom_topic(self._odom_topic)

        self._target_pub = rospy.Publisher(
            "%s/fsc_autopilot/position_controller/reference" % self._prefix,
            PositionControllerReference,
            queue_size=1,
        )

    @property
    def subs(self):
        return self._subs

    @property
    def odom_topic(self):
        return self._odom_topic

    def send_refs(self, x, y, z, yaw):
        if self._target_pub is None:
            return

        msg = PositionControllerReference()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.yaw = yaw
        msg.yaw_unit = PositionControllerReference.DEGREES
        msg.header.stamp = rospy.Time.now()
        self._target_pub.publish(msg)

    def subscribe_odom(self):
        if "odom" in self._subs:
            return
        for it in ("enu", "vel"):
            if it in self._subs:
                self._subs.pop(it).unregister()

        self._subs["odom"] = rospy.Subscriber(
            "%s%s" % (self._prefix, self._odom_topic), Odometry, self._odom_cb
        )

    def subscribe_local_position(self):
        if "enu" in self._subs and "vel" in self._subs:
            return
        if "odom" in self._subs:
            self._subs.pop("odom").unregister()

        self._subs["enu"] = rospy.Subscriber(
            "%s/mavros/local_position/pose" % self._prefix, PoseStamped, self._pose_cb
        )

        self._subs["vel"] = rospy.Subscriber(
            "%s/mavros/local_position/velocity_local" % self._prefix,
            TwistStamped,
            self._twist_cb,
        )

    def update_odom_topic(self, odom_topic):
        if odom_topic:
            self._odom_topic = odom_topic
            self.subscribe_odom()
        else:
            self.subscribe_local_position()

    def arming(self, value):
        self._arming_client(value)

    def set_mode(self, mode_str):
        self._set_mode_client(custom_mode=mode_str)

    def set_home(self, home_x, home_y, home_z):
        self._set_home_client(param1=home_x, param2=home_y, param3=home_z)

    def toggle_trajectory_exec(self, value):
        if value:
            self._start_exectraj_client.call()
        else:
            self._stop_exectraj_client.call()

    def load_trajectory_file(self, file_path):
        self._load_trajectory_file_client.call(file_path=file_path)

    def _imu_cb(self, msg):
        angles = Rotation.from_quat(
            [getattr(msg.orientation, it) for it in "xyzw"]
        ).as_euler("xyz", True)
        self.update_rpy.emit(*angles)

    def _battery_cb(self, msg):
        self.update_battery.emit(msg.percentage, msg.voltage)

    def _setpoint_cb(self, msg: AttitudeTarget):
        if (
            msg.type_mask
            == AttitudeTarget.IGNORE_ROLL_RATE
            | AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
        ):
            angles = Rotation.from_quat(
                [getattr(msg.orientation, it) for it in "xyzw"]
            ).as_euler("xyz", True)
            self.update_setpoints.emit(0, *angles, msg.thrust)

        if msg.type_mask == AttitudeTarget.IGNORE_ATTITUDE:
            self.update_setpoints.emit(
                1, *[getattr(msg.body_rate, it) for it in "xyz"], msg.thrust
            )

    def _odom_cb(self, msg):
        self.update_local_position.emit(
            *[getattr(msg.pose.pose.position, it) for it in "xyz"]
        )

        self.update_local_velocity.emit(
            *[getattr(msg.twist.twist.linear, it) for it in "xyz"]
        )

    def _pose_cb(self, msg):
        self.update_local_position.emit(
            *[getattr(msg.pose.position, it) for it in "xyz"]
        )

    def _twist_cb(self, msg):
        self.update_local_velocity.emit(
            *[getattr(msg.twist.linear, it) for it in "xyz"]
        )

    def _lla_cb(self, msg):
        self.update_lla.emit(msg.latitude, msg.longitude, msg.altitude)

    def _state_cb(self, msg):
        self.update_state.emit(msg.connected, msg.armed, msg.mode)

    def _nsat_cb(self, msg):
        self.update_nsats.emit(msg.data)

    def _trajectory_cb(self, msg):
        self.update_traj_progress.emit(msg.data)

    def shutdown(self):
        for it in self._subs.values():
            it.unregister()


class QNode:
    def __init__(self):
        super().__init__()
        prefixes = rospy.get_param("~prefixes", "")
        self._odom_topic = str(rospy.get_param("~odom_topic", ""))

        if isinstance(prefixes, str):
            self._vehicles = {"": VehicleNode(prefixes, self._odom_topic)}
            return

        if not isinstance(prefixes, collections.abc.Sequence):
            raise rospy.exceptions.ROSException(
                "Prefix(es) must be either a single string or a list of strings"
            )

        if not prefixes:
            raise rospy.exceptions.ROSException("Prefixes must not be empty")

        self._vehicles = {}
        for it in prefixes:
            self.add_vehicle(it)

    def add_vehicle(self, name):
        name = str(name).strip().rstrip("/")
        if name and not name.startswith("/"):
            name = "/%s" % name
        if name in self._vehicles:
            return None, None
        self._vehicles[name] = VehicleNode(name, self._odom_topic)
        return name, self._vehicles[name]

    @property
    def vehicles(self):
        return self._vehicles
