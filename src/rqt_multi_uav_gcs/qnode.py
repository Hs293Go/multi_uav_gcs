import collections.abc

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from python_qt_binding import QtCore
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import UInt32


class VehicleNode(QtCore.QObject):
    update_local_position = QtCore.pyqtSignal(float, float, float)
    update_rpy = QtCore.pyqtSignal(float, float, float)
    update_battery = QtCore.pyqtSignal(float, float)
    update_lla = QtCore.pyqtSignal(float, float, float)
    update_local_velocity = QtCore.pyqtSignal(float, float, float)
    update_state = QtCore.pyqtSignal(bool, bool, str)
    update_nsats = QtCore.pyqtSignal(int)
    update_setpoints = QtCore.pyqtSignal(int, float, float, float, float)

    def __init__(self, prefix="", odom_topic=""):
        super().__init__()
        self._odom_topic = odom_topic
        self._prefix = prefix

        self._arming_client = rospy.ServiceProxy(
            f"{self._prefix}/mavros/cmd/arming", CommandBool
        )
        self._set_mode_client = rospy.ServiceProxy(
            f"{self._prefix}/mavros/set_mode", SetMode
        )
        self._set_home_client = rospy.ServiceProxy(
            f"{self._prefix}/mavros/set_home", CommandHome
        )
        self._subs = {
            "imu": rospy.Subscriber(
                f"{self._prefix}/mavros/imu/data", Imu, self._imu_cb, queue_size=1
            ),
            "lla": rospy.Subscriber(
                f"{self._prefix}/mavros/global_position/global",
                NavSatFix,
                self._lla_cb,
                queue_size=1,
            ),
            "state": rospy.Subscriber(
                f"{self._prefix}/mavros/state", State, self._state_cb, queue_size=1
            ),
            "nsats": rospy.Subscriber(
                f"{self._prefix}/mavros/global_position/raw/satellites",
                UInt32,
                self._nsat_cb,
                queue_size=1,
            ),
            "sp": rospy.Subscriber(
                f"{self._prefix}/mavros/setpoint_raw/attitude",
                AttitudeTarget,
                self._setpoint_cb,
                queue_size=1,
            ),
            "batt": rospy.Subscriber(
                f"{self._prefix}/mavros/battery",
                BatteryState,
                self._battery_cb,
                queue_size=1,
            ),
        }
        self.update_odom_topic(self._odom_topic)

    @property
    def subs(self):
        return self._subs

    @property
    def odom_topic(self):
        return self._odom_topic

    def subscribe_odom(self):
        if "odom" in self._subs:
            return
        for it in ("enu", "vel"):
            if it in self._subs:
                self._subs.pop(it).unregister()

        self._subs["odom"] = rospy.Subscriber(
            f"{self._prefix}{self._odom_topic}", Odometry, self._odom_cb
        )

    def subscribe_local_position(self):
        if "enu" in self._subs and "vel" in self._subs:
            return
        if "odom" in self._subs:
            self._subs.pop("odom").unregister()

        self._subs["enu"] = rospy.Subscriber(
            f"{self._prefix}/mavros/local_position/pose", PoseStamped, self._pose_cb
        )

        self._subs["vel"] = rospy.Subscriber(
            f"{self._prefix}/mavros/local_position/velocity_local",
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
            self.update_setpoints.emit(0, msg.thrust, *angles)

        if msg.type_mask == AttitudeTarget.IGNORE_ATTITUDE:
            self.update_setpoints.emit(
                1, msg.thrust, *[getattr(msg.body_rate, it) for it in "xyz"]
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
            name = f"/{name}"
        if name in self._vehicles:
            return None, None
        self._vehicles[name] = VehicleNode(name, self._odom_topic)
        return name, self._vehicles[name]

    @property
    def vehicles(self):
        return self._vehicles
