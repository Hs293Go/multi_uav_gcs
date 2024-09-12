from math import pi

from python_qt_binding import QtCore, QtGui, QtWidgets

from rqt_multi_uav_gcs import qnode


class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        layout = QtWidgets.QGridLayout()

        prefix_label = QtWidgets.QLabel("Vehicle Prefix")
        layout.addWidget(prefix_label, 0, 0)

        self._prefix_editor = QtWidgets.QLineEdit()
        self._prefix_editor.setPlaceholderText(
            "If prefix is empty, node will subscribe to `/mavros/...`"
        )
        layout.addWidget(self._prefix_editor, 0, 1)

        self._add_uav_button = QtWidgets.QPushButton("Connect")
        layout.addWidget(self._add_uav_button, 0, 2)
        self._add_uav_button.clicked.connect(self.add_vehicle)

        spacer = QtWidgets.QSpacerItem(
            60, 40, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred
        )
        layout.addItem(spacer, 0, 3)

        self._tabs = QtWidgets.QTabWidget()
        self._tabs.setStyleSheet("""QTabWidget {
    font-size: 20px;
}""")
        self._tabs.setTabsClosable(True)
        self._tabs.tabCloseRequested.connect(self.remove_tab)

        self._node = qnode.QNode()
        for tab_title, tab_receiver in self._node.vehicles.items():
            self.make_tab(tab_title, tab_receiver)
        layout.addWidget(self._tabs, 1, 0, 1, 4)
        self.setLayout(layout)

    def make_tab(self, name, receiver):
        page = Page(receiver)

        if not name:
            pretty_title = "Main UAV"
        else:
            pretty_title = "Multi UAV: %s" % name
        self._tabs.addTab(page, pretty_title)

    def remove_tab(self, idx):
        pretty_title = self._tabs.tabText(idx)
        if pretty_title == "Main UAV":
            key = ""
        else:
            key = pretty_title.split(":")[1].strip()
        # Shutdown explicitly to ensure no subscribers will trigger callbacks
        # dependent on the expiring objects
        self._node.vehicles.pop(key).shutdown()
        self._tabs.removeTab(idx)

    def add_vehicle(self, _):
        name = self._prefix_editor.text()
        title, receiver = self._node.add_vehicle(name)
        if title is None:
            return
        self.make_tab(title, receiver)
        self._prefix_editor.clear()

    def shutdown(self):
        # Shutdown explicitly to ensure no subscribers will trigger callbacks
        # dependent on the expiring objects
        for it in list(self._node.vehicles):
            self._node.vehicles.pop(it).shutdown()


class Page(QtWidgets.QWidget):
    arming = QtCore.pyqtSignal(bool)
    set_mode = QtCore.pyqtSignal(str)
    update_odom_topic = QtCore.pyqtSignal(str)
    send_refs = QtCore.pyqtSignal(float, float, float, float)

    def __init__(self, receiver):
        super().__init__()
        self._is_armed = False
        self._receiver = receiver

        self.setWindowTitle("multi_uav_gcs_window")

        layout = QtWidgets.QHBoxLayout()
        lhs_frame = QtWidgets.QFrame()
        lhs_layout = QtWidgets.QVBoxLayout()
        lhs_layout.addWidget(self.make_vehicle_state_groupbox())
        lhs_layout.addWidget(self.make_user_command_groupbox())
        lhs_frame.setLayout(lhs_layout)
        layout.addWidget(lhs_frame)

        rhs_frame = QtWidgets.QFrame()
        rhs_layout = QtWidgets.QVBoxLayout()
        rhs_layout.addWidget(self.make_command_state_groupbox())
        rhs_layout.addWidget(self.make_offboard_control_groupbox())
        rhs_frame.setLayout(rhs_layout)
        layout.addWidget(rhs_frame)
        self.setLayout(layout)
        self.setFixedSize(self.sizeHint())

        # Setup Connections
        # ~~~~~~~~~~~~~~~~~
        self._receiver.update_rpy.connect(self.update_rpy)
        self._receiver.update_lla.connect(self.update_lla)
        self._receiver.update_state.connect(self.update_state)
        self._receiver.update_local_position.connect(self.update_local_position)
        self._receiver.update_local_velocity.connect(self.update_local_velocity)
        self._receiver.update_nsats.connect(self.update_nsats)
        self._receiver.update_setpoints.connect(self.update_setpoints)
        self._receiver.update_battery.connect(self.update_battery)
        self.arming.connect(self._receiver.arming)
        self.set_mode.connect(self._receiver.set_mode)
        self.update_odom_topic.connect(self._receiver.update_odom_topic)
        self.send_refs.connect(self._receiver.send_refs)

    # Group Box Definitions
    # ---------------------
    def make_vehicle_state_groupbox(self):
        layout = QtWidgets.QGridLayout()
        box = StyledGroupBox("UAV States")

        self._lla_box = ArrayDisplayGroupBox(
            "Global Position 🌐",
            ["lat (deg)", "lon (deg)", "alt (m-AGL)"],
            digits=(4, 4, 7),
        )
        self._enu_box = ArrayDisplayGroupBox(
            "Local Position ⤱", ["%s (m)" % it for it in "XYZ"]
        )
        self._vel_box = ArrayDisplayGroupBox(
            "Local Velocity ⇶",
            ["v<sub>%s</sub> (ms<sup>-1</sup>)" % it for it in "XYZ"],
        )
        self._rpy_box = ArrayDisplayGroupBox(
            "Orientation ⭯", ["%s (°)" % it for it in ("roll", "pitch", "yaw")]
        )
        layout.addWidget(self._lla_box, 1, 0, 1, 2)
        layout.addWidget(self._rpy_box, 1, 3, 1, 2)
        layout.addWidget(QtWidgets.QLabel("Odometry Topic"), 2, 0)
        self._odometry_line_edit = QtWidgets.QLineEdit()
        if self._receiver.odom_topic:
            self._odometry_line_edit.setPlaceholderText(self._receiver.odom_topic)
        else:
            self._odometry_line_edit.setPlaceholderText("Odometry topic")
        layout.addWidget(self._odometry_line_edit, 2, 1, 1, 3)
        if "odom" in self._receiver.subs:
            self._odom_toggle_button = QtWidgets.QPushButton("Use local position")
        else:
            self._odom_toggle_button = QtWidgets.QPushButton("Use odometry")
        self._odom_toggle_button.clicked.connect(self._toggle_odom_topic)
        layout.addWidget(self._odom_toggle_button, 2, 4)

        layout.addWidget(self._enu_box, 3, 0, 1, 2)
        layout.addWidget(self._vel_box, 3, 3, 1, 2)
        box.setLayout(layout)
        return box

    def make_command_state_groupbox(self):
        box = StyledGroupBox("UAV Status")
        layout = QtWidgets.QGridLayout()

        self._connected_lbl = StatusLabel("Disconnected", "red")
        self._connected_lbl.setFixedWidth(150)
        self._armed_lbl = StatusLabel("Disarmed", "green")
        self._armed_lbl.setFixedWidth(150)
        self._mode_lbl = StatusLabel("Unknown", "black")
        self._mode_lbl.setFixedWidth(200)

        layout.addWidget(self._connected_lbl, 0, 0)
        layout.addWidget(self._armed_lbl, 0, 1)
        layout.addWidget(self._mode_lbl, 0, 2)

        layout.addWidget(StatusLabel("# Sats", "black"), 0, 3)
        self._nsat_box = QtWidgets.QLCDNumber(2)

        self._nsat_box.setMinimumHeight(30)
        self._nsat_box.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        layout.addWidget(self._nsat_box, 0, 4)

        self._battery_level = QtWidgets.QProgressBar()
        self._battery_level.setSizePolicy(
            QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred
        )
        layout.addWidget(StatusLabel("Battery", "black"), 1, 0)
        layout.addWidget(self._battery_level, 1, 1, 1, 2)
        self._voltage_level = StatusLabel("N/A (V)", "black")
        layout.addWidget(self._voltage_level, 1, 3, 1, 2)
        box.setLayout(layout)
        return box

    def make_offboard_control_groupbox(self):
        box = StyledGroupBox("Offboard Control")
        layout = QtWidgets.QGridLayout()

        display_box = QtWidgets.QGroupBox("Setpoints feedback")
        self._setpoints = QtWidgets.QStackedLayout()

        self._sp_boxes = [
            ArrayDisplayGroupBox(
                "Attitude Setpoint",
                ["%s (°)" % it for it in ["roll", "pitch", "yaw"]] + ["Thrust"],
            ),
            ArrayDisplayGroupBox(
                "Body Rates Setpoint",
                ["ω<sub>%s</sub>" % it for it in "XYZ"] + ["Thrust"],
            ),
        ]
        for it in self._sp_boxes:
            self._setpoints.addWidget(it)

        display_box.setLayout(self._setpoints)
        layout.addWidget(display_box)

        refs_box = QtWidgets.QGroupBox("Send References")
        refs_layout = QtWidgets.QGridLayout()

        self._refs_line_edits = []
        self._refs_bounds_line_edits = []
        for idx, it in enumerate(list("xyz") + ["yaw"]):
            if it == "yaw":
                line_edit = BoundedLineEdit(0.0, (-pi, pi))
                label = "yaw reference (°)"
            else:
                line_edit = BoundedLineEdit(0.0)
                label = "%s reference (m)" % it.upper()
            line_edit.setPlaceholderText(label)
            refs_layout.addWidget(QtWidgets.QLabel(label), 0, 2 * idx, 1, 2)
            refs_layout.addWidget(line_edit, 1, 2 * idx, 1, 2)
            self._refs_line_edits.append(line_edit)
            if it != "yaw":
                bounds = (BoundedLineEdit(-5), BoundedLineEdit(5))
                line_edit.set_bounds(float(bounds[0].text()), float(bounds[1].text()))
                for v in bounds:
                    v.setEnabled(False)
                    v.setFixedWidth(80)
                bounds[0].textChanged.connect(
                    lambda lb_str, ub=bounds[1], target=line_edit: target.set_bounds(
                        float(lb_str), float(ub.text())
                    )
                )
                bounds[1].textChanged.connect(
                    lambda ub_str, lb=bounds[0], target=line_edit: target.set_bounds(
                        float(lb.text()), float(ub_str)
                    )
                )

                self._refs_bounds_line_edits.extend(bounds)
                refs_layout.addWidget(
                    QtWidgets.QLabel("Min. %s (m)" % it.upper()), 2, 2 * idx, 1, 1
                )
                refs_layout.addWidget(
                    QtWidgets.QLabel("Max. %s (m)" % it.upper()), 2, 2 * idx + 1, 1, 1
                )
                refs_layout.addWidget(bounds[0], 3, 2 * idx, 1, 1)
                refs_layout.addWidget(bounds[1], 3, 2 * idx + 1, 1, 1)
            else:
                unlock_bounds_checkbox = QtWidgets.QCheckBox("Unlock bounds")

                unlock_bounds_checkbox.stateChanged.connect(
                    lambda s: [
                        bnd.setEnabled(s == QtCore.Qt.Checked)
                        for bnd in self._refs_bounds_line_edits
                    ]
                )
                refs_layout.addWidget(unlock_bounds_checkbox, 3, 7, 1, 1)

        self._copy_position_button = QtWidgets.QPushButton("Copy current position")
        self._copy_position_button.clicked.connect(self._copy_position)
        refs_layout.addWidget(self._copy_position_button, 4, 0, 1, 2)

        self._send_refs_button = QtWidgets.QPushButton("Send Reference")
        self._send_refs_button.clicked.connect(self._send_refs)
        refs_layout.addWidget(self._send_refs_button, 4, 2, 1, 2)

        refs_box.setLayout(refs_layout)
        layout.addWidget(refs_box)
        box.setLayout(layout)

        return box

    def _set_arming_button_style(self):
        self._arming_button.setStyleSheet("""QPushButton {
    color : %s;
    background-color: %s;
    font-size: 25px;
    font-weight: bold;
}""" % (("black", "white") if self._is_armed else ("white", "red")))

    def make_user_command_groupbox(self):
        box = StyledGroupBox("Commands")
        layout = QtWidgets.QGridLayout()

        self._arming_button = QtWidgets.QPushButton(
            "Disarm" if self._is_armed else "Arm"
        )
        self._set_arming_button_style()
        self._arming_button.clicked.connect(self._trigger_arming)
        layout.addWidget(self._arming_button, 0, 0)

        self._mode_menu = QtWidgets.QComboBox()
        px4_modes = [
            "AUTO.PRECLAND",
            "AUTO.FOLLOW_TARGET",
            "AUTO.RTGS",
            "AUTO.LAND",
            "AUTO.RTL",
            "AUTO.MISSION",
            "RATTITUDE",
            "AUTO.LOITER",
            "STABILIZED",
            "AUTO.TAKEOFF",
            "OFFBOARD",
            "POSCTL",
            "ALTCTL",
            "AUTO.READY",
            "ACRO",
            "MANUAL",
        ]
        apm_modes = [
            "GUIDED_NOGPS",
            "AVOID_ADSB",
            "THROW",
            "BRAKE",
            "POSHOLD",
            "AUTOTUNE",
            "FLIP",
            "SPORT",
            "DRIFT",
            "OF_LOITER",
            "LAND",
            "POSITION",
            "CIRCLE",
            "RTL",
            "LOITER",
            "GUIDED",
            "AUTO",
            "ALT_HOLD",
            "ACRO",
            "STABILIZE",
        ]
        self._mode_menu.addItems(px4_modes)
        self._mode_menu.setStyleSheet("""QComboBox {
    font-size: 25px;
    font-weight: bold;
}""")
        self._mode_menu.currentIndexChanged.connect(self._set_mode)
        layout.addWidget(self._mode_menu, 0, 1)
        self._mode_toggle = QtWidgets.QCheckBox("Use APM Modes")
        self._mode_toggle.setStyleSheet("""QCheckBox {
    font-size: 25px;
    font-weight: bold;
}""")
        layout.addWidget(self._mode_toggle, 0, 2)

        def toggle_modeset(state):
            self._mode_menu.clear()
            if state == QtCore.Qt.Checked:
                self._mode_menu.addItems(apm_modes)
            else:
                self._mode_menu.addItems(px4_modes)

        self._mode_toggle.stateChanged.connect(toggle_modeset)

        box.setLayout(layout)
        return box

    # Public Slot definitions
    # -----------------------------------
    #
    # Connected to signals defined by an outside class

    def update_local_position(self, *values):
        self._enu_box.display(*values)

    def update_local_velocity(self, *values):
        self._vel_box.display(*values)

    def update_lla(self, *values):
        self._lla_box.display(*values)

    def update_rpy(self, *values):
        self._rpy_box.display(*values)

    def update_state(self, connected, armed, mode):
        self._is_armed = armed
        connectivity_str = ["Disconnected", "Connected"]
        connectivity_colors = ["yellow", "green"]
        self._connected_lbl.setText(connectivity_str[int(connected)])
        self._connected_lbl.set_color(connectivity_colors[int(connected)])
        armed_str = ["Disarmed", "Armed"]
        armed_colors = ["Green", "Red"]
        self._armed_lbl.setText(armed_str[int(self._is_armed)])
        self._armed_lbl.set_color(armed_colors[int(self._is_armed)])
        self._arming_button.setText("Disarm" if self._is_armed else "Arm")
        self._set_arming_button_style()
        self._mode_lbl.setText(mode)

    def update_nsats(self, value):
        self._nsat_box.display(value)

    def update_battery(self, percentage, voltage):
        self._battery_level.setValue(percentage * 100)
        self._voltage_level.setText("Voltage : %4.1f" % voltage)

    def update_setpoints(self, index, *values):
        self._setpoints.setCurrentIndex(index)
        self._sp_boxes[index].display(*values)

    # Private Slot Definitions
    # ------------------------
    #
    # These are connected to signals defined in this class

    def _trigger_arming(self):
        should_arm = not self._is_armed
        self.arming.emit(should_arm)

    def _set_mode(self, idx):
        self.set_mode.emit(self._mode_menu.itemText(idx))

    def _toggle_odom_topic(self):
        if "odom" not in self._receiver.subs:
            topic = self._odometry_line_edit.text()
            if not topic:
                return
            self.update_odom_topic.emit(topic)
            self._odometry_line_edit.clear()
            self._odometry_line_edit.setPlaceholderText(topic)
            self._odom_toggle_button.setText("Use local position")
        else:
            self.update_odom_topic.emit("")
            self._odometry_line_edit.setPlaceholderText("")
            self._odom_toggle_button.setText("Use odometry")

    def _send_refs(self):
        refs = [float(e.text()) for e in self._refs_line_edits]

        self.send_refs.emit(*refs)

    def _copy_position(self):
        values = self._enu_box.array_values
        yaw_value = self._rpy_box.array_values[-1]

        if values != 3:
            for e, v in zip(self._refs_line_edits, values):
                assert isinstance(e, QtWidgets.QLineEdit)
                e.setText("%.2f" % v)
        self._refs_line_edits[-1].setText("%.2f" % yaw_value)


class ArrayDisplayGroupBox(QtWidgets.QGroupBox):
    DESCRIPTION_ROW = 0
    DISPLAY_ROW = 1

    def __init__(self, title, elements, digits=5):
        super().__init__(title)
        self.setStyleSheet("""QGroupBox
{
    font-size: 20px;
    font-weight: bold;
}
""")
        layout = QtWidgets.QGridLayout()

        self._lcd_numbers = []
        if isinstance(digits, int):
            digits = [digits] * len(elements)
        for idx, lbl in enumerate(elements):
            lbl = QtWidgets.QLabel(lbl)
            lbl_font = QtGui.QFont()
            lbl_font.setPointSize(15)
            lbl.setFont(lbl_font)
            layout.addWidget(lbl, self.DESCRIPTION_ROW, idx)
            lcd_number = QtWidgets.QLCDNumber(digits[idx])
            lcd_number.setMinimumHeight(40)
            lcd_number.setMinimumWidth(160)
            lcd_number.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
            lcd_number.setStyleSheet("""
QLCDNumber {
    color: #33FF00;
    background-color: #171A1B;
}
""")
            layout.addWidget(lcd_number, self.DISPLAY_ROW, idx)
            self._lcd_numbers.append(lcd_number)
        self._array_values = []
        self.setLayout(layout)

    @property
    def array_values(self):
        return self._array_values

    def display(self, *values):
        if len(self._lcd_numbers) != len(values):
            raise RuntimeError("Mismatch between number of fields and values")
        self._array_values[:] = map(float, values)
        for field, val in zip(self._lcd_numbers, values):
            field.display(val)


class StatusLabel(QtWidgets.QLabel):
    def __init__(self, init_text, init_color=None):
        super().__init__(init_text)
        lbl_fontsize = QtGui.QFont()
        lbl_fontsize.setPointSize(15)
        self.setFont(lbl_fontsize)

        if init_color is not None:
            self.set_color(init_color)

    def set_color(self, color):
        self.setStyleSheet("""
QLabel {
    color: %s;
    border : 1px solid %s;
}""" % (color, color))


class StyledGroupBox(QtWidgets.QGroupBox):
    def __init__(self, title):
        super().__init__(title)
        self.setStyleSheet("""QGroupBox
{
    font-size: 25px;
    font-weight: bold;
}
""")


class StrictlyBoundedValidator(QtGui.QDoubleValidator):
    def validate(self, s, i):

        if not s or s == "-":
            return QtGui.QValidator.Intermediate, s, i

        try:
            d = float(s)
        except ValueError:
            return QtGui.QValidator.Invalid, s, i

        if self.bottom() <= d <= self.top():
            return QtGui.QValidator.Acceptable, s, i
        return QtGui.QValidator.Invalid, s, i


class BoundedLineEdit(QtWidgets.QLineEdit):
    def __init__(self, init_value=0.0, bounds=()):
        super().__init__(str(init_value))
        self._validator = StrictlyBoundedValidator()
        if len(bounds) == 2:
            self._validator.setRange(bounds[0], bounds[1])
        self.setValidator(self._validator)

    def set_bounds(self, lb, ub):
        self._validator.setRange(lb, ub)
        self.setValidator(self._validator)
