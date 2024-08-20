from python_qt_binding import QtGui, QtWidgets


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


class StyledGroupBox(QtWidgets.QGroupBox):
    def __init__(self, title):
        super().__init__(title)
        self.setStyleSheet("""QGroupBox
{
    font-size: 25px;
    font-weight: bold;
}
""")


class StatusLabel(QtWidgets.QLabel):
    def __init__(self, init_text, init_color=None, width=0):
        super().__init__(init_text)
        lbl_fontsize = QtGui.QFont()
        lbl_fontsize.setPointSize(15)
        self.setFont(lbl_fontsize)

        if width:
            self.setFixedWidth(width)

        if init_color is not None:
            self.set_color(init_color)

    def set_color(self, color):
        self.setStyleSheet("""
QLabel {
    color: %s;
    border : 1px solid %s;
}""" % (color, color))


class ArrayDisplayGroupBox(QtWidgets.QGroupBox):
    DESCRIPTION_ROW = 0
    DISPLAY_ROW = 1

    def __init__(self, title, elements, digits=4, width=0, height=0):
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
            if height:
                lcd_number.setMinimumHeight(height)
            if width:
                lcd_number.setMinimumWidth(width)
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
