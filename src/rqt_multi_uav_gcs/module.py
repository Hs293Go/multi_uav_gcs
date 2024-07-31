from qt_gui.plugin import Plugin

from rqt_multi_uav_gcs import main_window


class MultiUAVGCSPlugin(Plugin):

    def __init__(self, context):
        super(MultiUAVGCSPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName("MultiUAVGCSPlugin")

        # Create QWidget
        self._widget = main_window.MainWindow()
        # Get path to UI file which should be in the "resource" folder of this package

        # Give QObjects reasonable names
        self._widget.setObjectName("Multi UAV GCS")
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
