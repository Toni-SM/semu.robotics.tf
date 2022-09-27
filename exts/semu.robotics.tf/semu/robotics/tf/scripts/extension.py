import time
import threading

import omni
import carb
import omni.ext
import omni.ui as ui
from pxr import UsdGeom
from omni.kit.viewport.utility import get_active_viewport_window
from omni.isaac.ui.ui_utils import add_line_rect_flourish, get_style, SimpleCheckBox, LABEL_WIDTH, LABEL_HEIGHT

from .viewport_scene import ViewportScene
from . import ros_tf_listener as _ros_tf_listener
from . import ros2_tf_listener as _ros2_tf_listener


class Extension(omni.ext.IExt):
    
    WINDOW_NAME = "TF Viewer"
    MENU_PATH = f"Window/Robotics/{WINDOW_NAME}"

    def on_startup(self, ext_id):
        # get extension settings and manager
        self._settings = carb.settings.get_settings()
        self._ext_manager = omni.kit.app.get_app().get_extension_manager()

        # menu item
        self._editor_menu = omni.kit.ui.get_editor_menu()
        if self._editor_menu:
            self._menu = self._editor_menu.add_item(Extension.MENU_PATH, self._menu_callback, toggle=True, value=False)

        self._viewport_window = get_active_viewport_window()
        self._viewport_scene = ViewportScene(self._viewport_window, ext_id)

        self._window = None
        self._running = False
        self._tf_listener = None

        self._frames = set(["world", "map"])
        self._root_frame = "world"
        self._update_frequency = 20

    def on_shutdown(self):
        self._running = False
        # destroy scene
        if self._viewport_scene:
            self._viewport_scene.manipulator.clear()
            self._viewport_scene.destroy()
            self._viewport_scene = None
        # clean up menu item
        if self._menu is not None:
            try:
                self._editor_menu.remove_item(self._menu)
            except:
                self._editor_menu.remove_item(Extension.MENU_PATH)
            self._menu = None

    def _menu_callback(self, *args, **kwargs):
        # stage units
        stage = omni.usd.get_context().get_stage()
        self._stage_unit = UsdGeom.GetStageMetersPerUnit(stage)
        # window
        self._build_ui()
        self._window.visible = not self._window.visible

    def _update_frames(self, frames):
        previous_len = len(self._frames)
        self._frames.update(frames)
        # update ui
        if previous_len != len(self._frames):
            frames = sorted(self._frames)
            # root frame
            root_frame = self._root_frame
            for item in self._ui_root_frame_combo_box.get_item_children():
                self._ui_root_frame_combo_box.remove_item(item)
            for frame in frames:
                self._ui_root_frame_combo_box.append_child_item(None, ui.SimpleStringModel(frame))
            self._ui_root_frame_combo_box.get_item_value_model().set_value(frames.index(root_frame))
            self._root_frame = root_frame

    def _on_window(self, status):
        version = ""
        if self._ext_manager.is_extension_enabled("omni.isaac.ros_bridge"):
            version = "ros"
        elif self._ext_manager.is_extension_enabled("omni.isaac.ros2_bridge"):
            version = "ros2"
        else:
            carb.log_warn("Neither extension 'omni.isaac.ros_bridge' nor 'omni.isaac.ros2_bridge' is enabled")

        if status:
            if version:
                carb.log_info("Acquiring TF listener ({})...".format(version.upper()))
                # acquire listener
                if version == "ros":
                    self._tf_listener = _ros_tf_listener.acquire_tf_listener_interface(use_tf2=True)
                elif version == "ros2":
                    self._tf_listener = _ros2_tf_listener.acquire_tf_listener_interface()
                # update thread
                threading.Thread(target=self._update_transform_thread).start()
                carb.log_info("TF listener status: {}".format(self._tf_listener.is_ready()))
        else:
            carb.log_info("Releasing TF listener...")
            self._running = False
            # release listener
            if version == "ros":
                _ros_tf_listener.release_tf_listener_interface(self._tf_listener)
            elif version == "ros2":
                _ros2_tf_listener.release_tf_listener_interface(self._tf_listener)
            self._tf_listener = None
            # clear scene
            if self._viewport_scene:
                self._viewport_scene.manipulator.clear()
            carb.log_info("TF listener released")

    def _update_transform_thread(self, *args, **kwargs):
        self._running = True
        while self._running:
            # get transforms
            transforms, relations = self._tf_listener.get_transforms(self._root_frame)
            # update frames
            self._update_frames(list(transforms.keys()))
            # draw scene
            self._viewport_scene.manipulator.update_transforms(transforms, relations)
            time.sleep(1 / self._update_frequency)
        # clear scene
        if self._viewport_scene:
            self._viewport_scene.manipulator.clear()

    def _on_reset(self):
        if self._tf_listener:
            self._tf_listener.reset()

    def _on_root_frame_changed(self, model, item):
        try:
            selected_index = model.get_item_value_model().get_value_as_int()
            item = model.get_item_value_model(model.get_item_children()[selected_index])
            self._root_frame = item.get_value_as_string()
        except Exception as e:
            pass

    def _on_update_frequency_changed(self, model):
        frequency = model.as_int
        if frequency <= 0:
            frequency = 20
        self._update_frequency = frequency

    def _build_ui(self):
        if not self._window:
            self._window = ui.Window(title=Extension.WINDOW_NAME, 
                                     width=0, 
                                     height=0, 
                                     visible=False, 
                                     dockPreference=ui.DockPreference.LEFT_BOTTOM)
            self._window.set_visibility_changed_fn(self._on_window)

            with self._window.frame:
                with ui.VStack(spacing=5, height=0):

                    self._view = ui.CollapsableFrame(title="TF Viewer",
                                                     height=0,
                                                     collapsed=False,
                                                     style=get_style(),
                                                     style_type_name_override="CollapsableFrame",
                                                     horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                                     vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON)
                    with self._view:
                        with ui.VStack(spacing=5, height=0):
                            
                            with ui.HStack():
                                items = sorted(self._frames)
                                tooltip = "Frame on which to compute the transormations"
                                ui.Label("Root Frame:", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                self._ui_root_frame_combo_box = ui.ComboBox(0, *items, name="", width=ui.Fraction(1), 
                                                                            alignment=ui.Alignment.LEFT_CENTER).model
                                self._ui_root_frame_combo_box.add_item_changed_fn(self._on_root_frame_changed)
                                add_line_rect_flourish(False)

                            # frames
                            with ui.HStack():
                                tooltip = "Whether the frames (markers) are displayed"
                                ui.Label("Show Frames:", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                # show frames
                                self._ui_show_frames_checkbox = ui.SimpleBoolModel()
                                SimpleCheckBox(checked=True, 
                                               model=self._ui_show_frames_checkbox,
                                               on_checked_fn=lambda checked: self._viewport_scene.manipulator.set_frames_show(checked))
                                # frame color
                                ui.Spacer(width=2)
                                self._ui_show_frames_color = ui.ColorWidget(1.0, 1.0, 1.0, 1.0, width=0, tooltip="Marker color").model
                                for i, child in enumerate(self._ui_show_frames_color.get_item_children()):
                                    item_model = self._ui_show_frames_color.get_item_value_model(child)
                                    item_model.add_value_changed_fn(lambda m, i=i: self._viewport_scene.manipulator.set_frames_color(i, m.get_value_as_float()))
                                # frame size
                                ui.Spacer(width=12)
                                self._ui_show_frames_size = ui.FloatDrag(height=LABEL_HEIGHT, min=0, max=1, alignment=ui.Alignment.LEFT_CENTER, 
                                                                         tooltip="Marker size (relative)").model
                                self._ui_show_frames_size.add_value_changed_fn(lambda m: self._viewport_scene.manipulator.set_frames_size(m.as_float))
                                self._ui_show_frames_size.set_value(0.25)
                                ui.Spacer(width=5)
                                add_line_rect_flourish()

                            # names
                            with ui.HStack():
                                tooltip = "Whether the frames' names are displayed"
                                ui.Label("Show Names:", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                # show names
                                self._ui_show_names_checkbox = ui.SimpleBoolModel()
                                SimpleCheckBox(checked=True, 
                                               model=self._ui_show_names_checkbox,
                                               on_checked_fn=lambda checked: self._viewport_scene.manipulator.set_names_show(checked))
                                # text color
                                ui.Spacer(width=2)
                                self._ui_show_names_color = ui.ColorWidget(1.0, 1.0, 0.0, 1.0, width=0, tooltip="Text color").model
                                for i, child in enumerate(self._ui_show_names_color.get_item_children()):
                                    item_model = self._ui_show_names_color.get_item_value_model(child)
                                    item_model.add_value_changed_fn(lambda m, i=i: self._viewport_scene.manipulator.set_names_color(i, m.get_value_as_float()))
                                # text size
                                ui.Spacer(width=12)
                                self._ui_show_names_size = ui.FloatDrag(height=LABEL_HEIGHT, min=0, max=1, alignment=ui.Alignment.LEFT_CENTER, 
                                                                        tooltip="Text size (relative)").model
                                self._ui_show_names_size.add_value_changed_fn(lambda m: self._viewport_scene.manipulator.set_names_size(m.as_float))
                                self._ui_show_names_size.set_value(0.4)
                                ui.Spacer(width=5)
                                add_line_rect_flourish()

                            # axes
                            with ui.HStack():
                                tooltip = "Whether the frames's axes are displayed (RGB -> XYZ axes)"
                                ui.Label("Show Axes:", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                # show axes
                                self._ui_show_axes_checkbox = ui.SimpleBoolModel()
                                SimpleCheckBox(checked=True, 
                                               model=self._ui_show_axes_checkbox,
                                               on_checked_fn=lambda checked: self._viewport_scene.manipulator.set_axes_show(checked))
                                # axes length
                                ui.Spacer(width=2)
                                self._ui_show_axes_length = ui.FloatDrag(height=LABEL_HEIGHT, min=0, max=1, alignment=ui.Alignment.LEFT_CENTER,
                                                                         tooltip="Axis length (meters)").model
                                self._ui_show_axes_length.add_value_changed_fn(lambda m: self._viewport_scene.manipulator.set_axes_length(m.as_float / self._stage_unit))
                                self._ui_show_axes_length.set_value(0.15)
                                # axes thickness
                                ui.Spacer(width=12)
                                self._ui_show_axes_thickness = ui.FloatDrag(height=LABEL_HEIGHT, min=0, max=1, alignment=ui.Alignment.LEFT_CENTER, 
                                                                            tooltip="Axis thickness (relative)").model
                                self._ui_show_axes_thickness.add_value_changed_fn(lambda m: self._viewport_scene.manipulator.set_axes_thickness(m.as_float))
                                self._ui_show_axes_thickness.set_value(0.2)
                                ui.Spacer(width=5)
                                add_line_rect_flourish()

                            # arrows
                            with ui.HStack():
                                tooltip = "Whether to show the connection between the child frames and the parent frames"
                                ui.Label("Show Arrows:", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                # show arrows
                                self._ui_show_arrows_checkbox = ui.SimpleBoolModel()
                                SimpleCheckBox(checked=True,
                                               model=self._ui_show_arrows_checkbox,
                                               on_checked_fn=lambda checked: self._viewport_scene.manipulator.set_arrows_show(checked))
                                # line color
                                ui.Spacer(width=2)
                                self._ui_show_arrows_color = ui.ColorWidget(0.0, 1.0, 1.0, 1.0, width=0, tooltip="Line color").model
                                for i, child in enumerate(self._ui_show_arrows_color.get_item_children()):
                                    item_model = self._ui_show_arrows_color.get_item_value_model(child)
                                    item_model.add_value_changed_fn(lambda m, i=i: self._viewport_scene.manipulator.set_arrows_color(i, m.get_value_as_float()))
                                # line thickness
                                ui.Spacer(width=12)
                                self._ui_show_arrows_thickness = ui.FloatDrag(height=LABEL_HEIGHT, min=0, max=1, alignment=ui.Alignment.LEFT_CENTER, 
                                                                              tooltip="Line thickness (relative)").model
                                self._ui_show_arrows_thickness.add_value_changed_fn(lambda m: self._viewport_scene.manipulator.set_arrows_thickness(m.as_float))
                                self._ui_show_arrows_thickness.set_value(0.1)
                                ui.Spacer(width=5)
                                add_line_rect_flourish()

                            with ui.HStack():
                                tooltip = "Frame transformation update frequency. 0 means to do so every update cycle"
                                ui.Label("Update Frequency (Hz):", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                self._ui_update_interval = ui.IntDrag(height=LABEL_HEIGHT, min=1, max=60, alignment=ui.Alignment.LEFT_CENTER, 
                                                                      tooltip="Frequency (Hz)").model
                                self._ui_update_interval.add_value_changed_fn(self._on_update_frequency_changed)
                                self._ui_update_interval.set_value(20)
                                ui.Spacer(width=5)
                                add_line_rect_flourish()

                            with ui.HStack():
                                tooltip = "The length of time, in seconds, before a frame that has not been updated is considered 'dead'"
                                ui.Label("Frame Timeout:", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                self._ui_frame_timeout = ui.IntDrag(height=LABEL_HEIGHT, min=1, max=60, alignment=ui.Alignment.LEFT_CENTER, 
                                                                    tooltip="Seconds").model
                                self._ui_frame_timeout.set_value(15)
                                ui.Spacer(width=5)
                                add_line_rect_flourish()

                            # reset tf
                            with ui.HStack():
                                btn = ui.Button("Reset",
                                                width=LABEL_WIDTH / 2,
                                                clicked_fn=self._on_reset,
                                                style=get_style(),
                                                alignment=ui.Alignment.LEFT_CENTER,
                                                tooltip="Reset transformation tree")
                                ui.Spacer(width=5)
                                add_line_rect_flourish(True)

                    self._tool = ui.CollapsableFrame(title="Measuring tool",
                                                     height=0,
                                                     collapsed=False,
                                                     style=get_style(),
                                                     style_type_name_override="CollapsableFrame",
                                                     horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                                     vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON)
                    with self._tool:
                        with ui.VStack(spacing=5, height=0):
                            
                            with ui.HStack():
                                items = sorted(self._frames)
                                tooltip = "Calculate the distance between two frames according to the transformation tree"
                                ui.Label("Frame A:", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                self._ui_distance_combo_box_1 = ui.ComboBox(0, *items, name="", width=ui.Fraction(1), 
                                                                            alignment=ui.Alignment.LEFT_CENTER).model
                                add_line_rect_flourish(False)

                            with ui.HStack():
                                items = sorted(self._frames)
                                tooltip = "Calculate the distance between two frames according to the transformation tree"
                                ui.Label("Frame B:", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                self._ui_distance_combo_box_1 = ui.ComboBox(0, *items, name="", width=ui.Fraction(1), 
                                                                            alignment=ui.Alignment.LEFT_CENTER).model
                                add_line_rect_flourish(False)

                            ui.Spacer(height=1)
                            with ui.HStack():
                                items = ["", "world", "map"]
                                tooltip = "Distance between the frames according to the transformation tree"
                                ui.Label("Distance:", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                ui.Label("TODO", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER)
                                add_line_rect_flourish()

                            ui.Spacer(height=1)
                            with ui.HStack():
                                items = ["", "world", "map"]
                                tooltip = "Rotation between the frames according to the transformation tree"
                                ui.Label("Rotation:", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                                ui.Label("TODO", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER)
                                add_line_rect_flourish()
