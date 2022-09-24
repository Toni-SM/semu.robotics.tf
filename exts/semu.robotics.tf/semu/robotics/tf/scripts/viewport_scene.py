import omni.ui as ui
from omni.ui import scene as sc
from omni.ui import color as cl

from .view_manipulator import ViewManipulator


class ViewportScene():
    def __init__(self, viewport_window: ui.Window, ext_id: str) -> None:
        self._scene_view = None
        self._viewport_window = viewport_window

        # scene view frame
        with self._viewport_window.get_frame(ext_id):
            # scene view (default camera-model)
            self._scene_view = sc.SceneView()

            # add handlers into the scene view's scene
            with self._scene_view.scene:

                self.manipulator = ViewManipulator()

                transforms = {'finger_link_0': ([-0.049999999999821826, -0.03999999999762199, 1.3060000000000105], [0.0, 1.0341155355611996e-13, 3.1023466066582796e-13, 1.0]), 'gripper_link_0': ([-0.049999999999821826, -0.03999999999762199, 1.3060000000000105], [0.0, 1.0341155355611996e-13, 3.1023466066582796e-13, 1.0]), 'iiwa_link_ee': ([1.533593339215986e-13, 2.4091284522853584e-12, 1.306], [0.0, 1.0341155355611996e-13, 3.1023466066582796e-13, 1.0]), 'iiwa_link_7': ([1.440522941015478e-13, 2.409017429982896e-12, 1.261], [0.0, 1.0341155355611996e-13, 3.1023466066582796e-13, 1.0]), 'magnetic_link_0': ([1.9017384698758498e-13, 0.0015000000024091298, 1.4885], [0.0, 1.0341155355611996e-13, 3.1023466066582796e-13, 1.0]), 'magnetic_link_1': ([2.2001287714820424e-13, -0.04827587120511312, 1.4834455799170614], [0.13032092037274057, 1.4295971230470683e-13, 2.9411224854651516e-13, 0.9914718643074051]), 'finger_link_1': ([1.8128045338165477e-13, -0.018999999997590722, 1.3840000000000001], [0.21646548286577655, 1.6811478934686408e-13, 2.804940492955085e-13, 0.9762902717571686]), 'iiwa_link_0': ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]), 'iiwa_link_1': ([0.0, 0.0, 0.1575], [0.0, 0.0, 0.0, 1.0]), 'iiwa_link_2': ([0.0, 0.0, 0.36], [-7.312301077167311e-14, 0.7071067811848163, 0.7071067811882787, -7.312301077203115e-14]), 'iiwa_link_3': ([2.0710123116948267e-25, -1.00136565706066e-12, 0.5645], [0.0, -1.0341155355510721e-13, -1.0341155355561358e-13, -1.0]), 'iiwa_link_4': ([4.4570379582458296e-14, -1.00136565706066e-12, 0.7799999999999999], [-0.7071067811848163, -1.462460215440623e-13, -7.1612161887239275e-25, -0.7071067811882787]), 'iiwa_link_5': ([8.272924284410601e-14, -9.792167077193881e-14, 0.9645], [-1.0341155355460082e-13, -4.896583138958032e-12, -1.0, 2.0682310711122714e-13]), 'iiwa_link_6': ([1.272996224257025e-13, 2.01244576558679e-12, 1.18], [7.312301077238921e-14, -0.7071067811882787, -0.7071067811848163, 2.1936903231609349e-13])}
                relations = [('finger_link_0', 'gripper_link_0'), ('gripper_link_0', 'iiwa_link_ee'), ('iiwa_link_ee', 'iiwa_link_7'), ('iiwa_link_7', 'iiwa_link_6'), ('magnetic_link_0', 'finger_link_0'), ('magnetic_link_1', 'finger_link_1'), ('finger_link_1', 'gripper_link_0'), ('iiwa_link_0', 'world'), ('iiwa_link_1', 'iiwa_link_0'), ('iiwa_link_2', 'iiwa_link_1'), ('iiwa_link_3', 'iiwa_link_2'), ('iiwa_link_4', 'iiwa_link_3'), ('iiwa_link_5', 'iiwa_link_4'), ('iiwa_link_6', 'iiwa_link_5')]

                self.manipulator.update_transforms(transforms, relations)

            # register the scene view to get projection and view updates
            self._viewport_window.viewport_api.add_scene_view(self._scene_view)

    def __del__(self):
        self.destroy()

    def destroy(self):
        if self._scene_view:
            # empty the scene view
            self._scene_view.scene.clear()
            # un-register the scene view
            if self._viewport_window:
                self._viewport_window.viewport_api.remove_scene_view(self._scene_view)
        # remove references
        self._viewport_window = None
        self._scene_view = None
