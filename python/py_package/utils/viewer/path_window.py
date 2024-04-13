import json

import numpy as np
import sapien
from sapien import internal_renderer as R

from .plugin import Plugin


class Path:
    def __init__(self, entity):
        self.entity = entity
        self.poses = []


class PathWindow(Plugin):
    def __init__(self):
        self.reset()

    def reset(self):
        self._paths = []
        self._current_path_index = -1
        self.ui_window = None
        self.file_chooser_load = None
        self.file_chooser_save = None
        self._selected_point_index = -1
        self._curve = None
        self._curve_time = 0
        self.curve_segments = 128

    @property
    def transform_window(self):
        from .transform_window import TransformWindow

        return next(p for p in self.viewer.plugins if isinstance(p, TransformWindow))

    @property
    def scene(self):
        return self.viewer.scene

    @property
    def paths(self):
        return [p.entity.name for p in self._paths]

    @paths.setter
    def paths(self, _):
        pass

    def add_path(self, _):
        entity = self.viewer.selected_entity
        if entity is None:
            return

        self._paths.append(Path(entity))
        self.current_path_index = len(self._paths) - 1

        self.add_point(None)

    def remove_path(self, _):
        if self._current_path_index < 0 or self._current_path_index >= len(self._paths):
            return

        del self._paths[self._current_path_index]

        # force refresh
        if self._current_path_index >= len(self._paths):
            self.current_path_index = len(self._paths) - 1
        else:
            self.current_path_index = self._current_path_index

    @property
    def current_path_index(self):
        return self._current_path_index

    @current_path_index.setter
    def current_path_index(self, p):
        self._current_path_index = p
        self.show_curve()

    @property
    def current_path(self) -> Path:
        if self._current_path_index < 0 or self._current_path_index >= len(self._paths):
            return None

        return self._paths[self.current_path_index]

    def add_point(self, _):
        path = self.current_path
        if path is None:
            print("no path selected")
            return

        if self.viewer.selected_entity != self.current_path.entity:
            print("selected entity is not path entity")
            return

        tw = self.transform_window
        if not tw.enabled:
            tw.enabled = True

        if path.poses:
            if np.linalg.norm(path.poses[-1].p - tw._gizmo_pose.p) < 1e-4:
                print("path points too close!!!")
                return

        path.poses.append(tw._gizmo_pose)
        self.show_curve()
        self.selected_point_index = len(path.poses) - 1

    def insert_point(self, _):
        path = self.current_path
        if path is None:
            print("no path selected")
            return

        if self.viewer.selected_entity != self.current_path.entity:
            print("selected entity is not path entity")
            return

        tw = self.transform_window
        if not tw.enabled:
            tw.enabled = True

        if path.poses:
            if (
                np.linalg.norm(
                    path.poses[self.selected_point_index].p - tw._gizmo_pose.p
                )
                < 1e-4
            ):
                print("path points too close!!!")
                return

        path.poses.insert(self.selected_point_index + 1, tw._gizmo_pose)
        self.show_curve()
        self.selected_point_index = self.selected_point_index + 1

    def set_point(self, _):
        path = self.current_path
        if path is None:
            print("no path selected")
            return

        if self.viewer.selected_entity != self.current_path.entity:
            print("selected entity is not path entity")
            return

        tw = self.transform_window
        if not tw.enabled:
            tw.enabled = True

        path.poses[self.selected_point_index] = tw._gizmo_pose
        self.show_curve()

    def del_point(self, _):
        path = self.current_path
        if path is None:
            print("no path selected")
            return

        if self.viewer.selected_entity != self.current_path.entity:
            print("selected entity is not path entity")
            return

        del path.poses[self.selected_point_index]
        self.selected_point_index = 0
        self.show_curve()

    @property
    def selected_point_index(self):
        return self._selected_point_index

    @selected_point_index.setter
    def selected_point_index(self, index):
        self._selected_point_index = index
        self.transform_window.gizmo_matrix = self.current_path.poses[
            index
        ].to_transformation_matrix()

    @property
    def points(self):
        if not self.current_path:
            return []

        return [f"Point {i}" for i in range(len(self.current_path.poses))]

    def get_curve(self, knots=128):
        from scipy.interpolate import splev, splprep
        from scipy.spatial.transform import Rotation, Slerp

        if self.current_path is None:
            return None, None, None

        points = np.array([pose.p for pose in self.current_path.poses])
        scipy_quats = np.array(
            [pose.q[[1, 2, 3, 0]] for pose in self.current_path.poses]
        )

        if len(points) <= 1:
            return None, None, None

        tck, u = splprep(
            [points[:, 0], points[:, 1], points[:, 2]], s=0, k=min(3, len(points) - 1)
        )
        ts = np.linspace(0, 1, knots)
        points = np.stack(splev(ts, tck), axis=-1)
        rots = Slerp(u, Rotation.from_quat(scipy_quats))(ts)
        quats = rots.as_quat()[:, [3, 0, 1, 2]]
        return ts, points, quats

    def eval_curve(self, ts):
        from scipy.interpolate import splev, splprep
        from scipy.spatial.transform import Rotation, Slerp

        if self.current_path is None:
            return None, None
        points = np.array([pose.p for pose in self.current_path.poses])
        scipy_quats = np.array(
            [pose.q[[1, 2, 3, 0]] for pose in self.current_path.poses]
        )
        if len(points) <= 1:
            return None, None
        tck, u = splprep(
            [points[:, 0], points[:, 1], points[:, 2]], s=0, k=min(3, len(points) - 1)
        )
        points = np.stack(splev(ts, tck), axis=-1)
        rots = Slerp(u, Rotation.from_quat(scipy_quats))(ts)
        quats = rots.as_quat()[:, [3, 0, 1, 2]]
        return points, quats

    def show_curve(self, _=None):
        if not self.viewer.render_scene:
            self._curve = None
            return

        self.hide_curve(_)

        ts, points, _ = self.get_curve(self.curve_segments)
        if points is None:
            return

        segs = []
        for p0, p1 in zip(points[:-1], points[1:]):
            segs += list(p0)
            segs += list(p1)
        colors = [0, 0, 1, 1] * int(len(segs) / 3)
        self._curve = self.viewer.render_scene.add_line_set(
            self.viewer.renderer_context.create_line_set(segs, colors)
        )

    def hide_curve(self, _):
        if not self.viewer.render_scene:
            return
        if self._curve is None:
            return

        self.viewer.render_scene.remove_node(self._curve)
        self._curve = None

    @property
    def curve_time(self):
        return self._curve_time

    @curve_time.setter
    def curve_time(self, t):
        self._curve_time = t
        ps, qs = self.eval_curve([t])
        if ps is None:
            return
        self.transform_window.gizmo_matrix = sapien.Pose(
            ps[0], qs[0]
        ).to_transformation_matrix()

    def save(self, _=None):
        self.file_chooser_save.open()

    def save_confirm(self, _, name, path):
        data = {
            "version": 0,
            "name": self.current_path.entity.name,
            "trajectory": [
                [float(x) for x in p.p] + [float(x) for x in p.q]
                for p in self.current_path.poses
            ],
        }
        assert name.endswith(".json")
        with open(name, "w") as f:
            json.dump(data, f)

    def load(self, _=None):
        self.file_chooser_load.open()

    def load_confirm(self, _, name, path):
        assert name.endswith(".json")
        with open(name, "r") as f:
            data = json.load(f)

        assert data["version"] == 0
        if data["name"] != self.current_path.entity.name:
            print("name does not match!")

        self.current_path.poses = [
            sapien.Pose(p[:3], p[3:]) for p in data["trajectory"]
        ]
        self.show_curve()

    def build(self):
        if self.scene is None:
            self.ui_window = None
            return

        if self.ui_window is None:
            self.file_chooser_save = R.UIFileChooser().Label("Save")
            self.file_chooser_load = R.UIFileChooser().Label("Load")
            self.ui_window = (
                R.UIWindow()
                .Label("Path")
                .append(
                    R.UISameLine().append(
                        R.UIOptions()
                        .Id("paths")
                        .Style("select")
                        .BindItems(self, "paths")
                        .BindIndex(self, "current_path_index"),
                        R.UIButton().Label("+").Width(40).Callback(self.add_path),
                        R.UIButton().Label("-").Width(40).Callback(self.remove_path),
                    ),
                    R.UIOptions()
                    .Style("radio")
                    .BindItems(self, "points")
                    .BindIndex(self, "selected_point_index"),
                    R.UIConditional()
                    .Bind(lambda: self.current_path is not None)
                    .append(
                        R.UISameLine().append(
                            R.UIButton().Label("Add Point").Callback(self.add_point),
                            R.UIButton()
                            .Label("Insert Point")
                            .Callback(self.insert_point),
                            R.UIButton().Label("Set Point").Callback(self.set_point),
                            R.UIButton().Label("Del Point").Callback(self.del_point),
                        ),
                        R.UISliderFloat()
                        .Label("t")
                        .Min(0)
                        .Max(1)
                        .Bind(self, "curve_time"),
                        R.UIInputInt().Label("Segments").Bind(self, "curve_segments"),
                        R.UISameLine().append(
                            R.UIButton().Label("Save").Callback(self.save),
                            self.file_chooser_save.Filter(".json").Callback(
                                self.save_confirm
                            ),
                            R.UIButton().Label("Load").Callback(self.load),
                            self.file_chooser_load.Filter(".json").Callback(
                                self.load_confirm
                            ),
                        ),
                    ),
                )
            )

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []

    def close(self):
        self.reset()
