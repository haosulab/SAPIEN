import unittest
import sapien
import numpy as np


def sapien_pose_to_opencv_extrinsic(sapien_pose_matrix: np.ndarray) -> np.ndarray:
    sapien2opencv = np.array(
        [
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    ex = sapien2opencv @ np.linalg.inv(sapien_pose_matrix)  # world -> camera

    return ex


class TestCamera(unittest.TestCase):
    def test_free_camera(self):
        scene = sapien.Scene()

        width = np.random.randint(10, 100)
        height = np.random.randint(10, 100)
        fovy = np.random.random()
        near = np.random.random()
        far = near + np.random.random()

        cam = scene.add_camera("", width, height, fovy, near, far)
        fovx = 2 * np.arctan(np.tan(fovy * 0.5) * width / height)

        self.assertEqual(cam.width, width)
        self.assertEqual(cam.height, height)
        self.assertTrue(np.allclose(cam.fovy, fovy))
        self.assertTrue(np.allclose(cam.near, near))
        self.assertTrue(np.allclose(cam.far, far))
        self.assertTrue(np.allclose(cam.fovx, fovx))
        self.assertTrue(np.allclose(cam.cx, width / 2))
        self.assertTrue(np.allclose(cam.cy, height / 2))
        self.assertTrue(np.allclose(cam.skew, 0.0))

        scene = None

    def test_full_camera(self):
        scene = sapien.Scene()

        width = np.random.randint(10, 100)
        height = np.random.randint(10, 100)
        fx = np.random.random()
        fy = np.random.random()
        near = np.random.random()
        far = near + np.random.random()
        skew = np.random.random()
        cx = width * np.random.random()
        cy = height * np.random.random()

        cam = scene.add_camera("", width, height, 1, near, far)
        cam.set_perspective_parameters(near, far, fx, fy, cx, cy, skew)

        self.assertTrue(
            np.allclose(
                cam.get_intrinsic_matrix(),
                np.array([[fx, skew, cx], [0, fy, cy], [0, 0, 1]]),
            )
        )
        self.assertTrue(
            np.allclose(
                cam.get_projection_matrix(),
                np.array(
                    [
                        [2 * fx / width, -2 * skew / width, -2 * cx / width + 1, 0],
                        [0, -2 * fy / height, -2 * cy / height + 1, 0],
                        [0, 0, -far / (far - near), -far * near / (far - near)],
                        [0, 0, -1, 0],
                    ]
                ),
            )
        )

        self.assertEqual(cam.width, width)
        self.assertEqual(cam.height, height)
        self.assertTrue(np.allclose(cam.fx, fx))
        self.assertTrue(np.allclose(cam.fy, fy))
        self.assertTrue(np.allclose(cam.cx, cx))
        self.assertTrue(np.allclose(cam.cy, cy))
        self.assertTrue(np.allclose(cam.near, near))
        self.assertTrue(np.allclose(cam.far, far))
        self.assertTrue(np.allclose(cam.skew, skew))
        self.assertTrue(np.allclose(cam.fovx, 2 * np.arctan(width / 2 / fx)))
        self.assertTrue(np.allclose(cam.fovy, 2 * np.arctan(height / 2 / fy)))

    def test_mounted_camera(self):
        scene = sapien.Scene()

        b = scene.create_actor_builder()
        actor = b.build_kinematic()
        actor.set_pose(
            sapien.Pose(
                [0.65066646, -0.12961331, -0.55165323],
                [0.57956909, 0.13343633, -0.74564517, 0.30051238],
            )
        )
        cam = scene.add_mounted_camera(
            "",
            actor,
            sapien.Pose(
                [0.79756622, 0.80361573, -0.64800556],
                [0.7275946, 0.09149435, -0.43826028, 0.51977189],
            ),
            64,
            64,
            1.96352,
            0.042,
            10.534,
        )

        scene.update_render()
        model = cam.get_model_matrix()
        proj = cam.get_projection_matrix()
        extrinsic = cam.get_extrinsic_matrix()
        gt_model = [
            [-0.25517476, 0.01123044, 0.9668297, 0.4855722],
            [-0.41660327, -0.90363145, -0.0994575, 1.0099831],
            [0.8725409, -0.42816347, 0.23526263, 0.06144172],
            [0.0, 0.0, 0.0, 1.0],
        ]
        gt_proj = [
            [0.6681608, 0.0, 0.0, 0.0],
            [0.0, -0.6681608, 0.0, 0.0],
            [0.0, 0.0, -1.004003, -0.04216813],
            [0.0, 0.0, -1.0, 0.0],
        ]
        gt_extrinsic = [
            [-0.25517473, -0.41660324, 0.8725409, 0.49105754],
            [-0.0112305, 0.90363157, 0.42816344, -0.9335064],
            [-0.96682966, 0.09945743, -0.2352626, 0.38347024],
        ]

        self.assertTrue(np.allclose(model, gt_model))
        self.assertTrue(np.allclose(proj, gt_proj))
        self.assertTrue(np.allclose(extrinsic, gt_extrinsic))

    # TODO: make more comprehensive
