import sapyen
import sapyen_robot
import numpy as np
from robot.python.demonstration.movo_env import MOVOEnv
import transforms3d

PARTNET_DIR = "/home/sim/project/mobility_convex"


class DrawerTraditionalPipeline:
    def __init__(self, partnet_id: str):
        self.env = MOVOEnv(PARTNET_DIR, partnet_id)
        self.env.object.set_root_pose([2, 0, 0.5], [1, 0, 0, 0])
        self.mapping = self.env.semantic_mapping
        self.robot = self.env.robot
        self.obj = self.env.object
        self.manger = self.env.manger

        # Bind base function
        self.simulation_hz = self.env.simulation_hz

    def render_drawer_point_cloud(self):
        cloud, valid = self.env.render_point_cloud(0, rgba=True, segmentation=True)
        np.save("test_pc", cloud)


if __name__ == '__main__':
    import sys

    sapyen_robot.ros.init(sys.argv, "pipeline")
    partnet_id: str = "44826"
    pipeline = DrawerTraditionalPipeline(partnet_id)
    pipeline.env.renderer.show_window()
    # pipeline.env.move_camera_toward_semantic("drawer")
    for _ in range(100):
        pipeline.env.step()
    pipeline.env.translate_end_effector(np.array([0.2, 0, 0]))
    pipeline.env.close_gripper(1)
    pipeline.env.open_gripper(1)
    pipeline.env.apply_force_to_link("link_1", np.ones(6)*100)

    while True:
        pipeline.env.step()
