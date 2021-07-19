import sapien.core as sapien
import mplib
import numpy as np
from sapien.utils.viewer import Viewer

class PlanningDemo():
    def __init__(self):
        self.engine = sapien.Engine()
        self.renderer = sapien.VulkanRenderer()
        self.engine.set_renderer(self.renderer)

        scene_config = sapien.SceneConfig()
        self.scene = self.engine.create_scene(scene_config)
        self.scene.set_timestep(1 / 240.0)
        self.scene.add_ground(-0.8)
        physical_material = self.scene.create_physical_material(1, 1, 0.0)
        self.scene.default_physical_material = physical_material

        self.rscene = self.scene.get_renderer_scene()
        self.rscene.set_ambient_light([0.5, 0.5, 0.5])
        self.rscene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
        self.rscene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
        self.rscene.add_point_light([1, -2, 2], [1, 1, 1], shadow=True)
        self.rscene.add_point_light([-1, 0, 1], [1, 1, 1], shadow=True)

        self.viewer = Viewer(self.renderer)
        self.viewer.set_scene(self.scene)
        self.viewer.set_camera_xyz(x=1.2, y=0.25, z=0.4)
        self.viewer.set_camera_rpy(r=0, p=-0.4, y=2.7)

        # Robot
        # Load URDF
        loader: sapien.URDFLoader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        self.robot: sapien.Articulation = loader.load("../assets/robot/panda/panda.urdf")
        self.robot.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))

        # Set initial joint positions
        init_qpos = [0, 0.19634954084936207, 0.0, -2.617993877991494, 0.0, 2.941592653589793, 0.7853981633974483, 0, 0]
        self.robot.set_qpos(init_qpos)

        self.active_joints = self.robot.get_active_joints()
        for joint in self.active_joints:
            joint.set_drive_property(stiffness=1000, damping=200)

        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        self.table = builder.build_kinematic(name='table')
        self.table.set_pose(sapien.Pose([0.56, 0, - 0.025]))

        # boxes
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06], color=[1, 0, 0])
        self.red_cube = builder.build(name='red_cube')
        self.red_cube.set_pose(sapien.Pose([0.7, 0, 0.06]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.04, 0.04, 0.005])
        builder.add_box_visual(half_size=[0.04, 0.04, 0.005], color=[0, 1, 0])
        self.green_cube = builder.build(name='green_cube')
        self.green_cube.set_pose(sapien.Pose([0.4, 0.3, 0.005]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.05, 0.2, 0.1])
        builder.add_box_visual(half_size=[0.05, 0.2, 0.1], color=[0, 0, 1])
        self.blue_cube = builder.build(name='blue_cube')
        self.blue_cube.set_pose(sapien.Pose([0.55, 0, 0.1]))

        self.setup_planner()
    
    def setup_planner(self):
        link_names = [link.get_name() for link in self.robot.get_links()]
        joint_names = [joint.get_name() for joint in self.robot.get_active_joints()]
        self.planner = mplib.Planner(
            urdf="../assets/robot/panda/panda.urdf",
            srdf="../assets/robot/panda/panda.srdf",
            user_link_names=link_names,
            user_joint_names=joint_names,
            move_group="panda_hand",
            joint_vel_limits=np.ones(7),
            joint_acc_limits=np.ones(7))

    def follow_path(self, result):
        n_step = result['position'].shape[0]
        for i in range(n_step):  
            qf = self.robot.compute_passive_force(
                gravity=True, 
                coriolis_and_centrifugal=True)
            self.robot.set_qf(qf)
            for j in range(7):
                self.active_joints[j].set_drive_target(result['position'][i][j])
                self.active_joints[j].set_drive_velocity_target(result['velocity'][i][j])
            self.scene.step()
            if i % 4 == 0:
                self.scene.update_render()
                self.viewer.render()

    def open_gripper(self):
        for joint in self.active_joints[-2:]:
            joint.set_drive_target(0.4)
        for i in range(100): 
            qf = self.robot.compute_passive_force(
                gravity=True, 
                coriolis_and_centrifugal=True)
            self.robot.set_qf(qf)
            self.scene.step()
            if i % 4 == 0:
                self.scene.update_render()
                self.viewer.render()

    def close_gripper(self):
        for joint in self.active_joints[-2:]:
            joint.set_drive_target(0)
        for i in range(100):  
            qf = self.robot.compute_passive_force(
                gravity=True, 
                coriolis_and_centrifugal=True)
            self.robot.set_qf(qf)
            self.scene.step()
            if i % 4 == 0:
                self.scene.update_render()
                self.viewer.render()
        
    def add_point_cloud(self):
        import trimesh
        box = trimesh.creation.box([0.1, 0.4, 0.2])
        points, _ = trimesh.sample.sample_surface(box, 1000)
        points += [0.55, 0, 0.1]
        self.planner.update_point_cloud(points)
        return 

    def move_to_pose(self, pose):
        result = self.planner.plan_screw(pose, self.robot.get_qpos(), time_step=1/250,
                                        use_point_cloud=self.use_point_cloud, use_attach=self.use_attach)
        if result['status'] != "Success":
            result = self.planner.plan(pose, self.robot.get_qpos(), time_step=1/250,
                                        use_point_cloud=self.use_point_cloud, use_attach=self.use_attach)
            if result['status'] != "Success":
                print(result['status'])
                return -1 
        self.follow_path(result)
        return 0

    def demo(self, with_screw = True, use_point_cloud = True, use_attach = True):
        pickup_pose = [0.7, 0, 0.12, 0, 1, 0, 0]
        delivery_pose = [0.4, 0.3, 0.13, 0, 1, 0, 0]
        
        self.use_point_cloud = use_point_cloud
        if self.use_point_cloud:
            self.add_point_cloud()
        
        self.use_attach = False
        
        pickup_pose[2] += 0.2
        self.move_to_pose(pickup_pose) 
        self.open_gripper()
        pickup_pose[2] -= 0.12
        self.move_to_pose(pickup_pose)
        self.close_gripper()

        if use_attach:
            self.use_attach = True
            self.planner.update_attached_box([0.04, 0.04, 0.12], [0, 0, 0.14, 1, 0, 0, 0])

        pickup_pose[2] += 0.12
        self.move_to_pose(pickup_pose)
        delivery_pose[2] += 0.2
        self.move_to_pose(delivery_pose)
        delivery_pose[2] -= 0.12
        self.move_to_pose(delivery_pose)
        self.open_gripper()
        delivery_pose[2] += 0.12
        self.move_to_pose(delivery_pose)

if __name__ == '__main__':
    demo = PlanningDemo()
    demo.demo()
