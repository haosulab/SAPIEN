from .MOVOEnv import MOVOEnv, transform2mat
import os

PARTNET_DIR = '/home/sim/project/mobility-v0-prealpha3/mobility_verified'


class MOVOSapienEnv(MOVOEnv):
    def __init__(self, partnet_id: str, root_pose=((3, 0, 0.5), (1, 0, 0, 0))):
        # Partnet object and semantic mapping
        super(MOVOSapienEnv, self).__init__()
        part_dir = os.path.join(PARTNET_DIR, partnet_id)
        urdf = os.path.join(part_dir, "mobility.urdf")
        self.loader.fix_loaded_object = True
        self.loader.balance_passive_force = False
        self.obj = self.loader.load(urdf)
        self.obj.set_root_pose(root_pose[0], root_pose[1])
        self.semantic_mapping = self.build_semantic_mapping(part_dir=part_dir)

    def build_semantic_mapping(self, part_dir: str):
        header = {'id2link': {}, 'id2semantic': {}, 'id2motion': {}}
        semantics = os.path.join(part_dir, 'semantics.txt')
        link2semantics = {}
        link2motion = {}
        semantics2link = {}
        with open(semantics, 'r') as f:
            for line in f:
                if line.strip():
                    link, motion, semantics = line.split()
                    link2semantics[link] = semantics
                    semantics2link[semantics] = link
                    link2motion[link] = motion

        for wrapper in [self.robot, self.obj]:
            header['id2link'].update(dict(zip(wrapper.get_link_ids(), wrapper.get_link_names())))
            id2name = header['id2link']
            header['id2semantic'].update(
                {i: link2semantics[id2name[i]] for i in id2name if id2name[i] in link2semantics})
            header['id2motion'].update(
                {i: link2motion[id2name[i]] for i in id2name if id2name[i] in link2motion})

        header.update({"link2semantic": link2semantics})
        header.update({"semantic2link": semantics2link})
        return header

    def move_camera_toward_semantic(self, semantic_name):
        links = self.obj.get_links()
        semantic_link = self.semantic_mapping['semantic2link']
        name = semantic_link[semantic_name]
        link_names = self.obj.get_link_names()
        link_index = link_names.index(name)

        # Move robot to the right place
        link_pose = transform2mat(links[link_index].get_global_pose())
        camera_pose = self.get_robot_link_global_pose("kinect2_color_optical_frame")
        relative_pose = link_pose[:3, 3] - camera_pose[:3, 3]
        link_pose[0, 3] -= 1.5
        self.move_robot_to_target_place(link_pose)

        # Move camera to the right place
        for _ in range(self.simulation_hz):
            self.head_controller.move_joint(["tilt_joint"], -0.5)
            self.step()
