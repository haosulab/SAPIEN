from sapien.core import Pose, Engine, OptifuserController, OptifuserRenderer, VulkanRenderer, VulkanController
import sapien.core as sc
import xml.etree.ElementTree as ET
from transforms3d.euler import euler2quat
import os


def load_sapien_sdf(sdf_file, scene: sc.Scene):
    model_path = os.getenv('SAPIEN_MODEL_PATH')
    assert model_path, 'SAPIEN_MODEL_PATH environment variable is required'
    if model_path[-1] != '/':
        model_path += '/'
    sdf = ET.parse(sdf_file).getroot()
    world = sdf.find('world')
    actors = []
    for l in world.findall('light'):
        assert l.attrib['type'] == 'point'
        color = [float(x)/3.14 for x in l.find('diffuse').text.split()]
        position = [float(x) for x in l.find('pose').text.split()][:3]
        scene.add_point_light(position, color)
    for sdf_model in world.findall('model'):
        builder = scene.create_actor_builder()
        sdf_link = sdf_model.find('link')
        sdf_pose = sdf_model.find('pose')
        sdf_inertial = sdf_link.find('inertial')
        assert sdf_inertial is not None
        cs = sdf_link.findall('collision')
        vs = sdf_link.findall('visual')
        for col in cs:
            sdf_geom = col.find('geometry')
            sdf_mesh = sdf_geom.find('mesh')
            sdf_uri = sdf_mesh.find('uri')
            sdf_scale = sdf_mesh.find('scale')
            assert sdf_uri is not None and sdf_scale is not None
            filename = sdf_uri.text.replace('model://', model_path)
            scale = [float(x) for x in sdf_scale.text.strip().split()]
            assert len(scale) == 3
            assert os.path.isfile(filename), filename
            friction = float(col.find('surface').find('friction').find('ode').find('mu').text)
            assert friction == 0.5  # will all be 0.5
            builder.add_multiple_convex_shapes_from_file(filename, scale=scale)
        for v in vs:
            sdf_geom = v.find('geometry')
            sdf_mesh = sdf_geom.find('mesh')
            sdf_uri = sdf_mesh.find('uri')
            sdf_scale = sdf_mesh.find('scale')
            assert sdf_uri is not None and sdf_scale is not None
            filename = sdf_uri.text.replace('model://', model_path)
            scale = [float(x) for x in sdf_scale.text.strip().split()]
            assert len(scale) == 3
            assert os.path.isfile(filename), filename
            builder.add_visual_from_file(filename, scale=scale)
        sdf_mass = sdf_inertial.find('mass')
        sdf_pose = sdf_inertial.find('pose')
        sdf_inertia = sdf_inertial.find('inertia')
        assert sdf_mass is not None and sdf_pose is not None and sdf_inertia is not None
        mass = float(sdf_mass.text)
        xyzrpy = [float(x) for x in sdf_pose.text.strip().split()]
        assert len(xyzrpy) == 6
        ixx = float(sdf_inertia.find('ixx').text)
        iyy = float(sdf_inertia.find('iyy').text)
        izz = float(sdf_inertia.find('izz').text)
        ixy = float(sdf_inertia.find('ixy').text)
        ixz = float(sdf_inertia.find('ixz').text)
        iyz = float(sdf_inertia.find('iyz').text)
        assert ixy == ixz == iyz == 0
        builder.set_mass_and_inertia(mass, sc.Pose(xyzrpy[:3], euler2quat(*xyzrpy[3:])), [ixx, ixy, ixz])
        model_pose = sdf_model.find('pose')
        model = builder.build(name=sdf_model.attrib['name'])
        xyzrpy = [float(x) for x in model_pose.text.strip().split()]
        model.set_pose(sc.Pose(xyzrpy[:3], euler2quat(*xyzrpy[3:])))
        model.set_velocity([0, 0, 0])
        model.set_damping(1, 1)
        actors.append(model)
    return actors


if __name__ == '__main__':
    os.environ.update({
        "SAPIEN_MODEL_PATH": "/home/sim/project/iros_ws/src/simulator_for_manipulation_and_grasping/ocrtoc_materials/models"})
    sdf_file = '/home/sim/project/iros_ws/src/simulator_for_manipulation_and_grasping/ocrtoc_materials/scenes/1-1/input.world'
    engine = Engine()
    renderer = OptifuserRenderer()
    engine.set_renderer(renderer)
    controller = OptifuserController(renderer)
    config = sc.SceneConfig()
    config.enable_pcm = False
    config.default_restitution = 0
    config.default_dynamic_friction = 0.5
    config.default_static_friction = 0.5
    scene = engine.create_scene(config)
    load_sapien_sdf(sdf_file, scene)
    scene.add_ground(0)
    controller.set_current_scene(scene)
    controller.show_window()
    while not controller.should_quit:
        scene.step()
        scene.update_render()
        controller.render()
