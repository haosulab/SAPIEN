"""Create an articulation (a tree of actors/links connected by joints)

Each actor in the articulation is also called link.
The robot is an instance of articulation.

Concepts:
    - Create an articulation
    - Control the articulation basically (builtin position and velocity controller)
        sapien.Articulation.set_qf, sapien.Joint.set_drive_velocity_target
    - sapien.Articulation.get_qpos, sapien.Articulation.get_qvel
"""

import sapien.core as sapien
from sapien.utils.viewer import Viewer
import numpy as np
import transforms3d


def create_car(
        scene: sapien.Scene,
        body_size=(1.0, 0.5, 0.25),
        tire_radius=0.15,
        joint_friction=0.0,
        joint_damping=0.0,
        density=1.0,
) -> sapien.Articulation:
    body_half_size = np.array(body_size) / 2
    shaft_half_size = np.array([tire_radius * 0.1, tire_radius * 0.1, body_size[2] * 0.1])
    rack_half_size = np.array([tire_radius * 0.1, body_half_size[1] * 2.0, tire_radius * 0.1])
    builder: sapien.ArticulationBuilder = scene.create_articulation_builder()

    # car body (root of the articulation)
    body: sapien.LinkBuilder = builder.create_link_builder()  # LinkBuilder is similar to ActorBuilder
    body.set_name('body')
    body.add_box_shape(half_size=body_half_size, density=density)
    body.add_box_visual(half_size=body_half_size, color=[0.8, 0.6, 0.4])

    # front steering shaft
    front_shaft = builder.create_link_builder(body)
    front_shaft.set_name('front_shaft')
    front_shaft.set_joint_name('front_shaft_joint')
    front_shaft.add_box_shape(half_size=shaft_half_size, density=density)
    front_shaft.add_box_visual(half_size=shaft_half_size, color=[0.6, 0.4, 0.8])
    # The x-axis of the joint frame is the rotation axis of a revolute joint.
    front_shaft.set_joint_properties(
        'revolute',
        limits=[[-np.deg2rad(15), np.deg2rad(15)]],  # joint limits (for each DoF)
        # parent_pose refers to the relative transformation from the parent frame to the joint frame
        parent_pose=sapien.Pose(
            p=[(body_half_size[0] - tire_radius), 0, -body_half_size[2]],
            q=transforms3d.euler.euler2quat(0, -np.deg2rad(90), 0)
        ),
        # child_pose refers to the relative transformation from the child frame to the joint frame
        child_pose=sapien.Pose(
            p=[0.0, 0.0, shaft_half_size[2]],
            q=transforms3d.euler.euler2quat(0, -np.deg2rad(90), 0)
        ),
        friction=joint_friction,
        damping=joint_damping,
    )

    # back steering shaft (not drivable)
    back_shaft = builder.create_link_builder(body)
    back_shaft.set_name('back_shaft')
    back_shaft.set_joint_name('back_shaft_joint')
    back_shaft.add_box_shape(half_size=shaft_half_size, density=density)
    back_shaft.add_box_visual(half_size=shaft_half_size, color=[0.6, 0.4, 0.8])
    back_shaft.set_joint_properties(
        'fixed',
        limits=[],
        parent_pose=sapien.Pose(
            p=[-(body_half_size[0] - tire_radius), 0, -body_half_size[2]],
            q=transforms3d.euler.euler2quat(0, -np.deg2rad(90), 0)
        ),
        child_pose=sapien.Pose(
            p=[0.0, 0.0, shaft_half_size[2]],
            q=transforms3d.euler.euler2quat(0, -np.deg2rad(90), 0)
        ),
        friction=joint_friction,
        damping=joint_damping,
    )

    # front wheels
    front_wheels = builder.create_link_builder(front_shaft)
    front_wheels.set_name('front_wheels')
    front_wheels.set_joint_name('front_gear')
    # rack
    front_wheels.add_box_shape(half_size=rack_half_size, density=density)
    front_wheels.add_box_visual(half_size=rack_half_size, color=[0.8, 0.4, 0.6])
    # left wheel
    front_wheels.add_sphere_shape(pose=sapien.Pose(p=[0.0, rack_half_size[1] + tire_radius, 0.0]),
                                  radius=tire_radius, density=density)
    front_wheels.add_sphere_visual(pose=sapien.Pose(p=[0.0, rack_half_size[1] + tire_radius, 0.0]),
                                   radius=tire_radius,
                                   color=[0.4, 0.6, 0.8])
    # right wheel
    front_wheels.add_sphere_shape(pose=sapien.Pose(p=[0.0, -(rack_half_size[1] + tire_radius), 0.0]),
                                  radius=tire_radius, density=density)
    front_wheels.add_sphere_visual(pose=sapien.Pose(p=[0.0, -(rack_half_size[1] + tire_radius), 0.0]),
                                   radius=tire_radius,
                                   color=[0.4, 0.6, 0.8])
    # gear
    front_wheels.set_joint_properties(
        'revolute',
        limits=[[-np.inf, np.inf]],
        parent_pose=sapien.Pose(
            p=[0.0, 0, -(shaft_half_size[2] + rack_half_size[2])],
            q=transforms3d.euler.euler2quat(0, 0, np.deg2rad(90))
        ),
        child_pose=sapien.Pose(
            p=[0.0, 0.0, 0.0],
            q=transforms3d.euler.euler2quat(0, 0, np.deg2rad(90))
        ),
        friction=joint_friction,
        damping=joint_damping,
    )

    # back wheels
    back_wheels = builder.create_link_builder(back_shaft)
    back_wheels.set_name('back_wheels')
    back_wheels.set_joint_name('back_gear')
    # rack
    back_wheels.add_box_shape(half_size=rack_half_size, density=density)
    back_wheels.add_box_visual(half_size=rack_half_size, color=[0.8, 0.4, 0.6])
    # left wheel
    back_wheels.add_sphere_shape(pose=sapien.Pose(p=[0.0, rack_half_size[1] + tire_radius, 0.0]),
                                 radius=tire_radius, density=density)
    back_wheels.add_sphere_visual(pose=sapien.Pose(p=[0.0, rack_half_size[1] + tire_radius, 0.0]),
                                  radius=tire_radius,
                                  color=[0.4, 0.6, 0.8])
    # right wheel
    back_wheels.add_sphere_shape(pose=sapien.Pose(p=[0.0, -(rack_half_size[1] + tire_radius), 0.0]),
                                 radius=tire_radius, density=density)
    back_wheels.add_sphere_visual(pose=sapien.Pose(p=[0.0, -(rack_half_size[1] + tire_radius), 0.0]),
                                  radius=tire_radius,
                                  color=[0.4, 0.6, 0.8])
    # gear
    back_wheels.set_joint_properties(
        'revolute',
        limits=[[-np.inf, np.inf]],
        parent_pose=sapien.Pose(
            p=[0.0, 0, -(shaft_half_size[2] + rack_half_size[2])],
            q=transforms3d.euler.euler2quat(0, 0, np.deg2rad(90))
        ),
        child_pose=sapien.Pose(
            p=[0.0, 0.0, 0.0],
            q=transforms3d.euler.euler2quat(0, 0, np.deg2rad(90))
        ),
        friction=joint_friction,
        damping=joint_damping,
    )

    car = builder.build()
    car.set_name('car')
    return car


def get_joints_dict(articulation: sapien.Articulation):
    joints = articulation.get_joints()
    joint_names =  [joint.name for joint in joints]
    assert len(joint_names) == len(set(joint_names)), 'Joint names are assumed to be unique.'
    return {joint.name: joint for joint in joints}


def parse_args():
    import argparse
    parser = argparse.ArgumentParser()

    parser.add_argument('--static-friction', default=10.0, type=float, help='static friction')
    parser.add_argument('--dynamic-friction', default=10.0, type=float, help='dynamic friction')
    parser.add_argument('--restitution', default=0.1, type=float, help='restitution (elasticity of collision)')
    parser.add_argument('--joint-friction', default=0.0, type=float, help='joint friction')
    parser.add_argument('--joint-damping', default=0.0, type=float,
                        help='joint damping (resistance proportional to joint velocity)')

    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    engine = sapien.Engine()
    renderer = sapien.VulkanRenderer()
    engine.set_renderer(renderer)

    scene_config = sapien.SceneConfig()
    scene_config.default_static_friction = args.static_friction
    scene_config.default_dynamic_friction = args.dynamic_friction
    scene_config.default_restitution = args.restitution
    scene = engine.create_scene(scene_config)
    scene.set_timestep(1 / 100.0)

    # ---------------------------------------------------------------------------- #
    # Build a toy car
    # ---------------------------------------------------------------------------- #
    scene.add_ground(altitude=0)  # Ground
    car = create_car(scene, joint_friction=args.joint_friction, joint_damping=args.joint_damping)
    car.set_pose(sapien.Pose(p=[0., 0., 1.0]))
    print('The dof of the articulation is', car.dof)
    # ---------------------------------------------------------------------------- #

    viewer = Viewer(renderer)
    viewer.set_scene(scene)

    viewer.set_camera_xyz(x=-12, y=0, z=15)
    viewer.set_camera_rpy(y=0, p=-np.arctan2(2, 2), r=0)
    viewer.window.set_camera_parameters(near=0.001, far=100, fovy=1)
    viewer.focus_actor(car)

    scene.set_ambient_light([0.5, 0.5, 0.5])
    rscene = scene.get_render_scene()
    rscene.add_shadow_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    # ---------------------------------------------------------------------------- #
    # Control a toy car
    # ---------------------------------------------------------------------------- #
    joints = get_joints_dict(car)
    print(joints.keys())
    joints['front_shaft_joint'].set_drive_property(stiffness=1000.0, damping=0.0)  # front shaft
    joints['front_gear'].set_drive_property(0.0, 1000.0)  # front gear
    joints['back_gear'].set_drive_property(0.0, 0.0)  # back gear

    position = 0.0  # position target of joints
    velocity = 0.0  # velocity target of joints
    steps = 0
    while not viewer.closed:
        if viewer.window.key_down('i'):  # accelerate
            velocity += 2.0
            print('velocity increases:', velocity)
            joints['front_gear'].set_drive_velocity_target(velocity)
        elif viewer.window.key_down('k'):  # brake
            velocity -= 2.0
            print('velocity decreases:', velocity)
            joints['front_gear'].set_drive_velocity_target(velocity)
        elif viewer.window.key_down('j'):  # left turn
            position += 1
            joints['front_shaft_joint'].set_drive_target(np.deg2rad(position))
            print('position increases:', position)
        elif viewer.window.key_down('l'):  # right turn
            position -= 1
            print('position decreases:', position)
            joints['front_shaft_joint'].set_drive_target(np.deg2rad(position))
        elif viewer.window.key_down('r'):  # reset
            position = 0
            velocity = 0.0
            joints['front_shaft_joint'].set_drive_target(position)
            joints['front_gear'].set_drive_velocity_target(velocity)

        car.set_qf(car.compute_passive_force(True, True, False))
        scene.step()
        scene.update_render()
        viewer.render()

        if steps % 100 == 0:
            print('Pose', car.get_pose())
            print('Joint positions', car.get_qpos())
            print('Joint velocities', car.get_qvel())
        steps += 1

    del scene


if __name__ == '__main__':
    main()
