from sapien.core import (
    Engine,
    PhysicalMaterial,
    Scene,
    SceneConfig,
    Actor,
    ActorStatic,
    ActorBuilder,
    LinkBuilder,
    ArticulationBuilder,
    Drive,
)
import pickle


class SerializedScene:
    """
    This class serializes Scene. Currently it only serializes physical (collision) information.
    """

    def __init__(self, scene: Scene):
        # Scene config
        self.serialized_config = pickle.dumps(scene.get_config())

        # Timestep
        self.timestep = scene.get_timestep()

        # Default physical material
        self.serialized_default_material = self.SerializedMaterial(
            scene.default_physical_material
        )
        default_material_id = scene.default_physical_material.get_id()

        # Other used materials
        self.serialized_materials = {}
        for actor in scene.get_all_actors():  # Save used physical materials of actors
            for collision_shape in actor.get_collision_shapes():
                material = collision_shape.get_physical_material()
                material_id = material.get_id()
                if (
                    material_id != default_material_id
                    and material_id not in self.serialized_materials
                ):
                    self.serialized_materials[material_id] = self.SerializedMaterial(
                        material
                    )

        for (
            articulation
        ) in (
            scene.get_all_articulations()
        ):  # Save used physical materials of articulations
            for link in articulation.get_links():
                for collision_shape in link.get_collision_shapes():
                    material = collision_shape.get_physical_material()
                    material_id = material.get_id()
                    if (
                        material_id != default_material_id
                        and material_id not in self.serialized_materials
                    ):
                        self.serialized_materials[
                            material_id
                        ] = self.SerializedMaterial(material)

        # Actors (excluding Links)
        self.serialized_actors = {}
        for actor in scene.get_all_actors():
            actor_data = {}
            actor_data["name"] = actor.get_name()
            actor_data["type"] = actor.type

            builder = actor.get_builder()
            if builder:
                actor_data["builder"] = self.SerializedActorBuilder(builder)
            else:  # Ground
                actor_data["builder"] = self.SerializedGroundBuilder(actor)

            actor_data["state"] = actor.pack()

            if actor.type == "dynamic":
                actor_data["dof_are_locked"] = actor.dof_are_locked()
                actor_data["linear_damping"] = actor.get_linear_damping()
                actor_data["angular_damping"] = actor.get_angular_damping()
                actor_data["ccd"] = actor.ccd

            if actor.type == "kinematic":
                actor_data["dof_are_locked"] = actor.dof_are_locked()

            self.serialized_actors[actor.get_id()] = actor_data

        # Articulations
        self.serialized_articulations = []
        for articulation in scene.get_all_articulations():
            articulation_data = {}
            articulation_data["name"] = articulation.get_name()
            articulation_data["type"] = articulation.type

            builder = articulation.get_builder()
            articulation_data["builder"] = self.SerializedArticulationBuilder(builder)

            if articulation.type == "dynamic":
                articulation_data["fixed"] = articulation.fixed
                articulation_data["state"] = articulation.pack()
                articulation_data["drive_state"] = articulation.pack_drive()
            elif articulation.type == "kinematic":
                articulation_data["qpos"] = articulation.get_qpos()
                articulation_data["root_pose"] = articulation.get_root_pose()

            # Other data of Links
            links = []
            for link in articulation.get_links():
                link_data = {}
                link_data["name"] = link.get_name()

                if link.type == "link":  # Non-kinematic
                    link_data["linear_damping"] = link.get_linear_damping()
                    link_data["angular_damping"] = link.get_angular_damping()
                    link_data["ccd"] = link.ccd

                links.append(link_data)

            articulation_data["links"] = links

            self.serialized_articulations.append(articulation_data)

        # Drives
        self.serialized_drives = []
        for drive in scene.get_all_drives():
            self.serialized_drives.append(self.SerializedDrive(drive))

    def deserialize(self, engine: Engine) -> Scene:
        # Initialize with scene config
        scene_config = pickle.loads(self.serialized_config)
        scene = engine.create_scene(scene_config)

        # Timestep
        scene.set_timestep(self.timestep)

        # Default physical material
        scene.default_physical_material = self.serialized_default_material.deserialize(
            engine
        )

        # Other used materials
        materials = {}
        for material_id, material in self.serialized_materials.items():
            materials[material_id] = material.deserialize(engine)

        # Actors (excluding Links)
        actors = {}
        for actor_id, actor_data in self.serialized_actors.items():
            builder = actor_data["builder"].deserialize(scene, materials)

            actor = None
            if actor_data["type"] == "dynamic":
                actor = builder.build()
                actor.lock_motion(*actor_data["dof_are_locked"])
                actor.set_damping(
                    actor_data["linear_damping"], actor_data["angular_damping"]
                )
                actor.ccd = actor_data["ccd"]
            elif actor_data["type"] == "kinematic":
                actor = builder.build_kinematic()
                actor.lock_motion(*actor_data["dof_are_locked"])
            elif actor_data["type"] == "static":
                actor = builder.build_static()

            actor.set_name(actor_data["name"])
            actor.unpack(actor_data["state"])

            actor.render_collision()

            actors[actor_id] = actor

        # Articulations
        for articulation_data in self.serialized_articulations:
            builder = articulation_data["builder"].deserialize(scene, materials)

            articulation = None
            if articulation_data["type"] == "dynamic":
                articulation = builder.build(fix_root_link=articulation_data["fixed"])
                articulation.unpack(articulation_data["state"])
                articulation.unpack_drive(articulation_data["drive_state"])
            elif articulation_data["type"] == "kinematic":
                articulation = builder.build_kinematic()
                articulation.set_qpos(articulation_data["qpos"])
                articulation.set_root_pose(articulation_data["root_pose"])

            articulation.set_name(articulation_data["name"])

            for i, link in enumerate(articulation.get_links()):
                link.set_name(articulation_data["links"][i]["name"])

                if link.type == "link":  # Non-kinematic
                    link.set_damping(
                        articulation_data["links"][i]["linear_damping"],
                        articulation_data["links"][i]["angular_damping"],
                    )
                    link.ccd = articulation_data["links"][i]["ccd"]

                link.render_collision()

        # Drives
        for serialized_drive in self.serialized_drives:
            serialized_drive.deserialize(scene, actors)

        return scene

    def dump_state_into(self, scene: Scene):
        """
        Update states (pose, velocity, etc.) of actors and articulations of scene according to data saved in
        SerializedScene. SerializedScene and scene must have the exact same set of actors and articulations
        (with same ids).

        :param: scene: Scene to be updated
        """
        for actor_id, src_actor_data in self.serialized_actors.items():
            dest_actor = scene.find_actor_by_id(actor_id)
            dest_actor.unpack(src_actor_data["state"])

        dest_arts = scene.get_all_articulations()
        for i, src_art_data in enumerate(self.serialized_articulations):
            dest_art = dest_arts[i]
            if dest_art.type == "dynamic":
                dest_art.unpack(src_art_data["state"])
                dest_art.unpack_drive(src_art_data["drive_state"])
            elif dest_art.type == "kinematic":
                dest_art.set_qpos(src_art_data["qpos"])
                dest_art.set_root_pose(src_art_data["root_pose"])

    def update_state_from(self, scene: Scene):
        """
        Update states (pose, velocity, etc.) of actors and articulations of SerializedScene according to data saved in
        scene. SerializedScene and scene must have the exact same set of actors and articulations (with same ids).

        :param: scene: Scene that SerializedScene is updated from
        """
        for src_actor in scene.get_all_actors():
            actor_id = src_actor.get_id()
            self.serialized_actors[actor_id]["state"] = src_actor.pack()

        for i, src_art in enumerate(scene.get_all_articulations()):
            dest_art_data = self.serialized_articulations[i]
            if src_art.type == "dynamic":
                dest_art_data["state"] = src_art.pack()
                dest_art_data["drive_state"] = src_art.pack_drive()
            elif src_art.type == "kinematic":
                dest_art_data["qpos"] = src_art.get_qpos()
                dest_art_data["root_pose"] = src_art.get_root_pose()

    class SerializedMaterial:
        def __init__(self, material: PhysicalMaterial):
            self.static_friction = material.get_static_friction()
            self.dynamic_friction = material.get_dynamic_friction()
            self.restitution = material.get_restitution()

        def deserialize(self, engine: Engine) -> PhysicalMaterial:
            material = engine.create_physical_material(
                self.static_friction, self.dynamic_friction, self.restitution
            )
            return material

    class GroundBuilder:
        def __init__(self, scene: Scene, altitude: float, material: PhysicalMaterial):
            self.scene = scene
            self.altitude = altitude
            self.material = material

        def build_static(self) -> ActorStatic:
            return self.scene.add_ground(altitude=self.altitude, material=self.material)

    class SerializedGroundBuilder:
        def __init__(self, ground: Actor):
            collision_shape = ground.get_collision_shapes()[0]
            self.altitude = collision_shape.get_local_pose().p[2]
            self.material_id = collision_shape.get_physical_material().get_id()

        def deserialize(
            self, scene: Scene, materials: dict
        ) -> "SerializedScene.GroundBuilder":
            return SerializedScene.GroundBuilder(
                scene, self.altitude, materials[self.material_id]
            )

    class SerializedActorBuilder:
        def __init__(self, builder: ActorBuilder):
            self.type_list = []
            self.filename_list = []
            self.scale_list = []
            self.radius_list = []
            self.length_list = []
            self.material_id_list = []
            self.pose_list = []
            self.density_list = []
            self.patch_radius_list = []
            self.min_patch_radius_list = []
            self.is_trigger_list = []
            for shape_record in builder.get_collisions():
                self.type_list.append(shape_record.type)
                self.filename_list.append(shape_record.filename)
                self.scale_list.append(shape_record.scale)
                self.radius_list.append(shape_record.radius)
                self.length_list.append(shape_record.length)
                if shape_record.material:
                    self.material_id_list.append(shape_record.material.get_id())
                else:
                    self.material_id_list.append(None)
                self.pose_list.append(shape_record.pose)
                self.density_list.append(shape_record.density)
                self.patch_radius_list.append(shape_record.patch_radius)
                self.min_patch_radius_list.append(shape_record.min_patch_radius)
                self.is_trigger_list.append(shape_record.is_trigger)

            self.use_density = builder.use_density
            self.mass = builder.mass
            self.c_mass_pose = builder.c_mass_pose
            self.inertia = builder.inertia
            self.collision_groups = builder.collision_groups

        def _update_builder(self, builder: ActorBuilder, materials: dict):
            for i in range(len(self.type_list)):
                material = None
                if self.material_id_list[i] is not None:
                    material = materials[self.material_id_list[i]]

                if self.type_list[i] == "Mesh":
                    builder.add_collision_from_file(
                        self.filename_list[i],
                        self.pose_list[i],
                        self.scale_list[i],
                        material,
                        self.density_list[i],
                        self.patch_radius_list[i],
                        self.min_patch_radius_list[i],
                        self.is_trigger_list[i],
                    )
                elif self.type_list[i] == "Meshes":
                    builder.add_multiple_collisions_from_file(
                        self.filename_list[i],
                        self.pose_list[i],
                        self.scale_list[i],
                        material,
                        self.density_list[i],
                        self.patch_radius_list[i],
                        self.min_patch_radius_list[i],
                        self.is_trigger_list[i],
                    )
                elif self.type_list[i] == "Box":
                    builder.add_box_collision(
                        self.pose_list[i],
                        self.scale_list[i],
                        material,
                        self.density_list[i],
                        self.patch_radius_list[i],
                        self.min_patch_radius_list[i],
                        self.is_trigger_list[i],
                    )
                elif self.type_list[i] == "Capsule":
                    builder.add_capsule_collision(
                        self.pose_list[i],
                        self.radius_list[i],
                        self.length_list[i],
                        material,
                        self.density_list[i],
                        self.patch_radius_list[i],
                        self.min_patch_radius_list[i],
                        self.is_trigger_list[i],
                    )
                elif self.type_list[i] == "Sphere":
                    builder.add_sphere_collision(
                        self.pose_list[i],
                        self.radius_list[i],
                        material,
                        self.density_list[i],
                        self.patch_radius_list[i],
                        self.min_patch_radius_list[i],
                        self.is_trigger_list[i],
                    )
                elif self.type_list[i] == "Nonconvex":
                    builder.add_nonconvex_collision_from_file(
                        self.filename_list[i],
                        self.pose_list[i],
                        self.scale_list[i],
                        material,
                        self.patch_radius_list[i],
                        self.min_patch_radius_list[i],
                        self.is_trigger_list[i],
                    )

            if not self.use_density:
                builder.set_mass_and_inertia(self.mass, self.c_mass_pose, self.inertia)

            builder.set_collision_groups(*self.collision_groups)

        def deserialize(self, scene: Scene, materials: dict) -> ActorBuilder:
            builder = scene.create_actor_builder()

            self._update_builder(builder, materials)

            return builder

    class SerializedLinkBuilder(SerializedActorBuilder):
        def __init__(self, builder: LinkBuilder):
            super().__init__(builder)

            joint_record = builder.get_joint()
            self.joint_name = joint_record.name
            self.joint_type = joint_record.joint_type
            self.joint_limits = joint_record.limits
            self.joint_pose_in_parent = joint_record.pose_in_parent
            self.joint_pose_in_child = joint_record.pose_in_child
            self.joint_friction = joint_record.friction
            self.joint_damping = joint_record.damping

            self.parent = builder.get_parent()
            self.name = builder.get_name()

        def deserialize(
            self, art_builder: ArticulationBuilder, materials: dict
        ) -> LinkBuilder:
            builder = art_builder.create_link_builder()

            super()._update_builder(builder, materials)

            builder.set_joint_properties(
                self.joint_type,
                self.joint_limits,
                self.joint_pose_in_parent,
                self.joint_pose_in_child,
                self.joint_friction,
                self.joint_damping,
            )

            builder.set_parent(self.parent)
            builder.set_name(self.name)

            return builder

    class SerializedArticulationBuilder:
        def __init__(self, builder: ArticulationBuilder):
            self.serialized_link_builders = []

            for link_builder in builder.get_link_builders():
                self.serialized_link_builders.append(
                    SerializedScene.SerializedLinkBuilder(link_builder)
                )

        def deserialize(self, scene: Scene, materials: dict) -> ArticulationBuilder:
            builder = scene.create_articulation_builder()

            for serialized_link_builder in self.serialized_link_builders:
                serialized_link_builder.deserialize(builder, materials)

            return builder

    class SerializedDrive:
        def __init__(self, drive: Drive):
            self.actor1_id = None
            self.actor2_id = None
            if drive.get_actor1():
                self.actor1_id = drive.get_actor1().get_id()
            if drive.get_actor2():
                self.actor2_id = drive.get_actor2().get_id()
            self.pose1 = drive.get_pose1()
            self.pose2 = drive.get_pose2()

            self.dof_states = drive.get_dof_states()
            self.x_limit = drive.get_x_limit()
            self.y_limit = drive.get_y_limit()
            self.z_limit = drive.get_z_limit()
            self.distance_limit = drive.get_distance_limit()
            self.x_twist_limit = drive.get_x_twist_limit()
            self.yz_cone_limit = drive.get_yz_cone_limit()
            self.yz_pyramid_limit = drive.get_yz_pyramid_limit()
            self.x_properties = drive.get_x_properties()
            self.y_properties = drive.get_y_properties()
            self.z_properties = drive.get_z_properties()
            self.x_twist_properties = drive.get_x_twist_properties()
            self.yz_swing_properties = drive.get_yz_swing_properties()
            self.slerp_properties = drive.get_slerp_properties()
            self.target = drive.get_target()
            self.target_velocity = drive.get_target_velocity()

        def deserialize(self, scene: Scene, actors: dict) -> Drive:
            actor1 = None
            actor2 = None
            if self.actor1_id is not None:
                actor1 = actors[self.actor1_id]
            if self.actor2_id is not None:
                actor2 = actors[self.actor2_id]

            drive = scene.create_drive(actor1, self.pose1, actor2, self.pose2)

            are_locked = []
            for state in self.dof_states:
                if state == "locked":
                    are_locked.append(True)
                else:
                    are_locked.append(False)

            drive.lock_motion(*are_locked)

            if self.dof_states[0] == "limited":
                drive.set_x_limit(*self.x_limit)

            if self.dof_states[1] == "limited":
                drive.set_y_limit(*self.y_limit)

            if self.dof_states[2] == "limited":
                drive.set_z_limit(*self.z_limit)

            if (
                self.dof_states[0] == "limited"
                and self.dof_states[1] == "limited"
                and self.dof_states[2] == "limited"
            ):
                drive.set_distance_limit(self.distance_limit)

            if self.dof_states[3] == "limited":
                drive.set_x_twist_limit(*self.x_twist_limit)

            if self.dof_states[4] == "limited" and self.dof_states[5] == "limited":
                drive.set_yz_cone_limit(*self.yz_cone_limit)
                drive.set_yz_pyramid_limit(*self.yz_pyramid_limit)

            drive.set_x_properties(**self.x_properties)
            drive.set_y_properties(**self.y_properties)
            drive.set_z_properties(**self.z_properties)
            drive.set_x_twist_properties(**self.x_twist_properties)
            drive.set_yz_swing_properties(**self.yz_swing_properties)
            drive.set_slerp_properties(**self.slerp_properties)

            drive.set_target(self.target)
            drive.set_target_velocity(*self.target_velocity)

            return drive
