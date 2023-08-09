# serialization guide of Types

```
struct Pose {
    q: Quat
    p: Vec3
}

struct PhysicalMaterial {
    staticFriction: float
    dynamicFriction: float
    restitution: float
}

struct CollisionShapeCommon {
    classId: uint64
    collisionGroups: int32[4]
    restOffset: float
    contactOffset: float
    patchRadius: float
    minPatchRadius: float
    isTrigger: int
    localPose: Pose
    materialId: id(Material)
}

struct CollisionShapePlane {
    common: CollisionShapeCommon
}

struct CollisionShapeBox {
    common: CollisionShapeCommon
    halfLengths: Vec3
}

struct CollisionShapeSphere {
    common: CollisionShapeCommon
    radius: float
}

struct CollisionShapeCapsule {
    common: CollisionShapeCommon
    radius: float
    halfLength: float
}

struct CollisionShapeConvexMesh {
    common: CollisionShapeCommon
    scale: Vec3
    geometryId: id(ConvexMeshGeometry)
}

struct CollisionShapeTriangleMesh {
    common: CollisionShapeCommon
    scale: Vec3
    geometryId: id(TriangleMeshGeometry)
}

struct Component {
    classId: uint64
}

struct RigidBaseComponent {
    base: Component
    collisionShapeCount: uint32
    collisionShapes: collisionShapes[]
}

struct RigidStaticComponent {
    base: RigidBaseComponent
}

struct RigidDynamicComponent {
    base: RigidBaseComponent
    mass: float
    inertia: Vec3
    cmassLocalPose: Pose
    autoComputeMass: int
    kinematic: int
    lockFlag: int
    linearVelocity: Vec3
    angularVelocity: Vec3
}

struct JointRecord {
    jointType: int
    parentPose: Pose
    childPose: Pose
    friction: float
    
    limitLow: float?
    limitHigh: float?
    stiffness: float?
    damping: float?
    maxForce: float?
    driveType: int?
    targetPosition: float?
    targetVelocity: float?
}

struct ArticulationLinkComponent {
    base: RigidBaseComponent
    mass: float
    inertia: Vec3
    cmassLocalPose: Pose
    autoComputeMass: int

    parent: id(ArticulationLinkComponent)
    joint: JointRecord
}

struct Articulation {
    linkCount: uint32
    linkIds: id(ArticulationLinkComponent)[]
    rootLinearVelocity: Vec3
    rootAngularVelocity: Vec3
    dof: uint32
    qpos: float[]
    qvel: float[]
}


```
