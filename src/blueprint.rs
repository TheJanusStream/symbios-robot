use bevy_heavy::ComputeMassProperties3d;
use bevy_math::Isometry3d;
use bevy_math::bounding::{Aabb3d, Bounded3d, BoundingSphere, BoundingVolume};
use bevy_math::primitives::{Capsule3d, Cuboid, Cylinder, Sphere};
use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// A unique identifier for a robot module (rigid body).
/// Maps to L-System derivation steps or Turtle spawn indices.
pub type ModuleId = u16;

/// A generic material identifier referencing an external palette.
///
/// 16-bit to support large multi-material assemblies (sensor housings, fasteners,
/// PCB substrates, etc.) where the legacy 8-bit cap (256 entries) was restrictive.
pub type MaterialId = u16;

/// A unique identifier for a named end-effector / TCP frame.
pub type EndEffectorId = u16;

/// The complete, engine-agnostic definition of a robot's topology.
///
/// This structure represents the "Phenotype" generated from an L-System.
/// It contains a graph of rigid bodies (modules) connected by joints.
///
/// Marked `#[non_exhaustive]` so that new fields can be added without breaking
/// downstream code that matches or constructs this type.
#[non_exhaustive]
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RobotBlueprint {
    /// The ID of the root module (base of the robot).
    pub root_module: Option<ModuleId>,

    /// All rigid bodies in the robot, indexed by their unique ID.
    pub modules: HashMap<ModuleId, RobotModule>,

    /// All physical connections between modules.
    pub joints: Vec<JointDefinition>,

    /// Named end-effector / TCP frames declared on the robot.
    ///
    /// Multiple EEs are supported (e.g. left/right gripper). Each entry pins a
    /// pose offset to a specific module so IK and grasp planners can resolve a
    /// tool-center-point pose from joint angles.
    pub end_effectors: Vec<EndEffector>,
}

impl RobotBlueprint {
    /// Creates an empty blueprint with no modules or joints.
    pub fn new() -> Self {
        Self::default()
    }

    /// Inserts a module into the blueprint.
    ///
    /// The first module inserted is automatically set as [`root_module`](Self::root_module).
    /// If `id` already exists it is silently overwritten.
    pub fn add_module(&mut self, id: ModuleId, module: RobotModule) {
        if self.modules.is_empty() {
            self.root_module = Some(id);
        }
        self.modules.insert(id, module);
    }

    /// Appends a joint to the blueprint.
    pub fn add_joint(&mut self, joint: JointDefinition) {
        self.joints.push(joint);
    }

    /// Registers an end-effector frame on the blueprint.
    pub fn add_end_effector(&mut self, ee: EndEffector) {
        self.end_effectors.push(ee);
    }

    /// Looks up an end-effector by its numeric ID.
    pub fn end_effector(&self, id: EndEffectorId) -> Option<&EndEffector> {
        self.end_effectors.iter().find(|e| e.id == id)
    }

    /// Compute the axis-aligned bounding box of the entire robot
    /// after applying `rotation` to the blueprint's rest pose.
    pub fn aabb(&self, rotation: Quat) -> Aabb3d {
        let mut combined: Option<Aabb3d> = None;
        for module in self.modules.values() {
            let (pos, rot) = module.transform;
            let isometry = Isometry3d::new(rotation * pos, rotation * rot);
            let aabb = module.shape.to_bevy_primitive().aabb_3d(isometry);
            combined = Some(match combined {
                Some(c) => c.merge(&aabb),
                None => aabb,
            });
        }
        combined.unwrap_or(Aabb3d::new(Vec3::ZERO, Vec3::ZERO))
    }
}

/// A single rigid body segment of the robot.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotModule {
    /// The physical shape of this segment.
    pub shape: ShapePrimitive,

    /// Mass in kg, computed from shape volume and density via `bevy_heavy`.
    pub mass: f32,

    /// Density in kg/m³ used to derive mass properties.
    pub density: f32,

    /// Material ID for visual rendering (links to external palette).
    pub material_id: MaterialId,

    /// Sensors attached directly to this module.
    pub sensors: Vec<SensorMount>,

    /// Initial World Transform (Position, Rotation) for the Rest Pose.
    /// Essential for stable physics initialization.
    pub transform: (Vec3, Quat),
}

/// Supported geometric primitives for robot segments.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum ShapePrimitive {
    /// A box defined by half-extents (x, y, z).
    Box(Vec3),
    /// A cylinder defined by radius and height (aligned along Y axis).
    Cylinder { radius: f32, height: f32 },
    /// A sphere defined by radius.
    Sphere(f32),
    /// A capsule defined by radius and height (aligned along Y axis).
    Capsule { radius: f32, height: f32 },
}

/// A type-erased wrapper around `bevy_math` primitives.
///
/// Enables calling [`bevy_heavy::ComputeMassProperties3d`] and
/// [`bevy_math::bounding::Bounded3d`] on any [`ShapePrimitive`] variant through a
/// single enum dispatch. Obtain one via [`ShapePrimitive::to_bevy_primitive`].
#[derive(Clone, Copy, Debug)]
pub enum BevyPrimitive {
    Cuboid(Cuboid),
    Cylinder(Cylinder),
    Sphere(Sphere),
    Capsule(Capsule3d),
}

impl ComputeMassProperties3d for BevyPrimitive {
    fn mass(&self, density: f32) -> f32 {
        match self {
            Self::Cuboid(s) => s.mass(density),
            Self::Cylinder(s) => s.mass(density),
            Self::Sphere(s) => s.mass(density),
            Self::Capsule(s) => s.mass(density),
        }
    }

    fn unit_principal_angular_inertia(&self) -> Vec3 {
        match self {
            Self::Cuboid(s) => s.unit_principal_angular_inertia(),
            Self::Cylinder(s) => s.unit_principal_angular_inertia(),
            Self::Sphere(s) => s.unit_principal_angular_inertia(),
            Self::Capsule(s) => s.unit_principal_angular_inertia(),
        }
    }

    fn center_of_mass(&self) -> Vec3 {
        match self {
            Self::Cuboid(s) => s.center_of_mass(),
            Self::Cylinder(s) => s.center_of_mass(),
            Self::Sphere(s) => s.center_of_mass(),
            Self::Capsule(s) => s.center_of_mass(),
        }
    }
}

impl Bounded3d for BevyPrimitive {
    fn aabb_3d(&self, isometry: impl Into<Isometry3d>) -> Aabb3d {
        match self {
            Self::Cuboid(s) => s.aabb_3d(isometry),
            Self::Cylinder(s) => s.aabb_3d(isometry),
            Self::Sphere(s) => s.aabb_3d(isometry),
            Self::Capsule(s) => s.aabb_3d(isometry),
        }
    }

    fn bounding_sphere(&self, isometry: impl Into<Isometry3d>) -> BoundingSphere {
        match self {
            Self::Cuboid(s) => s.bounding_sphere(isometry),
            Self::Cylinder(s) => s.bounding_sphere(isometry),
            Self::Sphere(s) => s.bounding_sphere(isometry),
            Self::Capsule(s) => s.bounding_sphere(isometry),
        }
    }
}

impl ShapePrimitive {
    /// Convert to the corresponding `bevy_math` primitive for mass-property computation.
    pub fn to_bevy_primitive(self) -> BevyPrimitive {
        match self {
            Self::Box(half_extents) => BevyPrimitive::Cuboid(Cuboid {
                half_size: half_extents,
            }),
            Self::Cylinder { radius, height } => {
                BevyPrimitive::Cylinder(Cylinder::new(radius, height))
            }
            Self::Sphere(r) => BevyPrimitive::Sphere(Sphere::new(r)),
            Self::Capsule { radius, height } => {
                BevyPrimitive::Capsule(Capsule3d::new(radius, height))
            }
        }
    }
}

/// A kinematic connection between two modules.
///
/// The drive axis (for single-axis joints) is carried on the [`JointType`] payload
/// so it cannot be set on a joint that has no axis. Per-axis motion limits are
/// stored in [`JointDefinition::limits`] as a vector — empty means "unlimited",
/// a single entry covers a single-axis joint, and up to three entries can pin
/// the swing-twist constraints on a [`JointType::Ball`].
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointDefinition {
    /// The parent module (the one closer to the root).
    pub parent_id: ModuleId,

    /// The child module (the one attached to the parent).
    pub child_id: ModuleId,

    /// The anchor point on the parent module, in parent's local space.
    pub anchor_parent: Vec3,

    /// The anchor point on the child module, in child's local space.
    pub anchor_child: Vec3,

    /// The type of mechanical connection. Carries the drive axis (if any).
    pub joint_type: JointType,

    /// Per-axis physical limits. Empty = unlimited.
    pub limits: Vec<AxisLimit>,
}

/// Types of mechanical joints.
///
/// Variants carry the data that is *intrinsic* to the joint type:
/// - [`Fixed`](Self::Fixed) and [`Ball`](Self::Ball) have no drive axis.
/// - [`Hinge`](Self::Hinge), [`Prismatic`](Self::Prismatic), and [`Screw`](Self::Screw) carry a single axis in
///   the parent module's local space.
/// - [`Screw`](Self::Screw) additionally carries a `pitch` (meters of linear travel per
///   full revolution; positive = right-handed thread).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum JointType {
    /// Fixed connection (welded). No degrees of freedom.
    Fixed,
    /// Rotates around a single axis (e.g., knee, elbow).
    Hinge { axis: Vec3 },
    /// Ball and socket (3 rotational degrees of freedom).
    Ball,
    /// Slides along a single axis (linear actuator).
    Prismatic { axis: Vec3 },
    /// Helical joint: rotation about `axis` is coupled to translation along it
    /// by `pitch` meters per full revolution. Models lead screws and screw-driven
    /// linear stages.
    Screw { axis: Vec3, pitch: f32 },
}

impl JointType {
    /// Returns the drive axis for single-axis joints, or `None` for [`Fixed`](Self::Fixed) / [`Ball`](Self::Ball).
    pub fn axis(&self) -> Option<Vec3> {
        match self {
            Self::Fixed | Self::Ball => None,
            Self::Hinge { axis } | Self::Prismatic { axis } | Self::Screw { axis, .. } => {
                Some(*axis)
            }
        }
    }

    /// Returns the discriminant kind (no axis / pitch payload).
    pub fn kind(&self) -> JointTypeKind {
        match self {
            Self::Fixed => JointTypeKind::Fixed,
            Self::Hinge { .. } => JointTypeKind::Hinge,
            Self::Ball => JointTypeKind::Ball,
            Self::Prismatic { .. } => JointTypeKind::Prismatic,
            Self::Screw { .. } => JointTypeKind::Screw,
        }
    }
}

/// Tag enum naming a joint variant *without* axis/pitch payload.
///
/// Used by the turtle layer to decouple "which kind of joint do I want next"
/// (a static decision baked into a symbol mapping) from the axis/pitch values
/// that get filled in from the live turtle state when the joint is built.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum JointTypeKind {
    Fixed,
    Hinge,
    Ball,
    Prismatic,
    Screw,
}

/// Motion limits along (or about) a single axis.
///
/// For [`JointType::Hinge`] / [`JointType::Screw`] limits are in radians and the
/// `effort` is a torque (Nm). For [`JointType::Prismatic`] limits are in meters
/// and `effort` is a force (N). Multiple `AxisLimit`s can be attached to a
/// [`JointType::Ball`] to express swing-twist cones.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct AxisLimit {
    /// Axis (in parent local space) that this limit constrains.
    pub axis: Vec3,
    /// Minimum angle (radians) or distance (meters).
    pub min: f32,
    /// Maximum angle (radians) or distance (meters).
    pub max: f32,
    /// Maximum torque (Nm) or force (N) the motor can apply about/along this axis.
    pub effort: f32,
    /// Maximum velocity (rad/s or m/s) about/along this axis.
    pub velocity: f32,
}

/// A named end-effector / tool-center-point frame attached to a module.
///
/// IK solvers, grasp planners, and tooling pipelines consume this frame to
/// resolve "where the gripper / probe / nozzle actually is" relative to the
/// kinematic chain.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct EndEffector {
    /// Stable identifier (e.g. 0 = primary, 1 = secondary). Used by external
    /// tooling to address the frame.
    pub id: EndEffectorId,
    /// The module the EE is rigidly attached to.
    pub module_id: ModuleId,
    /// EE pose offset relative to the module's center (position + orientation).
    pub local_position: Vec3,
    pub local_rotation: Quat,
}

/// A sensor attachment point.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SensorMount {
    /// Type of sensor (Camera, Lidar, Touch, IMU).
    pub sensor_type: SensorType,

    /// Position relative to the module's center.
    pub local_position: Vec3,

    /// Orientation relative to the module.
    pub local_rotation: Quat,
}

/// The kind of sensor mounted on a module.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SensorType {
    /// RGB or depth camera.
    Camera,
    /// Laser range scanner.
    Lidar,
    /// Contact / pressure sensor.
    Touch,
    /// Inertial measurement unit (accelerometer + gyroscope).
    IMU,
    /// Ultrasonic distance sensor.
    Ultrasonic,
}
