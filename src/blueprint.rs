use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// A unique identifier for a robot module (rigid body).
/// Maps to L-System derivation steps or Turtle spawn indices.
pub type ModuleId = u16;

/// A generic material identifier referencing an external palette.
pub type MaterialId = u8;

/// The complete, engine-agnostic definition of a robot's topology.
///
/// This structure represents the "Phenotype" generated from an L-System.
/// It contains a graph of rigid bodies (modules) connected by joints.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RobotBlueprint {
    /// The ID of the root module (base of the robot).
    pub root_module: Option<ModuleId>,

    /// All rigid bodies in the robot, indexed by their unique ID.
    pub modules: HashMap<ModuleId, RobotModule>,

    /// All physical connections between modules.
    pub joints: Vec<JointDefinition>,
    pub(crate) sensors: (),
}

impl RobotBlueprint {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_module(&mut self, id: ModuleId, module: RobotModule) {
        if self.modules.is_empty() {
            self.root_module = Some(id);
        }
        self.modules.insert(id, module);
    }

    pub fn add_joint(&mut self, joint: JointDefinition) {
        self.joints.push(joint);
    }
}

/// A single rigid body segment of the robot.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotModule {
    /// The physical shape of this segment.
    pub shape: ShapePrimitive,

    /// Mass in kg. If 0.0, physics engines should treat it as static or calculate from density.
    pub mass: f32,

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

/// A kinematic connection between two modules.
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

    /// The type of mechanical connection.
    pub joint_type: JointType,

    /// The axis of rotation/translation in the Parent's local space.
    pub axis: Vec3,

    /// Physical limits of the joint.
    pub limits: Option<JointLimit>,
}

/// Types of mechanical joints.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum JointType {
    /// Fixed connection (welded).
    Fixed,
    /// Rotates around a single axis (e.g., knee, elbow).
    Hinge,
    /// Ball and socket (3 degrees of freedom).
    Ball,
    /// Slides along a single axis (linear actuator).
    Prismatic,
}

/// Limits for a joint's motion.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct JointLimit {
    /// Minimum angle (radians) or distance (meters).
    pub min: f32,
    /// Maximum angle (radians) or distance (meters).
    pub max: f32,
    /// Maximum torque (Nm) or force (N) the joint motor can apply.
    pub effort: f32,
    /// Maximum velocity (rad/s or m/s).
    pub velocity: f32,
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

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum SensorType {
    Camera,
    Lidar,
    Touch,
    IMU,
    Ultrasonic,
}
