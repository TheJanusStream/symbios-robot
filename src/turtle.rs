//! Turtle state and operations for robotic interpretation.

use crate::blueprint::{AxisLimit, JointType, JointTypeKind, MaterialId, ModuleId, SensorType};
use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

/// Configuration for the next joint to be created.
///
/// This acts as a "pen style" for physics. When the turtle spawns a new module attached
/// to an existing one, it uses these settings to create the connection.
///
/// `joint_type` already encodes the drive axis (for Hinge/Prismatic/Screw). The
/// separate `axis` field here is the *staging* axis: it is what subsequent
/// `SetJointType` ops apply when constructing a new variant, and what
/// `SetJointLimits` (single-axis form) tags the appended limit with. This
/// decouples "what direction the turtle is currently configured for" from
/// "what variant happens to be active right now".
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ActiveJointConfig {
    /// The mechanical type and (where applicable) drive axis of the connection.
    pub joint_type: JointType,

    /// Staging axis — used to populate axis-bearing variants and to tag
    /// per-axis limits when no axis is given explicitly. Defaults to `+X`.
    pub axis: Vec3,

    /// Per-axis physical limits accumulated for the next joint. Empty = unlimited.
    pub limits: Vec<AxisLimit>,
}

impl Default for ActiveJointConfig {
    fn default() -> Self {
        Self {
            joint_type: JointType::Fixed, // Default to rigid welding
            axis: Vec3::X,
            limits: Vec::new(),
        }
    }
}

/// The state of the Robot Builder Turtle.
///
/// Tracks position, orientation, and the topological context (which module we are currently extending).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotTurtleState {
    /// Current world-space position of the "cursor".
    pub position: Vec3,

    /// Current world-space orientation.
    pub rotation: Quat,

    /// The ID of the module (rigid body) the turtle is currently "standing on".
    /// If this is Some(id), the NEXT spawned module will be jointed to this one.
    pub current_module_id: Option<ModuleId>,

    /// Configuration for the next joint creation.
    pub joint_config: ActiveJointConfig,

    /// Current material ID for new modules.
    pub material_id: MaterialId,

    /// Current default width/radius for shapes (can be modified by `!`).
    pub width: f32,
}

impl Default for RobotTurtleState {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            current_module_id: None,
            joint_config: ActiveJointConfig::default(),
            material_id: 0,
            width: 0.1,
        }
    }
}

impl RobotTurtleState {
    /// Returns the turtle's local up direction (Y-axis) in world space.
    pub fn up(&self) -> Vec3 {
        self.rotation * Vec3::Y
    }

    /// Returns the turtle's local forward direction (Z-axis) in world space.
    pub fn forward(&self) -> Vec3 {
        self.rotation * Vec3::Z
    }

    /// Returns the turtle's local right direction (X-axis) in world space.
    pub fn right(&self) -> Vec3 {
        self.rotation * Vec3::X
    }

    /// Rotates the turtle around its local X axis by `angle` radians (Pitch).
    pub fn rotate_local_x(&mut self, angle: f32) {
        let rot = Quat::from_axis_angle(Vec3::X, angle);
        self.rotation *= rot;
    }

    /// Rotates the turtle around its local Y axis by `angle` radians (Roll).
    pub fn rotate_local_y(&mut self, angle: f32) {
        let rot = Quat::from_axis_angle(Vec3::Y, angle);
        self.rotation *= rot;
    }

    /// Rotates the turtle around its local Z axis by `angle` radians (Yaw).
    pub fn rotate_local_z(&mut self, angle: f32) {
        let rot = Quat::from_axis_angle(Vec3::Z, angle);
        self.rotation *= rot;
    }
}

/// Operations that can be performed by the robot turtle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RobotOp {
    // --- Spatial Navigation ---
    /// Move forward without spawning geometry (`f`).
    Move,
    /// Rotate around Z (`+`/`-`).
    Yaw(f32),
    /// Rotate around X (`&`/`^`).
    Pitch(f32),
    /// Rotate around Y (`\` / `/`).
    Roll(f32),
    /// Turn 180 degrees (`|`).
    TurnAround,

    // --- Geometry Spawning (The Body) ---
    /// Spawn a Box shape. Params: `(length, width, height)`.
    /// If params missing, uses `(default_step, width, width)`.
    SpawnBox,
    /// Spawn a Cylinder shape. Params: `(length, radius)`.
    SpawnCylinder,
    /// Spawn a Sphere shape. Params: `(radius)`.
    SpawnSphere,
    /// Spawn a Capsule shape. Params: `(length, radius)`.
    SpawnCapsule,

    // --- Configuration (The Physics) ---
    /// Set the kind of the NEXT joint to be created.
    ///
    /// Axis-bearing variants ([`JointType::Hinge`], [`JointType::Prismatic`],
    /// [`JointType::Screw`]) are populated from the turtle's staging
    /// [`ActiveJointConfig::axis`] at interpretation time, so the variant
    /// passed here can be a placeholder (`Vec3::X` is used by the standard
    /// symbol map).
    SetJointType(JointTypeKind),
    /// Set the staging joint axis. Params: `(x, y, z)`.
    SetJointAxis,
    /// Set the screw `pitch` (meters per revolution) for the next [`JointType::Screw`].
    /// Params: `(pitch)`.
    SetScrewPitch,
    /// Append one [`AxisLimit`] using the turtle's current staging axis.
    /// Params: `(min, max, effort, velocity)`.
    SetJointLimits,
    /// Append one [`AxisLimit`] with an explicit axis.
    /// Params: `(ax, ay, az, min, max, effort, velocity)`.
    AddAxisLimit,
    /// Clear any accumulated [`AxisLimit`]s from the staging joint config.
    ClearJointLimits,
    /// Set the Material ID for visual rendering. Params: `(material_id)`.
    SetMaterial,
    /// Set the default width/radius for subsequent shapes.
    SetWidth,

    // --- Attachments (The Senses) ---
    /// Mount a sensor at the current location.
    MountSensor(SensorType),
    /// Declare an end-effector / TCP frame at the turtle's current pose,
    /// pinned to the module the turtle is currently standing on.
    /// Params: `(ee_id)`.
    MarkEndEffector,

    // --- Flow Control ---
    /// Save the full turtle state onto the stack (`[`).
    Push,
    /// Restore the most recently pushed turtle state (`]`).
    Pop,
    /// No-op — symbol has no registered meaning.
    Ignore,
}
