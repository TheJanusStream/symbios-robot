//! Turtle state and operations for robotic interpretation.

use crate::blueprint::{JointLimit, JointType, MaterialId, ModuleId, SensorType};
use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

/// Configuration for the next joint to be created.
///
/// This acts as a "pen style" for physics. When the turtle spawns a new module attached
/// to an existing one, it uses these settings to create the connection.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ActiveJointConfig {
    /// The mechanical type of the connection (Hinge, Fixed, etc.).
    pub joint_type: JointType,

    /// The axis of rotation/translation relative to the parent's orientation.
    /// Defaults to X-axis (Pitch).
    pub axis: Vec3,

    /// Physical limits (angle, velocity, effort).
    pub limits: Option<JointLimit>,
}

impl Default for ActiveJointConfig {
    fn default() -> Self {
        Self {
            joint_type: JointType::Fixed, // Default to rigid welding
            axis: Vec3::X,
            limits: None,
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
    /// Set the type of the NEXT joint to be created.
    SetJointType(JointType),
    /// Set joint limits. Params: `(min, max, effort, velocity)`.
    SetJointLimits,
    /// Set the Material ID for visual rendering.
    SetMaterial,
    /// Set the default width/radius for subsequent shapes.
    SetWidth,

    // --- Attachments (The Senses) ---
    /// Mount a sensor at the current location.
    MountSensor(SensorType),

    // --- Flow Control ---
    /// Save the full turtle state onto the stack (`[`).
    Push,
    /// Restore the most recently pushed turtle state (`]`).
    Pop,
    /// No-op — symbol has no registered meaning.
    Ignore,
}
