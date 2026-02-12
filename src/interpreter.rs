//! Interpreter that converts L-System symbols into a RobotBlueprint.

use crate::blueprint::{
    JointDefinition, JointLimit, JointType, ModuleId, RobotBlueprint, RobotModule, SensorMount,
    SensorType, ShapePrimitive,
};
use crate::turtle::{RobotOp, RobotTurtleState};
use glam::{Quat, Vec3};
use std::collections::HashMap;
use std::f32::consts::PI;
use symbios::{SymbiosState, SymbolTable};

/// Configuration for robot interpretation.
#[derive(Clone, Debug)]
pub struct RobotConfig {
    /// Default length/height for shapes if no parameter is provided.
    pub default_length: f32,
    /// Default width/radius for shapes.
    pub default_width: f32,
    /// Default density (kg/m^3) for calculating mass. Default: 1000 (Water/Plastic-ish).
    pub default_density: f32,
    /// Default rotation angle (in radians) for Yaw/Pitch/Roll.
    pub default_angle: f32,
    /// Maximum stack depth for push/pop operations.
    pub max_stack_depth: usize,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            default_length: 1.0,
            default_width: 0.2,
            default_density: 1000.0,
            default_angle: 45.0f32.to_radians(),
            max_stack_depth: 1024,
        }
    }
}

/// Interprets L-System output to build a RobotBlueprint.
pub struct RobotInterpreter {
    op_map: Vec<RobotOp>,
    config: RobotConfig,
}

impl RobotInterpreter {
    pub fn new(config: RobotConfig) -> Self {
        Self {
            op_map: Vec::new(),
            config,
        }
    }

    pub fn with_map(mut self, map: Vec<RobotOp>) -> Self {
        self.op_map = map;
        self
    }

    pub fn set_op(&mut self, sym_id: u16, op: RobotOp) {
        let idx = sym_id as usize;
        if idx >= self.op_map.len() {
            self.op_map.resize(idx + 1, RobotOp::Ignore);
        }
        self.op_map[idx] = op;
    }

    /// Populates standard robot symbols.
    pub fn populate_standard_symbols(&mut self, interner: &SymbolTable) {
        let mappings = [
            // Spatial
            ("f", RobotOp::Move),
            ("+", RobotOp::Yaw(1.0)),
            ("-", RobotOp::Yaw(-1.0)),
            ("&", RobotOp::Pitch(1.0)),
            ("^", RobotOp::Pitch(-1.0)),
            ("\\", RobotOp::Roll(1.0)),
            ("/", RobotOp::Roll(-1.0)),
            ("|", RobotOp::TurnAround),
            // Geometry
            ("B", RobotOp::SpawnBox),
            ("C", RobotOp::SpawnCylinder),
            ("O", RobotOp::SpawnSphere), // Orb
            ("K", RobotOp::SpawnCapsule),
            // Config
            ("!", RobotOp::SetWidth),
            ("'", RobotOp::SetMaterial), // Using ' like visual turtle
            // Joint Configuration
            ("J", RobotOp::SetJointType(JointType::Hinge)), // Default J is Hinge
            ("Jf", RobotOp::SetJointType(JointType::Fixed)),
            ("Jb", RobotOp::SetJointType(JointType::Ball)),
            ("Jl", RobotOp::SetJointLimits),
            // Sensors
            ("S", RobotOp::MountSensor(SensorType::Camera)), // Generic S
            ("Si", RobotOp::MountSensor(SensorType::IMU)),
            ("St", RobotOp::MountSensor(SensorType::Touch)),
            ("Sl", RobotOp::MountSensor(SensorType::Lidar)),
            // Flow
            ("[", RobotOp::Push),
            ("]", RobotOp::Pop),
        ];

        for (sym, op) in mappings {
            if let Some(id) = interner.resolve_id(sym) {
                self.set_op(id, op);
            }
        }
    }

    /// Builds a blueprint from the L-System state.
    pub fn build_blueprint(&self, state: &SymbiosState) -> RobotBlueprint {
        let mut blueprint = RobotBlueprint::default();
        let mut turtle = RobotTurtleState {
            width: self.config.default_width,
            ..Default::default()
        };
        let mut stack = Vec::new();

        // Track the world-space transform (Position, Rotation) of every module we spawn.
        // We need this to calculate relative anchor points for joints and sensors.
        let mut module_transforms: HashMap<ModuleId, (Vec3, Quat)> = HashMap::new();
        let mut next_module_id: ModuleId = 0;

        for i in 0..state.len() {
            let view = match state.get_view(i) {
                Some(v) => v,
                None => break,
            };

            let op = self
                .op_map
                .get(view.sym as usize)
                .unwrap_or(&RobotOp::Ignore);

            // Param helpers
            let p = |idx: usize, def: f32| -> f32 {
                view.params.get(idx).map(|&x| x as f32).unwrap_or(def)
            };
            let p0 = p(0, 0.0);

            match op {
                // --- SPATIAL ---
                RobotOp::Move => {
                    let len = p(0, self.config.default_length);
                    turtle.position += turtle.up() * len;
                }
                RobotOp::Yaw(s) => turtle
                    .rotate_local_z(p(0, self.config.default_angle.to_degrees()).to_radians() * s),
                RobotOp::Pitch(s) => turtle
                    .rotate_local_x(p(0, self.config.default_angle.to_degrees()).to_radians() * s),
                RobotOp::Roll(s) => turtle
                    .rotate_local_y(p(0, self.config.default_angle.to_degrees()).to_radians() * s),
                RobotOp::TurnAround => turtle.rotate_local_z(PI),

                // --- GEOMETRY ---
                RobotOp::SpawnBox
                | RobotOp::SpawnCylinder
                | RobotOp::SpawnSphere
                | RobotOp::SpawnCapsule => {
                    let id = next_module_id;
                    next_module_id += 1;

                    // 1. Determine Dimensions & Shape
                    // Default growth axis is Y (Up).
                    let (shape, height_axis_len) = match op {
                        RobotOp::SpawnBox => {
                            let len = p(0, self.config.default_length); // Y axis (Growth)
                            let wid = p(1, turtle.width); // X axis
                            let hgt = p(2, turtle.width); // Z axis
                            (
                                ShapePrimitive::Box(Vec3::new(wid / 2.0, len / 2.0, hgt / 2.0)),
                                len,
                            )
                        }
                        RobotOp::SpawnCylinder => {
                            let len = p(0, self.config.default_length);
                            let rad = p(1, turtle.width / 2.0);
                            (
                                ShapePrimitive::Cylinder {
                                    radius: rad,
                                    height: len,
                                },
                                len,
                            )
                        }
                        RobotOp::SpawnCapsule => {
                            let len = p(0, self.config.default_length);
                            let rad = p(1, turtle.width / 2.0);
                            (
                                ShapePrimitive::Capsule {
                                    radius: rad,
                                    height: len,
                                },
                                len,
                            )
                        }
                        RobotOp::SpawnSphere => {
                            let rad = p(0, turtle.width / 2.0);
                            (ShapePrimitive::Sphere(rad), rad * 2.0)
                        }
                        _ => unreachable!(),
                    };

                    // 2. Calculate World Transform of the new Module
                    // The module's pivot is at the bottom (0, -h/2, 0).
                    // The turtle is at the pivot point.
                    // So the module's CENTER is TurtlePos + (TurtleUp * h/2).
                    let module_center_pos =
                        turtle.position + (turtle.up() * (height_axis_len / 2.0));
                    let module_rotation = turtle.rotation;

                    // 3. Register Module
                    let volume = calculate_approx_volume(&shape);
                    blueprint.add_module(
                        id,
                        RobotModule {
                            shape,
                            mass: volume * self.config.default_density,
                            material_id: turtle.material_id,
                            sensors: Vec::new(),
                            transform: (module_center_pos, module_rotation),
                        },
                    );
                    module_transforms.insert(id, (module_center_pos, module_rotation));

                    // 4. Create Joint (if parent exists)
                    if let Some(parent_id) = turtle.current_module_id
                        && let Some((parent_pos, parent_rot)) = module_transforms.get(&parent_id)
                    {
                        // Anchor on Parent: Where is the Turtle relative to Parent Center?
                        // Transform (TurtlePos - ParentPos) into Parent Local Space.
                        let world_offset = turtle.position - *parent_pos;
                        let anchor_parent = parent_rot.inverse() * world_offset;

                        // Anchor on Child: The child's pivot is at its 'bottom' relative to its center.
                        let anchor_child = Vec3::new(0.0, -height_axis_len / 2.0, 0.0);

                        // Axis: Transform turtle's joint axis (usually X) into Parent Local Space
                        // Note: Axis is defined relative to the *joint frame*, which usually aligns with child?
                        // Simpler: Use the axis relative to the Parent.
                        // Turtle rotation represents the Child frame orientation relative to World.
                        // We need the axis in Parent Local Space.
                        // Global Axis = turtle.rotation * config.axis
                        // Local Axis = parent_rot.inverse() * Global Axis
                        let global_axis = turtle.rotation * turtle.joint_config.axis;
                        let local_axis = parent_rot.inverse() * global_axis;

                        blueprint.add_joint(JointDefinition {
                            parent_id,
                            child_id: id,
                            anchor_parent,
                            anchor_child,
                            joint_type: turtle.joint_config.joint_type,
                            axis: local_axis,
                            limits: turtle.joint_config.limits,
                        });
                    }

                    // 5. Advance Turtle
                    // Move the cursor to the 'top' of the new module (the distal end).
                    turtle.position += turtle.up() * height_axis_len;
                    turtle.current_module_id = Some(id);
                }

                // --- CONFIG ---
                RobotOp::SetJointType(t) => turtle.joint_config.joint_type = *t,
                RobotOp::SetJointLimits => {
                    // Params: min, max, effort, velocity
                    let a = p(0, -PI);
                    let b = p(1, PI);
                    // Mutation can jitter limits so min > max; swap to avoid Avian3D panic.
                    let (min, max) = if a <= b { (a, b) } else { (b, a) };
                    let effort = p(2, 100.0);
                    let vel = p(3, 10.0);
                    turtle.joint_config.limits = Some(JointLimit {
                        min,
                        max,
                        effort,
                        velocity: vel,
                    });
                }
                RobotOp::SetMaterial => turtle.material_id = p0 as u8,
                RobotOp::SetWidth => turtle.width = p(0, turtle.width),

                // --- SENSORS ---
                RobotOp::MountSensor(sensor_type) => {
                    if let Some(mod_id) = turtle.current_module_id
                        && let Some((mod_pos, mod_rot)) = module_transforms.get(&mod_id)
                    {
                        // Calculate relative transform from Module Center to Turtle Pos
                        let world_offset = turtle.position - *mod_pos;
                        let local_pos = mod_rot.inverse() * world_offset;
                        // Sensor orientation relative to module
                        let local_rot = mod_rot.inverse() * turtle.rotation;

                        if let Some(module) = blueprint.modules.get_mut(&mod_id) {
                            module.sensors.push(SensorMount {
                                sensor_type: match op {
                                    // Handle specific overrides if needed, or rely on enum
                                    _ => *sensor_type,
                                },
                                local_position: local_pos,
                                local_rotation: local_rot,
                            });
                        }
                    }
                }

                // --- FLOW ---
                RobotOp::Push => {
                    if stack.len() < self.config.max_stack_depth {
                        stack.push(turtle.clone());
                    }
                }
                RobotOp::Pop => {
                    if let Some(state) = stack.pop() {
                        turtle = state;
                    }
                }
                RobotOp::Ignore => {}
            }
        }

        blueprint
    }
}

fn calculate_approx_volume(shape: &ShapePrimitive) -> f32 {
    match shape {
        ShapePrimitive::Box(extents) => extents.x * 2.0 * extents.y * 2.0 * extents.z * 2.0,
        ShapePrimitive::Cylinder { radius, height } => PI * radius * radius * height,
        ShapePrimitive::Sphere(radius) => (4.0 / 3.0) * PI * radius.powi(3),
        ShapePrimitive::Capsule { radius, height } => {
            let cyl_vol = PI * radius * radius * height;
            let sphere_vol = (4.0 / 3.0) * PI * radius.powi(3);
            cyl_vol + sphere_vol
        }
    }
}
