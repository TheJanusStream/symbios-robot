//! # symbios-robot
//!
//! Engine-agnostic robot interpretation layer for [Symbios](https://crates.io/crates/symbios) L-Systems.
//!
//! This crate translates an L-System symbol sequence into a [`blueprint::RobotBlueprint`] —
//! a complete description of a robot's topology (rigid bodies, joints, sensors) that decouples
//! the *Genotype* (L-System string) from the *Phenotype* (physics simulation).
//!
//! ## Quick start
//!
//! ```rust,ignore
//! use symbios::{SymbiosState, SymbolTable};
//! use symbios_robot::{RobotConfig, RobotInterpreter};
//!
//! let mut interner = SymbolTable::new();
//! interner.intern("B").unwrap();
//!
//! let mut interpreter = RobotInterpreter::new(RobotConfig::default());
//! interpreter.populate_standard_symbols(&interner);
//!
//! let mut state = SymbiosState::new();
//! let b_id = interner.resolve_id("B").unwrap();
//! state.push(b_id, 0.0, &[1.0, 0.1, 0.1]).unwrap();
//!
//! let blueprint = interpreter.build_blueprint(&state);
//! assert_eq!(blueprint.modules.len(), 1);
//! ```
//!
//! ## Modules
//!
//! - [`blueprint`] — Engine-agnostic data: [`blueprint::RobotBlueprint`],
//!   [`blueprint::RobotModule`], [`blueprint::ShapePrimitive`],
//!   [`blueprint::JointDefinition`], [`blueprint::JointType`] /
//!   [`blueprint::JointTypeKind`], [`blueprint::AxisLimit`],
//!   [`blueprint::EndEffector`] (addressed by [`blueprint::EndEffectorId`]),
//!   [`blueprint::SensorMount`] / [`blueprint::SensorType`], and the
//!   [`blueprint::ModuleId`] / [`blueprint::MaterialId`] aliases.
//! - [`interpreter`] — [`interpreter::RobotInterpreter`] (with
//!   [`interpreter::RobotInterpreter::set_op`],
//!   [`interpreter::RobotInterpreter::with_map`], and
//!   [`interpreter::RobotInterpreter::populate_standard_symbols`]) plus
//!   [`interpreter::RobotConfig`].
//! - [`turtle`] — [`turtle::RobotTurtleState`], [`turtle::RobotOp`], and
//!   [`turtle::ActiveJointConfig`] (the per-step state the interpreter
//!   threads through the symbol stream).
//!
//! All public items in these modules are re-exported at the crate root.

pub mod blueprint;
pub mod interpreter;
pub mod turtle;

pub use blueprint::*;
pub use interpreter::*;
pub use turtle::*;
