//! # symbios-robot
//!
//! A sovereign interpretation crate for [Symbios](https://crates.io/crates/symbios) that translates
//! L-System grammars into engine-agnostic robotic blueprints.
//!
//! It decouples the *Genotype* (L-System String) from the *Phenotype* (Physics Simulation),
//! producing a `RobotBlueprint` structure that can be ingested by game engines (Bevy),
//! simulators (Gazebo), or physical manufacturing pipelines.

pub mod blueprint;
pub mod interpreter;
pub mod turtle;

pub use blueprint::*;
pub use interpreter::*;
pub use turtle::*;
