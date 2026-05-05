//! Coverage for the new prismatic and screw joint variants (issue #2).
//!
//! The standard symbol table assigns:
//!   `Jp` -> Prismatic, `Js` -> Screw, `Ja` -> set staging axis,
//!   `Jh` -> set screw pitch (helix). These tests exercise each path.

use glam::Vec3;
use symbios::{SymbiosState, SymbolTable};
use symbios_robot::{JointType, RobotConfig, RobotInterpreter};

/// Intern every symbol the standard mapping cares about so we can drive
/// the interpreter purely through `populate_standard_symbols`.
fn standard_setup() -> (RobotInterpreter, SymbolTable) {
    let mut interner = SymbolTable::new();
    for sym in [
        "f", "+", "-", "&", "^", "\\", "/", "|", "B", "C", "O", "K", "!", "'", "J", "Jf", "Jb",
        "Jp", "Js", "Ja", "Jh", "Jl", "Jla", "Jlc", "S", "Si", "St", "Sl", "Su", "E", "[", "]",
    ] {
        interner.intern(sym).unwrap();
    }
    let mut interp = RobotInterpreter::new(RobotConfig::default());
    interp.populate_standard_symbols(&interner);
    (interp, interner)
}

#[test]
fn two_link_prismatic_chain() {
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let jp = sym.resolve_id("Jp").unwrap();
    let ja = sym.resolve_id("Ja").unwrap();
    let jl = sym.resolve_id("Jl").unwrap();

    // Base box, then set staging axis to +Y, switch joint type to Prismatic
    // with travel limits [0, 0.5]m, then spawn the second box.
    let mut state = SymbiosState::new();
    state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap();
    state.push(ja, 0.0, &[0.0, 1.0, 0.0]).unwrap();
    state.push(jp, 0.0, &[]).unwrap();
    state.push(jl, 0.0, &[0.0, 0.5, 200.0, 0.5]).unwrap();
    state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap();

    let bp = interp.build_blueprint(&state);
    assert_eq!(bp.modules.len(), 2);
    assert_eq!(bp.joints.len(), 1);

    let joint = &bp.joints[0];
    match joint.joint_type {
        JointType::Prismatic { axis } => {
            // Both turtle and parent are at identity rotation — staging axis
            // (0,1,0) survives the child-local -> parent-local transform.
            assert!(axis.abs_diff_eq(Vec3::Y, 1e-6));
        }
        other => panic!("expected Prismatic, got {other:?}"),
    }

    assert_eq!(joint.limits.len(), 1);
    let lim = &joint.limits[0];
    assert!(lim.axis.abs_diff_eq(Vec3::Y, 1e-6));
    assert_eq!(lim.min, 0.0);
    assert_eq!(lim.max, 0.5);
    assert_eq!(lim.effort, 200.0);
    assert_eq!(lim.velocity, 0.5);
}

#[test]
fn screw_joint_records_pitch() {
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let js = sym.resolve_id("Js").unwrap();
    let jh = sym.resolve_id("Jh").unwrap();

    // Base, then mark the next joint as a Screw with a 5mm/rev lead.
    let mut state = SymbiosState::new();
    state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap();
    state.push(js, 0.0, &[]).unwrap();
    state.push(jh, 0.0, &[0.005]).unwrap();
    state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap();

    let bp = interp.build_blueprint(&state);
    assert_eq!(bp.joints.len(), 1);

    match bp.joints[0].joint_type {
        JointType::Screw { axis, pitch } => {
            // Default staging axis is +X, untouched here.
            assert!(axis.abs_diff_eq(Vec3::X, 1e-6));
            assert!((pitch - 0.005).abs() < 1e-9);
        }
        other => panic!("expected Screw, got {other:?}"),
    }
}

#[test]
fn jh_promotes_non_screw_to_screw() {
    // Setting pitch on a non-Screw joint should promote the staging type to Screw,
    // using the staging axis. This makes the standard `Js Jh(p)` order optional.
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let jh = sym.resolve_id("Jh").unwrap();

    let mut state = SymbiosState::new();
    state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap();
    state.push(jh, 0.0, &[0.002]).unwrap();
    state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap();

    let bp = interp.build_blueprint(&state);
    match bp.joints[0].joint_type {
        JointType::Screw { pitch, .. } => assert!((pitch - 0.002).abs() < 1e-9),
        other => panic!("expected promoted Screw, got {other:?}"),
    }
}

#[test]
fn jointtype_helpers() {
    // Sanity-check the JointType helpers we expose for downstream consumers.
    assert!(JointType::Fixed.axis().is_none());
    assert!(JointType::Ball.axis().is_none());
    assert_eq!(JointType::Hinge { axis: Vec3::Z }.axis(), Some(Vec3::Z));
    assert_eq!(JointType::Prismatic { axis: Vec3::Y }.axis(), Some(Vec3::Y));
    assert_eq!(
        JointType::Screw {
            axis: Vec3::X,
            pitch: 0.01
        }
        .axis(),
        Some(Vec3::X)
    );
}

#[test]
fn screw_serde_round_trip() {
    // The new variants must round-trip through serde so downstream tooling
    // (URDF export, persistence) can rely on them.
    use symbios_robot::JointDefinition;
    let original = JointDefinition {
        parent_id: 0,
        child_id: 1,
        anchor_parent: Vec3::ZERO,
        anchor_child: Vec3::ZERO,
        joint_type: JointType::Screw {
            axis: Vec3::new(0.0, 0.0, 1.0),
            pitch: 0.0123,
        },
        limits: Vec::new(),
    };
    let json = serde_json::to_string(&original).expect("serialize");
    let back: JointDefinition = serde_json::from_str(&json).expect("deserialize");
    match back.joint_type {
        JointType::Screw { axis, pitch } => {
            assert_eq!(axis, Vec3::Z);
            assert!((pitch - 0.0123).abs() < 1e-9);
        }
        other => panic!("expected Screw after round-trip, got {other:?}"),
    }
}
