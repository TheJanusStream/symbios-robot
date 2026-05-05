//! End-effector / TCP frame coverage (issue #5).
//!
//! Verifies that the `E` standard symbol pins a named EE frame to whichever
//! module the turtle is currently standing on, and that the recorded local
//! pose composes correctly with the module's world transform to recover the
//! turtle's world pose at declaration time.

use glam::{Quat, Vec3};
use symbios::{SymbiosState, SymbolTable};
use symbios_robot::{RobotConfig, RobotInterpreter};

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
fn six_dof_arm_with_single_ee() {
    // 6 hinge segments stacked Y-up, alternating axes (X, Z, X, Z, X, Z) to
    // ensure the EE survives a non-trivial kinematic chain. The EE marker is
    // placed at the very tip.
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let j = sym.resolve_id("J").unwrap();
    let ja = sym.resolve_id("Ja").unwrap();
    let e = sym.resolve_id("E").unwrap();

    let mut state = SymbiosState::new();
    // Helper sequence: B, set hinge axis, J, repeat 6 times.
    let segs = [Vec3::X, Vec3::Z, Vec3::X, Vec3::Z, Vec3::X, Vec3::Z];
    for axis in segs {
        state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap();
        state
            .push(ja, 0.0, &[axis.x as f64, axis.y as f64, axis.z as f64])
            .unwrap();
        state.push(j, 0.0, &[]).unwrap();
    }
    // Final segment to attach the EE to.
    state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap();
    // Mark EE id 0 at the turtle's current pose (top of last module).
    state.push(e, 0.0, &[0.0]).unwrap();

    let bp = interp.build_blueprint(&state);

    assert_eq!(
        bp.modules.len(),
        7,
        "6 hinges + 1 base + final tip = 7 boxes"
    );
    assert_eq!(bp.joints.len(), 6, "six hinges between seven modules");
    assert_eq!(bp.end_effectors.len(), 1, "one EE registered");

    let ee = bp.end_effector(0).expect("EE id 0 should be reachable");
    // The EE is pinned to the most recently spawned module, which has the
    // highest ID assigned during interpretation (6).
    assert_eq!(ee.module_id, 6);

    // EE pose composition check: world_pose = module_world * local_offset.
    // At declaration time the turtle was at the top of module 6 (1.0m above
    // the module's center along the chain). Without rotation, the EE's local
    // position should be (0, +0.5, 0) relative to the module's center.
    assert!(
        ee.local_position
            .abs_diff_eq(Vec3::new(0.0, 0.5, 0.0), 1e-5),
        "ee local_position was {:?}",
        ee.local_position
    );
    // No rotations were applied to the turtle along the chain (only axis swaps,
    // which do not move the turtle), so the EE rotation is identity.
    assert!(ee.local_rotation.abs_diff_eq(Quat::IDENTITY, 1e-5));
}

#[test]
fn multiple_ees_addressed_by_id() {
    // A two-arm body with one EE per arm, distinguished by id.
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let push = sym.resolve_id("[").unwrap();
    let pop = sym.resolve_id("]").unwrap();
    let plus = sym.resolve_id("+").unwrap();
    let minus = sym.resolve_id("-").unwrap();
    let e = sym.resolve_id("E").unwrap();

    let mut state = SymbiosState::new();
    state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap(); // torso
    // Left arm: branch, yaw +90, two segments, EE id 0
    state.push(push, 0.0, &[]).unwrap();
    state.push(plus, 0.0, &[90.0]).unwrap();
    state.push(b, 0.0, &[0.5, 0.05, 0.05]).unwrap();
    state.push(b, 0.0, &[0.5, 0.05, 0.05]).unwrap();
    state.push(e, 0.0, &[0.0]).unwrap();
    state.push(pop, 0.0, &[]).unwrap();
    // Right arm: branch, yaw -90, two segments, EE id 1
    state.push(push, 0.0, &[]).unwrap();
    state.push(minus, 0.0, &[90.0]).unwrap();
    state.push(b, 0.0, &[0.5, 0.05, 0.05]).unwrap();
    state.push(b, 0.0, &[0.5, 0.05, 0.05]).unwrap();
    state.push(e, 0.0, &[1.0]).unwrap();
    state.push(pop, 0.0, &[]).unwrap();

    let bp = interp.build_blueprint(&state);
    assert_eq!(bp.end_effectors.len(), 2);
    let left = bp.end_effector(0).expect("left EE");
    let right = bp.end_effector(1).expect("right EE");
    // The two EEs must be on different modules.
    assert_ne!(left.module_id, right.module_id);
}

#[test]
fn ee_without_current_module_is_dropped() {
    // Marking an EE before any module has been spawned must not panic and must
    // simply produce no entry — there is nothing to pin to.
    let (interp, sym) = standard_setup();
    let e = sym.resolve_id("E").unwrap();
    let mut state = SymbiosState::new();
    state.push(e, 0.0, &[0.0]).unwrap();
    let bp = interp.build_blueprint(&state);
    assert!(bp.end_effectors.is_empty());
}
