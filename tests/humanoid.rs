//! Humanoid morphology fixture (issue #7).
//!
//! Topology:
//!   torso (1) → head (1)
//!         ├── left arm  (3 segments) ── EE 0 (left gripper)
//!         ├── right arm (3 segments) ── EE 1 (right gripper)
//!         ├── left leg  (4 segments)
//!         └── right leg (4 segments)
//!
//! Total: 1 torso + 1 head + 2 * 3 arm + 2 * 4 leg = 16 modules,
//!        15 joints (every module except the root has one parent joint),
//!        2 end-effectors.
//!
//! Branching is driven by `[ ... ]` push/pop pairs rooted on the torso so
//! each limb chains off the same parent. Each limb starts with a yaw or pitch
//! to point in its respective direction.

use glam::Quat;
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
fn humanoid_topology_and_aabb() {
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let j = sym.resolve_id("J").unwrap();
    let push = sym.resolve_id("[").unwrap();
    let pop = sym.resolve_id("]").unwrap();
    let plus = sym.resolve_id("+").unwrap();
    let minus = sym.resolve_id("-").unwrap();
    let pitch = sym.resolve_id("&").unwrap();
    let e = sym.resolve_id("E").unwrap();
    let turn = sym.resolve_id("|").unwrap();

    let arm_seg = |st: &mut SymbiosState| {
        st.push(j, 0.0, &[]).unwrap();
        st.push(b, 0.0, &[0.4, 0.06, 0.06]).unwrap();
    };
    let leg_seg = |st: &mut SymbiosState| {
        st.push(j, 0.0, &[]).unwrap();
        st.push(b, 0.0, &[0.5, 0.08, 0.08]).unwrap();
    };

    let mut state = SymbiosState::new();

    // --- Torso + head (chain) ---
    state.push(b, 0.0, &[0.6, 0.3, 0.2]).unwrap(); // torso
    state.push(j, 0.0, &[]).unwrap();
    state.push(b, 0.0, &[0.25, 0.15, 0.15]).unwrap(); // head

    // After torso+head we are at the top of the head. Pop back to the torso
    // top before branching limbs from there.
    // Use Push/Pop so each limb starts from the same anchor.
    // We need to "rewind" to torso-top: re-derive via a Push before the head.
    // Simpler approach: rebuild the state with Push *before* head so we can
    // Pop and have the torso as the current module.

    // To keep this clean, rebuild from scratch with the correct branching.
    let mut state = SymbiosState::new();
    state.push(b, 0.0, &[0.6, 0.3, 0.2]).unwrap(); // torso (current = 0)

    // Branch: head
    state.push(push, 0.0, &[]).unwrap();
    state.push(j, 0.0, &[]).unwrap();
    state.push(b, 0.0, &[0.25, 0.15, 0.15]).unwrap(); // head
    state.push(pop, 0.0, &[]).unwrap();

    // Branch: left arm — yaw left, then 3 short segments, then EE 0
    state.push(push, 0.0, &[]).unwrap();
    state.push(plus, 0.0, &[90.0]).unwrap();
    arm_seg(&mut state);
    arm_seg(&mut state);
    arm_seg(&mut state);
    state.push(e, 0.0, &[0.0]).unwrap();
    state.push(pop, 0.0, &[]).unwrap();

    // Branch: right arm — yaw right
    state.push(push, 0.0, &[]).unwrap();
    state.push(minus, 0.0, &[90.0]).unwrap();
    arm_seg(&mut state);
    arm_seg(&mut state);
    arm_seg(&mut state);
    state.push(e, 0.0, &[1.0]).unwrap();
    state.push(pop, 0.0, &[]).unwrap();

    // Branch: left leg — turn 180 (downward), pitch slightly outward
    state.push(push, 0.0, &[]).unwrap();
    state.push(turn, 0.0, &[]).unwrap();
    state.push(pitch, 0.0, &[10.0]).unwrap();
    leg_seg(&mut state);
    leg_seg(&mut state);
    leg_seg(&mut state);
    leg_seg(&mut state);
    state.push(pop, 0.0, &[]).unwrap();

    // Branch: right leg
    state.push(push, 0.0, &[]).unwrap();
    state.push(turn, 0.0, &[]).unwrap();
    state.push(pitch, 0.0, &[-10.0]).unwrap();
    leg_seg(&mut state);
    leg_seg(&mut state);
    leg_seg(&mut state);
    leg_seg(&mut state);
    state.push(pop, 0.0, &[]).unwrap();

    let bp = interp.build_blueprint(&state);

    // Module count: 1 torso + 1 head + 3+3 arms + 4+4 legs = 16
    assert_eq!(
        bp.modules.len(),
        16,
        "humanoid module count: got {}",
        bp.modules.len()
    );

    // Joint count = modules - 1 (every non-root module has exactly one parent
    // joint in a tree morphology).
    assert_eq!(bp.joints.len(), 15);

    // Two end-effectors, one per arm.
    assert_eq!(bp.end_effectors.len(), 2);
    assert!(bp.end_effector(0).is_some());
    assert!(bp.end_effector(1).is_some());

    // Sanity: AABB exists and is non-degenerate.
    let aabb = bp.aabb(Quat::IDENTITY);
    let extent = aabb.max - aabb.min;
    assert!(
        extent.length() > 0.5,
        "humanoid AABB should be substantial; got extent {extent:?}"
    );
}
