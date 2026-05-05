//! Quadruped morphology fixture (issue #7).
//!
//! Topology:
//!   spine (1 module) with four legs branching off, each leg = 3 segments.
//!   Total: 1 + 4*3 = 13 modules, 12 joints, 0 end-effectors.
//!
//! Each leg starts with a unique combination of yaw/pitch so the AABB grows
//! along multiple axes (a degenerate quadruped that pointed all legs the same
//! way would still pass an `n` check — the AABB extent guard catches that).

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
fn quadruped_topology_and_aabb() {
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let j = sym.resolve_id("J").unwrap();
    let push = sym.resolve_id("[").unwrap();
    let pop = sym.resolve_id("]").unwrap();
    let plus = sym.resolve_id("+").unwrap();
    let minus = sym.resolve_id("-").unwrap();
    let pitch = sym.resolve_id("&").unwrap();

    // Each leg: 3 segments stacked end-to-end with hinges between them.
    fn leg(st: &mut SymbiosState, b: u16, j: u16) {
        for _ in 0..3 {
            st.push(j, 0.0, &[]).unwrap();
            st.push(b, 0.0, &[0.4, 0.06, 0.06]).unwrap();
        }
    }

    let mut state = SymbiosState::new();
    state.push(b, 0.0, &[1.0, 0.2, 0.2]).unwrap(); // spine

    // Four legs at different yaw/pitch combos. Each pair of (push, ops, pop)
    // restores the turtle to the spine top before branching the next limb.
    let configs = [
        (60.0_f64, 30.0_f64),
        (-60.0, 30.0),
        (60.0, -30.0),
        (-60.0, -30.0),
    ];
    for (yaw_deg, pitch_deg) in configs {
        state.push(push, 0.0, &[]).unwrap();
        let yaw_op = if yaw_deg >= 0.0 { plus } else { minus };
        state.push(yaw_op, 0.0, &[yaw_deg.abs()]).unwrap();
        state.push(pitch, 0.0, &[pitch_deg]).unwrap();
        leg(&mut state, b, j);
        state.push(pop, 0.0, &[]).unwrap();
    }

    let bp = interp.build_blueprint(&state);

    assert_eq!(bp.modules.len(), 1 + 4 * 3, "spine + 4 legs of 3 segments");
    assert_eq!(bp.joints.len(), 12);

    // Every leg's first joint should reference the spine (module 0) as parent.
    let parents_to_spine = bp.joints.iter().filter(|j| j.parent_id == 0).count();
    assert_eq!(
        parents_to_spine, 4,
        "exactly 4 joints should branch off the spine"
    );

    // AABB should extend in three dimensions, not flatten along one axis.
    let aabb = bp.aabb(Quat::IDENTITY);
    let extent = aabb.max - aabb.min;
    assert!(extent.x > 0.2, "x extent: {}", extent.x);
    assert!(extent.y > 0.2, "y extent: {}", extent.y);
    assert!(extent.z > 0.2, "z extent: {}", extent.z);
}

#[test]
fn quadruped_root_module_is_spine() {
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let mut state = SymbiosState::new();
    state.push(b, 0.0, &[1.0, 0.2, 0.2]).unwrap();
    let bp = interp.build_blueprint(&state);
    assert_eq!(bp.root_module, Some(0));
}
