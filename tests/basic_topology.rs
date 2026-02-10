// tests/basic_topology.rs
use glam::Vec3;
use symbios::{SymbiosState, SymbolTable};
use symbios_robot::{JointType, RobotConfig, RobotInterpreter, RobotOp};

fn setup() -> (RobotInterpreter, SymbolTable) {
    let mut interner = SymbolTable::new();
    let mut interpreter = RobotInterpreter::new(RobotConfig::default());

    // Intern symbols
    interner.intern("B").unwrap(); // Box
    interner.intern("J").unwrap(); // Joint (Hinge)
    interner.intern("+").unwrap(); // Yaw

    // Map them
    interpreter.set_op(interner.resolve_id("B").unwrap(), RobotOp::SpawnBox);
    interpreter.set_op(
        interner.resolve_id("J").unwrap(),
        RobotOp::SetJointType(JointType::Hinge),
    );
    interpreter.set_op(interner.resolve_id("+").unwrap(), RobotOp::Yaw(1.0));

    (interpreter, interner)
}

#[test]
fn test_simple_arm_topology() {
    let (interpreter, interner) = setup();
    let b_id = interner.resolve_id("B").unwrap();
    let j_id = interner.resolve_id("J").unwrap();

    // Grammar: B(1) J B(1)
    // 1. Spawn Box 1 (Length 1.0). Turtle moves to (0, 1, 0).
    // 2. Set Joint to Hinge.
    // 3. Spawn Box 2 (Length 1.0). Attached to Box 1.

    let mut state = SymbiosState::new();
    state.push(b_id, 0.0, &[1.0, 0.1, 0.1]).unwrap(); // Base
    state.push(j_id, 0.0, &[]).unwrap(); // Config Joint
    state.push(b_id, 0.0, &[1.0, 0.1, 0.1]).unwrap(); // Forearm

    let blueprint = interpreter.build_blueprint(&state);

    // Assertions
    assert_eq!(blueprint.modules.len(), 2, "Should have 2 modules");
    assert_eq!(blueprint.joints.len(), 1, "Should have 1 joint");

    let joint = &blueprint.joints[0];

    // Topology check
    assert_eq!(joint.parent_id, 0);
    assert_eq!(joint.child_id, 1);
    assert_eq!(joint.joint_type, JointType::Hinge);

    // Geometric check (Direct Attachment Logic)
    // Parent (Mod 0) is Box(1.0 height). Center at (0, 0.5, 0).
    // Turtle was at (0, 1.0, 0) when Mod 1 was spawned.
    // Offset = TurtlePos - ParentCenter = (0, 0.5, 0).
    // In Parent Local Space (Identity Rot), anchor should be (0, 0.5, 0) (Top face).
    assert_eq!(joint.anchor_parent, Vec3::new(0.0, 0.5, 0.0));

    // Child (Mod 1) is Box(1.0 height).
    // Child Anchor is always bottom face relative to center: (0, -0.5, 0).
    assert_eq!(joint.anchor_child, Vec3::new(0.0, -0.5, 0.0));
}
