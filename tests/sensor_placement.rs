//! Sensor placement on branched morphologies (issue #7).
//!
//! Verifies that sensors mounted at different turtle positions inside `[ ... ]`
//! branches end up on the correct modules and that their local poses round-trip
//! through serde.

use glam::Vec3;
use symbios::{SymbiosState, SymbolTable};
use symbios_robot::{RobotBlueprint, RobotConfig, RobotInterpreter, SensorType};

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
fn camera_imu_lidar_on_distinct_modules() {
    // Body with three branches:
    //   - head module with a Camera (S)  on top
    //   - chest module with an IMU (Si)  in the middle
    //   - back  module with a Lidar (Sl) at the rear
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let push = sym.resolve_id("[").unwrap();
    let pop = sym.resolve_id("]").unwrap();
    let plus = sym.resolve_id("+").unwrap();
    let minus = sym.resolve_id("-").unwrap();
    let pitch = sym.resolve_id("&").unwrap();
    let s_cam = sym.resolve_id("S").unwrap();
    let s_imu = sym.resolve_id("Si").unwrap();
    let s_lidar = sym.resolve_id("Sl").unwrap();
    let s_touch = sym.resolve_id("St").unwrap();
    let s_us = sym.resolve_id("Su").unwrap();

    let mut state = SymbiosState::new();
    state.push(b, 0.0, &[1.0, 0.2, 0.2]).unwrap(); // body (mod 0)

    // Head branch (mod 1) with camera
    state.push(push, 0.0, &[]).unwrap();
    state.push(b, 0.0, &[0.3, 0.1, 0.1]).unwrap();
    state.push(s_cam, 0.0, &[]).unwrap();
    state.push(pop, 0.0, &[]).unwrap();

    // Left arm (mod 2) with IMU mid-segment, ultrasonic at tip
    state.push(push, 0.0, &[]).unwrap();
    state.push(plus, 0.0, &[90.0]).unwrap();
    state.push(b, 0.0, &[0.5, 0.05, 0.05]).unwrap();
    state.push(s_imu, 0.0, &[]).unwrap();
    state.push(s_us, 0.0, &[]).unwrap();
    state.push(pop, 0.0, &[]).unwrap();

    // Right arm (mod 3) with touch at tip
    state.push(push, 0.0, &[]).unwrap();
    state.push(minus, 0.0, &[90.0]).unwrap();
    state.push(b, 0.0, &[0.5, 0.05, 0.05]).unwrap();
    state.push(s_touch, 0.0, &[]).unwrap();
    state.push(pop, 0.0, &[]).unwrap();

    // Tail (mod 4) with lidar
    state.push(push, 0.0, &[]).unwrap();
    state.push(pitch, 0.0, &[180.0]).unwrap();
    state.push(b, 0.0, &[0.4, 0.08, 0.08]).unwrap();
    state.push(s_lidar, 0.0, &[]).unwrap();
    state.push(pop, 0.0, &[]).unwrap();

    let bp = interp.build_blueprint(&state);
    assert_eq!(bp.modules.len(), 5, "body + 4 branches");
    assert_eq!(bp.joints.len(), 4);

    // Each sensor should land on the most recently spawned module in its branch.
    assert_sensor_on(&bp, 1, SensorType::Camera);
    assert_sensor_on(&bp, 2, SensorType::IMU);
    assert_sensor_on(&bp, 2, SensorType::Ultrasonic);
    assert_sensor_on(&bp, 3, SensorType::Touch);
    assert_sensor_on(&bp, 4, SensorType::Lidar);

    // The body itself has no sensors.
    assert!(bp.modules[&0].sensors.is_empty());
}

fn assert_sensor_on(bp: &RobotBlueprint, module_id: u16, kind: SensorType) {
    let module = bp
        .modules
        .get(&module_id)
        .unwrap_or_else(|| panic!("module {module_id} missing"));
    assert!(
        module.sensors.iter().any(|s| s.sensor_type == kind),
        "module {module_id} should have a {kind:?} sensor; got {:?}",
        module
            .sensors
            .iter()
            .map(|s| s.sensor_type)
            .collect::<Vec<_>>()
    );
}

#[test]
fn sensor_local_pose_recovers_world_position() {
    // Place a single sensor on a branched arm and verify that
    //   module_world_pos + module_world_rot * sensor.local_position
    // recovers the world position the turtle was at when `S` was processed.
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let push = sym.resolve_id("[").unwrap();
    let pop = sym.resolve_id("]").unwrap();
    let plus = sym.resolve_id("+").unwrap();
    let s_cam = sym.resolve_id("S").unwrap();

    let mut state = SymbiosState::new();
    state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap(); // body
    state.push(push, 0.0, &[]).unwrap();
    state.push(plus, 0.0, &[45.0]).unwrap();
    state.push(b, 0.0, &[0.5, 0.05, 0.05]).unwrap(); // arm tip (mod 1)
    state.push(s_cam, 0.0, &[]).unwrap();
    state.push(pop, 0.0, &[]).unwrap();

    let bp = interp.build_blueprint(&state);
    let arm = &bp.modules[&1];
    assert_eq!(arm.sensors.len(), 1);
    let cam = &arm.sensors[0];

    // Reconstruct sensor world position.
    let (mod_pos, mod_rot) = arm.transform;
    let world_sensor = mod_pos + mod_rot * cam.local_position;

    // The turtle was at the top of the arm (distal end) when it mounted the
    // camera. The arm started at the body top (0, 1, 0) and pointed along the
    // turtle's local +Y after a 45° yaw about Z. local_up = Rz(45) * Y.
    let expected_dir = glam::Quat::from_rotation_z(45.0_f32.to_radians()) * Vec3::Y;
    let expected = Vec3::new(0.0, 1.0, 0.0) + expected_dir * 0.5;
    assert!(
        world_sensor.abs_diff_eq(expected, 1e-5),
        "sensor world pos {world_sensor:?} != expected {expected:?}"
    );
}

#[test]
fn blueprint_serde_round_trip_preserves_sensors_and_ees() {
    // Whole-blueprint round-trip including sensors and end-effectors.
    let (interp, sym) = standard_setup();
    let b = sym.resolve_id("B").unwrap();
    let s_cam = sym.resolve_id("S").unwrap();
    let e = sym.resolve_id("E").unwrap();

    let mut state = SymbiosState::new();
    state.push(b, 0.0, &[1.0, 0.1, 0.1]).unwrap();
    state.push(s_cam, 0.0, &[]).unwrap();
    state.push(e, 0.0, &[7.0]).unwrap();

    let original = interp.build_blueprint(&state);
    let json = serde_json::to_string(&original).expect("serialize");
    let back: RobotBlueprint = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(back.modules.len(), original.modules.len());
    assert_eq!(back.end_effectors.len(), 1);
    assert_eq!(back.end_effector(7).map(|e| e.id), Some(7));
    assert_eq!(back.modules[&0].sensors.len(), 1);
}
