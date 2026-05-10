# symbios-robot

An engine-agnostic robot interpretation layer for [Symbios](https://crates.io/crates/symbios) L-Systems.

This crate translates L-System grammars into a `RobotBlueprint` — a complete description of a robot's topology (rigid bodies, joints, sensors) that can be ingested by game engines, physics simulators, or manufacturing pipelines. It decouples the *Genotype* (L-System string) from the *Phenotype* (physics simulation).

## Concepts

### Genotype → Phenotype

An L-System produces a sequence of symbols and parameters. `RobotInterpreter` walks that sequence, maintaining a **turtle** (a cursor with position, orientation, and state), and progressively builds a `RobotBlueprint`.

```text
L-System String  →  RobotInterpreter  →  RobotBlueprint
  "B J B(2)"            (turtle)           2 modules
                                           1 hinge joint
```

### The Turtle

The turtle state tracks:

- **Position / Rotation** — where the cursor is in world space
- **Current module** — the last rigid body spawned (used as the joint parent)
- **Active joint config** — type, axis, and limits for the *next* joint
- **Width** — the default radius/width for shapes

When a geometry symbol is interpreted (e.g. `B`, `C`, `O`, `K`), a new `RobotModule` is spawned at the turtle's current position, and the turtle advances to the distal end of the new segment. If a previous module exists, a `JointDefinition` connecting them is created automatically.

### The Blueprint

`RobotBlueprint` is a plain data structure — no engine dependencies. It contains:

- `modules`: a map of `ModuleId → RobotModule` (shape, mass, density, material, transform, sensors)
- `joints`: a list of `JointDefinition` (parent, child, anchors, type, per-axis limits)
- `end_effectors`: a list of named `EndEffector` frames (TCP / gripper / probe poses)
- `root_module`: the ID of the first module spawned (base of the kinematic chain)

`RobotBlueprint` is marked `#[non_exhaustive]` so future additions (new sensor
graphs, cabling, etc.) can be made without breaking downstream consumers.
Construct one via `RobotBlueprint::new()` / `Default` and the
`add_module` / `add_joint` / `add_end_effector` helpers — never with a struct
literal.

### Joint Types

The drive axis (and screw pitch) live *inside* the `JointType` variant, so it
is impossible to construct a joint that has both a "Fixed" type and a stray
axis floating around:

| Variant                              | DoF | Description                                                                   |
|--------------------------------------|-----|-------------------------------------------------------------------------------|
| `Fixed`                              | 0   | Welded — child is rigidly bonded to parent                                    |
| `Hinge { axis }`                     | 1   | Rotation about `axis` (knee, elbow, wheel hub)                                |
| `Ball`                               | 3   | Free rotation; constrain via per-axis limits                                  |
| `Prismatic { axis }`                 | 1   | Translation along `axis` (linear actuator, telescoping arm)                   |
| `Screw { axis, pitch }`              | 1   | Rotation about `axis` coupled to translation by `pitch` m/rev (lead screw)    |

Limits are stored as `Vec<AxisLimit>` — empty means unconstrained, single-axis
joints carry one entry, and `Ball` joints can carry up to three swing-twist
constraints.

### End-Effectors

Tool-center-point frames are declared with the `E` symbol (parameter: `ee_id`).
Each `EndEffector { id, module_id, local_position, local_rotation }` pins a
named frame to the module the turtle is currently standing on. Look up by id:

```rust,ignore
let gripper = blueprint.end_effector(0).expect("primary gripper");
```

Multiple EEs are supported (e.g. `0` = left gripper, `1` = right gripper).

## Usage

```rust
use symbios::{SymbiosState, SymbolTable};
use symbios_robot::{RobotConfig, RobotInterpreter};

// 1. Set up a symbol table and intern the symbols your grammar uses.
let mut interner = SymbolTable::new();
interner.intern("B").unwrap();  // Box segment
interner.intern("J").unwrap();  // Set joint to Hinge
interner.intern("+").unwrap();  // Yaw rotation

// 2. Create an interpreter and register standard symbol mappings.
let config = RobotConfig::default();
let mut interpreter = RobotInterpreter::new(config);
interpreter.populate_standard_symbols(&interner);

// 3. Build an L-System state (or derive one from a grammar).
let b_id = interner.resolve_id("B").unwrap();
let j_id = interner.resolve_id("J").unwrap();

let mut state = SymbiosState::new();
state.push(b_id, 0.0, &[1.0, 0.1, 0.1]).unwrap(); // base segment: length=1, width=0.1, depth=0.1
state.push(j_id, 0.0, &[]).unwrap();               // switch next joint to Hinge
state.push(b_id, 0.0, &[1.0, 0.1, 0.1]).unwrap(); // forearm segment

// 4. Interpret into a blueprint.
let blueprint = interpreter.build_blueprint(&state);

assert_eq!(blueprint.modules.len(), 2);
assert_eq!(blueprint.joints.len(), 1);
```

## Standard Symbol Mappings

The interpreter is built around a sparse `Vec<RobotOp>` indexed by `symbios`
symbol ID. Three ways to populate it:

- `RobotInterpreter::populate_standard_symbols(&interner)` — registers the
  conventional symbol→op mappings shown below for every symbol that has been
  interned. Symbols that were never interned are silently skipped.
- `RobotInterpreter::set_op(sym_id, op)` — assigns a single op; the map is
  grown as needed and gaps are filled with `RobotOp::Ignore`.
- `RobotInterpreter::with_map(map)` — replaces the entire op map in one shot
  (builder style); useful when you've precomputed the full mapping table.

Set-joint-type entries use `JointTypeKind` (a payload-free tag enum) so the
mapping table doesn't have to invent a placeholder axis or pitch — those are
filled in from the live turtle's `ActiveJointConfig` when the joint is built.

| Symbol | Operation                         | Parameters                                 |
|--------|-----------------------------------|--------------------------------------------|
| `f`    | Move forward (no geometry)        | `(length)`                                 |
| `+`    | Yaw +1× default angle             | `(angle_deg)` override                     |
| `-`    | Yaw −1× default angle             | `(angle_deg)` override                     |
| `&`    | Pitch +1× default angle           | `(angle_deg)` override                     |
| `^`    | Pitch −1× default angle           | `(angle_deg)` override                     |
| `\`    | Roll +1× default angle            | `(angle_deg)` override                     |
| `/`    | Roll −1× default angle            | `(angle_deg)` override                     |
| `\|`   | Turn around 180°                  | —                                          |
| `B`    | Spawn Box                         | `(length, width, depth)`                   |
| `C`    | Spawn Cylinder                    | `(length, radius)`                         |
| `O`    | Spawn Sphere                      | `(radius)`                                 |
| `K`    | Spawn Capsule                     | `(length, radius)`                         |
| `!`    | Set default width/radius          | `(width)`                                  |
| `'`    | Set material ID                   | `(material_id)`                            |
| `J`    | Set next joint → Hinge            | —                                          |
| `Jf`   | Set next joint → Fixed            | —                                          |
| `Jb`   | Set next joint → Ball             | —                                          |
| `Jp`   | Set next joint → Prismatic        | —                                          |
| `Js`   | Set next joint → Screw            | —                                          |
| `Ja`   | Set staging joint axis            | `(ax, ay, az)`                             |
| `Jh`   | Set screw pitch (helix)           | `(pitch_m_per_rev)`                        |
| `Jl`   | Append axis limit (current axis)  | `(min, max, effort, velocity)`             |
| `Jla`  | Append axis limit (explicit axis) | `(ax, ay, az, min, max, effort, velocity)` |
| `Jlc`  | Clear accumulated joint limits    | —                                          |
| `S`    | Mount Camera sensor               | —                                          |
| `Si`   | Mount IMU sensor                  | —                                          |
| `St`   | Mount Touch sensor                | —                                          |
| `Sl`   | Mount Lidar sensor                | —                                          |
| `Su`   | Mount Ultrasonic sensor           | —                                          |
| `E`    | Mark end-effector / TCP           | `(ee_id)`                                  |
| `[`    | Push turtle state                 | —                                          |
| `]`    | Pop turtle state                  | —                                          |

## Configuration

`RobotConfig` controls interpreter defaults:

| Field             | Default       | Description                                                        |
|-------------------|---------------|--------------------------------------------------------------------|
| `default_length`  | `1.0` m       | Segment length (along growth axis) when no parameter given         |
| `default_width`   | `0.2` m       | Initial turtle width / lateral extent when no parameter given      |
| `default_density` | `100.0` kg/m³ | Density used by `bevy_heavy` to derive each module's mass          |
| `default_angle`   | `45°`         | Rotation step for `+`, `-`, `&`, `^`, `\`, `/`                     |
| `max_stack_depth` | `1024`        | Maximum push/pop nesting depth (excess pushes are silently dropped)|

## Bounding Box

`RobotBlueprint::aabb(rotation)` computes the axis-aligned bounding box of the entire robot in its rest pose, optionally rotated by `rotation`.

```rust
use glam::Quat;
let aabb = blueprint.aabb(Quat::IDENTITY);
println!("Robot size: {:?}", aabb.half_size());
```

## Dependencies

- [`symbios`](https://crates.io/crates/symbios) — L-System engine providing `SymbiosState` and `SymbolTable`
- [`glam`](https://crates.io/crates/glam) — Math primitives (`Vec3`, `Quat`)
- [`bevy_math`](https://crates.io/crates/bevy_math) — Geometric primitives and bounding volume computation
- [`bevy_heavy`](https://crates.io/crates/bevy_heavy) — Mass property computation from shape geometry
- [`serde`](https://crates.io/crates/serde) — Serialization of the blueprint

## License

MIT
