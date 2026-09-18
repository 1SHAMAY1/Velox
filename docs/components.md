# 📦 Component Reference

Velox uses Plain-Old-Data (POD) component structures without virtual dispatch or inheritance overhead. All components reside in `src/physics/Components.h`.

---

## 🧲 Core Physics Components

| Component | Key Fields | Description |
| :--- | :--- | :--- |
| **`TransformComponent`** | `Position` ($x, y$)<br>`Rotation` (rad)<br>`Scale` ($x, y$) | World-space 2D transform and orientation. |
| **`RigidBodyComponent`** | `Mass`<br>`InverseMass`<br>`IsStatic`<br>`IsSleeping`<br>`AllowSleep` | Inertial properties. Setting `InverseMass = 0` (or `IsStatic = true`) makes the body immovable. Tracks sleep state for deactivation. |
| **`MovementComponent`** | `Velocity`<br>`AngularVelocity`<br>`Force`<br>`Torque`<br>`LinearDamping`<br>`AngularDamping`<br>`PrevPosition`<br>`PrevVelocity` | Linear/angular motion state and historical snapshots for XPBD integration and velocity derivation. |
| **`ColliderComponent`** | `Type` (`Circle`, `Box`, `Polygon`, `Chain`)<br>`Radius`<br>`BoxHalfExtents`<br>`Vertices`<br>`CenterOffset`<br>`IsSensor`<br>`GroupId` | Geometric collision data. `IsSensor` flags overlap detection without collision response. `GroupId` filters collisions between matched groups. |
| **`PhysicalMaterialComponent`** | `StaticFriction`<br>`DynamicFriction`<br>`Restitution`<br>`Density` | Surface physical traits governing friction and bounciness. |

---

## 🔗 Constraints & Joints

| Component | Key Fields | Description |
| :--- | :--- | :--- |
| **`JointComponent`** | `BodyA`, `BodyB`<br>`AnchorA`, `AnchorB`<br>`Length`<br>`Compliance`<br>`EnableMotor`, `MotorSpeed` | XPBD distance constraint maintaining fixed distance between two anchors with optional motor. |
| **`RevoluteJointComponent`** | `BodyA`, `BodyB`<br>`AnchorA`, `AnchorB`<br>`EnableLimits`, `LowerAngle`, `UpperAngle`<br>`EnableMotor`, `MotorSpeed`, `MaxMotorTorque` | Hinge joint allowing rotation around a shared point with angular limits and motor control. |
| **`PrismaticJointComponent`** | `BodyA`, `BodyB`<br>`AnchorA`, `AnchorB`<br>`Axis`<br>`EnableLimits`, `LowerLimit`, `UpperLimit`<br>`EnableMotor`, `MotorSpeed` | Slider joint constraining motion along a specified linear axis with translation limits. |
| **`GearJointComponent`** | `JointA`, `JointB`<br>`Ratio` | Transmits rotational/translational motion between two joints based on a gear ratio. |
| **`PulleyJointComponent`** | `BodyA`, `BodyB`<br>`GroundA`, `GroundB`<br>`LengthA`, `LengthB`, `TotalLength`<br>`Ratio` | Simulates a rope passing through two fixed overhead pulleys connecting two bodies. |
| **`SoftBodyComponent`** | `Type` (`Blob`, `ShapeMatched`)<br>`Nodes` (Entity list)<br>`RestArea`, `Compliance`<br>`Stiffness` | Deformable body simulation supporting volume-preserving blobs and elastic shape-matched bodies. |

---

## 🎮 Gameplay & Behaviour Components

| Component | Description |
| :--- | :--- |
| **`ForceFieldComponent`** | Applies radial forces (`Inward`, `Outward`, `Clockwise`, `AntiClockwise`) to nearby dynamic entities. |
| **`OscillationComponent`** | Moves static or kinematic platforms in a periodic sine-wave pattern along a designated axis. |
| **`RotationComponent`** | Drives constant rotational motion (kinematic turntable / spinning hazard). |
| **`ProjectileComponent`** | Automatically aligns entity rotation to its velocity vector (e.g., arrows, missiles) with configurable bounce factors. |
| **`PhysicsBehavior`** | Extensible base for decentralized custom user behaviors (e.g., buoyancy water simulation). |
