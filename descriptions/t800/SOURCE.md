# EngineAI T800 right-arm model provenance

The kinematic transforms, joint axes, limits, inertials and armatures in
`right_arm_fixed.xml` were transcribed from:

```text
engineai_robotics_native_sdk
commit 335c60e88772c26c7852d0abd6b3c7439037dd8f
assets/resource/robot/t800/xml/serial_links.xml
```

The visual OBJ meshes under `descriptions/engineai_t800/meshes/` come from
the same pinned SDK revision.  Only the torso and five right-arm meshes needed
by the fixed MCC harness are tracked here; the vendor's complete floating-base
model is intentionally not loaded by this harness.

The model is deliberately fixed at the T800 torso and contains only joints
J18--J22. It is an MCC estimator/IK harness, not a replacement whole-body T800
simulator. The `right_hand_contact` position currently uses the center of the
vendor model's right-wrist collision sphere. Measure and update it against the
physical dummy-hand pull point before quantitative real-hardware force tests.

The position-actuator stiffness and joint damping mirror the supplied
`extand_right_hand` motion plan:

```text
stiffness = [40, 40, 20, 40, 20]
damping   = [1, 1, 1, 1, 1]
```

The fixed-arm MCC harness uses zero gravity to represent the motion plan's
ideal `use_gravity_compensation: true` setting. The real backend does not
publish a gravity term—the factory `lower_body_balance` controller remains the
owner of gravity compensation—and establishes a static torque baseline only
after the preparation motion and settle window.
