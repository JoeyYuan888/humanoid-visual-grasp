# Application Entrypoints

`apps/` is the operator-facing layer of the project.

The current project goal is:

```text
transport -> navigation -> grasp -> place
```

Current validated operator entrypoints are split by business stage.

```text
apps/grasp/       Plastic-bag grasp and post-grasp OCR/QR entrypoints
apps/transport/   Two-hand box transport, transport navigation handoff, and return
apps/navigation/  Basic map waypoint navigation
apps/place/       Shelf AprilTag lock, box pull-out, bag drop, and box push-back
```

Daily operation should use the relevant stage entrypoint under `apps/`.
`tools/` contains reusable one-off utilities for calibration, capture,
debugging, and low-level checks.
