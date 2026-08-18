# Application Entrypoints

`apps/` is the operator-facing layer of the project.

The current project goal is:

```text
grasp -> transport -> navigation -> place
```

Only the `grasp` stage is implemented and validated today. `transport`,
navigation, and standalone place stages are reserved here so the project can
grow without mixing future code into the existing grasp scripts.

```text
apps/grasp/       Current validated visual grasping entrypoints
apps/transport/   Reserved for two-hand box transport
apps/navigation/  Reserved for navigation planning and execution
apps/place/       Reserved for standalone placement logic
```

Daily operation should use the entrypoints under `apps/grasp/`. `tools/`
contains reusable one-off utilities for calibration, capture, debugging, and
low-level checks.
