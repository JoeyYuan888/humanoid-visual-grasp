# Navigation Stage

Navigation is not implemented yet.

Planned responsibility:

```text
input : OCR/QR result from grasp stage
logic : destination selection, map/localization integration, navigation command
output: robot arrives at target area and reports arrival status
```

Keep navigation code under:

```text
apps/navigation/
```

Do not add navigation logic into the current grasp flow.
