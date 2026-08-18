# Tools

`tools/` contains engineering helpers only: debug scripts, calibration tools,
pose/sample capture tools, and maintenance checks.

Daily operation should prefer the stable wrappers in `apps/grasp/`:

```text
apps/grasp/run_grasp_flow.py
apps/grasp/run_perception_lock.py
apps/grasp/run_visual_grasp_test.py
apps/grasp/run_post_grasp_qr_scan.py
apps/grasp/test_yolo.py
```

Do not use `tools/` as the normal operator entrypoint. Use these scripts only
when debugging one subtask or collecting calibration/capture data.

Tool categories:

```text
tools/debug/
tools/calibration/
tools/capture/
tools/maintenance/
```
