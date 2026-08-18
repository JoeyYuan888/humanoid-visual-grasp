# Grasp Stage

This directory contains the current MPC visual grasp flow entrypoints and
implementation modules.

Daily full-flow command:

```bash
python apps/grasp/run_grasp_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --return-mode qr-present \
  --scan-qr-after-present \
  --qr-transport raw \
  --qr-raw-throttle-ms 3000 \
  --place-after-qr \
  --max-z 1.70 \
  --execute
```

`tools/` is no longer used as the normal grasp-flow entrypoint. Keep daily
operation in this directory.
