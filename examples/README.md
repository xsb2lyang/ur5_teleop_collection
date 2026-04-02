# Examples

This folder contains standalone scripts for hardware checks and legacy experiments.
They are intentionally separated from `src/` to keep the production pipeline clean.

## Structure

- `camera/realsense_d435_preview.py`
  - Opens D435 color stream and previews live image.
- `camera/zed2i_preview.py`
  - Opens ZED 2i and previews left camera stream.
- `robot/gripper_keyboard_control.py`
  - Keyboard-based gripper control (`x` open / `y` close / `esc` exit).
- `robot/ur5_rtde_pose_smoke_test.py`
  - Simple UR5 RTDE connectivity and pose read/move smoke test.
- `legacy/multi_waypoint_collection_legacy.py`
  - Legacy scripted collection flow with fixed waypoints and URScript.

## Notes

- These scripts may contain hard-coded IPs and paths; update before running.
- They are not covered by CI and are not used by `src/main.py`.
- Run them only in a safe robot workspace with proper supervision.
