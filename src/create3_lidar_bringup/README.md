# Create 3 ROS package

This package contains hardware launch files, manual velocity ownership, mapping,
fixed-map navigation and coverage planning/measurement. Keep the package name and
ROS interfaces stable.

Use the repository [README](../../README.md) for supported module commands, setup,
safety semantics and current hardware-validation status.
Edit `config/coverage_settings.yaml` for coverage/navigation settings and
`config/slam_toolbox.yaml` for manual mapping. `coverage.yaml` and
`nav2_coverage.yaml` are inactive historical references.

Offline geometric previews are not actual measured coverage. Historical baselines
without registration hashes cannot be reused for current comparisons.
