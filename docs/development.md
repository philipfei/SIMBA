# Development and validation

## Source ownership and current status

SIMBA is the development source; the Pi mirrors its ROS package. Original source and
maps are preserved separately. No Pi or Create 3 was connected during reorganization.
A geometric preview, passing fake-action tests, or old successful teleop is not proof
of real navigation, stopping distance, docking, native cancellation or coverage quality.
Historical validation under examples is not SIMBA validation. New results are in reports.

The ROS package retains its Apache-2.0 declaration. No new third-party algorithm was
copied. Ubuntu/ROS/Python dependencies retain their own licenses and are not vendored.
Map ownership is unchanged. Do not infer new license grants for old user artifacts.

## Nav2 point-to-point acceptance

1. Verify Jazzy/arm64 dependency observations, firmware interfaces and power stability.
2. Stationary checks: fresh scan/odom/hazards/battery, timestamps, complete TF, one
   localization owner, stable AMCL/scan agreement, sole external velocity publisher.
3. Check idle silence and native action ownership before any motion. Test sensor loss
   and stale transforms with fake inputs first. Preserve native backup_only and reflexes.
4. With explicit operator coordination, run <=2 m goals, pause/cancel and separate
   reset/resume. Use current 0.12 m/s, 0.40 rad/s caps; measure stopping behavior.
5. Test obstruction, bounded failure, callback cancellation and no-progress timeouts.
6. Independently calibrate and validate the dock before full coverage start (currently
   required). Then test a small selected region and finally the whole room.

Current configuration uses controller_server/planner_server actions directly, not a
complete BT navigator. RViz goal selection does not automatically execute a mission.

## Charging and checkpoint resume TODO

Current completion or <=25% battery returns to staging, hands off silently to native
Dock, and ends the task after dock confirmation. Charging current is not yet a resume
gate. No CHARGING_WAIT or restart-resume implementation exists.

Implement separately: checkpoint -> return -> dock -> confirm charging -> wait ->
undock -> re-localize -> replan actual remaining coverage. Persist map/config/registration
identity, task identity, fixed denominator, actual covered mask, phase/stripe parameters,
rounds, failures, deadline accounting and dock calibration. Use atomic checkpoints and
fresh action tokens on process restart. Do not resume based only on the last waypoint.

Before implementing this feature, decide the resume battery threshold and charging-time
budget policy explicitly. Current >=30% start threshold is not a charging-complete rule.
Do not reset the mission watchdog to evade its budget. A safety stop/operator cancel
must not become an automatic restart after charging. Test interrupted writes, power loss,
map changes, moved dock and charger failure. Verify Pi/LiDAR power while docked.

## Dynamic obstacles

AMCL uses a fixed map; sensor costmaps can mark/clear observed temporary obstacles.
Coverage does not yet implement automatic expiry/reopening of blocked targets. Saved
obstacles do not disappear merely because furniture moves. New unknown floor does not
expand the denominator. A changed map identity pauses the current task. Future work:
bounded temporary-block retries and local residual replanning, with fixed task area;
remap/recalibrate for lasting layout changes. Do not run mapping SLAM alongside AMCL.

## Deployment maintenance

The active install is a link to an immutable, non-symlink-installed candidate. New
candidates build and test at their final path. Atomic rename changes only the link;
existing shells/processes retain their selected version. Always open a fresh session.
No source rollback, automatic restart, or robot configuration change occurs.

If preflight finds an old ordinary install directory or source-linked installation,
normal --apply refuses before source synchronization. A separate supervised maintenance
window is required: stop launches, identify the exact prior working source/version,
build a self-contained installation at a managed final location, validate its executables,
resources and tests, preserve the original install directory, and explicitly establish
its managed activation record/link. This conversion is not performed by sync.sh. The explicit PC entry `modules/deployment/migrate_legacy.py --apply --stopped` implements it without contacting the robot motion APIs. It compares the rebuilt scripts, package metadata and configuration with the old installed payload, preserves the original directory using atomic Linux rename exchange, and only then makes owned source-mirror directories writable. It does not claim hardware validation or automated test coverage for a historical package with no tests. If the
old source version cannot be established, do not claim that a new build preserves it.

Failure status includes stage, source sync state, active install, exit code and logs.
The failed candidate cannot replace the prior install. Fix explicitly and re-run, or
inspect/revert the source manually. Never copy an unchecked candidate over the active one.
The source mirror may be newer than the active install after failure; that is intentional.

Retain the most recent 3 failed candidates by default (configurable), including logs.
Cleanup runs under the deployment lock only during apply. Active/current, ever-activated,
referenced, running and unknown-status candidates are protected. Old successful versions
require manual retirement; do not retire one still used by a shell/process. Space shortage
fails explicitly rather than deleting usable versions. Cleanup retains a small failure
record and records the reason and removed byte count.

## Legacy operating-instruction migration audit

The two excluded quick-reference files were read, not copied. The inventory below maps
all source sections and contiguous instruction blocks to current README coverage or an
explicit omission. Line numbers refer to the pre-migration files. Original file hashes
are recorded in the migration validation report. This audit does not treat historical
claims as newly verified facts.

Specific corrections:

- Windows-only paths/topology and the old removable-drive workspace are superseded by
  Ubuntu PC module commands. mDNS is primary; no historical DHCP address is trusted.
- Robot Bluetooth name/ID and AP address are omitted: this workflow uses the wired link,
  and these identifiers are not required for operation. Firmware remains unverified.
- Raw /cmd_vel publishing and one-shot stop examples are intentionally retired: they
  bypass current safety ownership. Use keyboard stop, revoke ownership and the gate.
- Historical 0.30/1.00 teleop claims are replaced by actual +0.25/-0.15/0.50 client caps;
  autonomous caps remain 0.12/0.40. Historical 95% policy is replaced by current 90%
  phase-boundary completion and bounded resweeps.
- base_link/laser_frame suggestions are replaced by actual base_footprint/laser TF.
- Blanket apt full-upgrade -y and unreviewed netplan changes are not startup steps;
  maintenance caveats and deliberate diagnostics remain in README.
- The old claim that ping/direct Twist movement proves today's navigation is omitted:
  it was a historical milestone, not current validation.
- Old copy/build --symlink-install commands are replaced by audited isolated deployment.
  Rsync dry-run is never described as a SHA-256 content audit.
- Generic future LiDAR-identification questions are resolved by current A2M8 settings;
  live firmware, payload clearance and sensor freshness still require verification.
- systemd auto-start is intentionally not implemented. Launching a moving mission at
  startup conflicts with the explicit-start policy.
- Historical full-room-before-dock ordering is corrected: full task start requires
  validated dock metadata, so docking validation precedes a full coverage mission.
- Old baseline reuse is retired where registration hashes are absent. Historical
  examples remain labelled historical_unverifiable; there is no hash backfill.

### Complete block inventory

Every non-empty instruction block is classified; no pending items. Headings and
separator-only blocks are included for complete line coverage. The specific corrections
above take precedence over a section-level carried classification.

| Source | Lines | Disposition | README target / reason |
|---|---:|---|---|
| EN | 1-2 | updated | [manual-control-pc](../README.md#manual-control-pc); Current-source values and SIMBA paths supersede historical summaries. |
| EN | 4-4 | updated | [manual-control-pc](../README.md#manual-control-pc); Current-source values and SIMBA paths supersede historical summaries. |
| EN | 6-9 | carried | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); No passwords in Git; rotate exposed passwords. |
| EN | 12-13 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Current PC and wired topology replace Windows/router assumptions; firmware stays unverified. Bluetooth/robot ID/AP metadata is intentionally omitted as unused. |
| EN | 15-27 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Current PC and wired topology replace Windows/router assumptions; firmware stays unverified. Bluetooth/robot ID/AP metadata is intentionally omitted as unused. |
| EN | 29-39 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Current PC and wired topology replace Windows/router assumptions; firmware stays unverified. Bluetooth/robot ID/AP metadata is intentionally omitted as unused. |
| EN | 41-43 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Current PC and wired topology replace Windows/router assumptions; firmware stays unverified. Bluetooth/robot ID/AP metadata is intentionally omitted as unused. |
| EN | 45-47 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Current PC and wired topology replace Windows/router assumptions; firmware stays unverified. Bluetooth/robot ID/AP metadata is intentionally omitted as unused. |
| EN | 49-51 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Current PC and wired topology replace Windows/router assumptions; firmware stays unverified. Bluetooth/robot ID/AP metadata is intentionally omitted as unused. |
| EN | 54-55 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Ubuntu SSH/scp replace PowerShell paths; host-key removal requires fingerprint verification. |
| EN | 57-58 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Ubuntu SSH/scp replace PowerShell paths; host-key removal requires fingerprint verification. |
| EN | 60-61 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Ubuntu SSH/scp replace PowerShell paths; host-key removal requires fingerprint verification. |
| EN | 63-64 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Ubuntu SSH/scp replace PowerShell paths; host-key removal requires fingerprint verification. |
| EN | 66-68 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Ubuntu SSH/scp replace PowerShell paths; host-key removal requires fingerprint verification. |
| EN | 70-71 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Ubuntu SSH/scp replace PowerShell paths; host-key removal requires fingerprint verification. |
| EN | 73-74 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Ubuntu SSH/scp replace PowerShell paths; host-key removal requires fingerprint verification. |
| EN | 76-77 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Ubuntu SSH/scp replace PowerShell paths; host-key removal requires fingerprint verification. |
| EN | 80-81 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 83-84 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 86-88 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 90-92 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 94-96 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 98-99 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 101-102 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 104-105 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 107-109 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 111-112 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 114-116 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 118-119 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 121-122 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 124-125 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); System checks retained; full-upgrade -y retired as an automatic fix; reboot/shutdown require stopped robot work. |
| EN | 128-129 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); SSH status, deliberate enable and listening socket checks. |
| EN | 131-132 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); SSH status, deliberate enable and listening socket checks. |
| EN | 134-135 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); SSH status, deliberate enable and listening socket checks. |
| EN | 137-138 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); SSH status, deliberate enable and listening socket checks. |
| EN | 141-142 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 144-145 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 147-148 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 150-151 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 153-154 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 156-157 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 159-160 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 162-163 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 165-167 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 169-172 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 174-174 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 176-183 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Network diagnostics/static address retained; Netplan apply is maintenance, not routine startup. |
| EN | 186-187 | carried | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Environment, package/interface inventory and discovery restart also listed in Diagnostics. |
| EN | 189-190 | carried | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Environment, package/interface inventory and discovery restart also listed in Diagnostics. |
| EN | 192-195 | carried | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Environment, package/interface inventory and discovery restart also listed in Diagnostics. |
| EN | 197-200 | carried | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Environment, package/interface inventory and discovery restart also listed in Diagnostics. |
| EN | 202-203 | carried | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Environment, package/interface inventory and discovery restart also listed in Diagnostics. |
| EN | 205-207 | carried | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Environment, package/interface inventory and discovery restart also listed in Diagnostics. |
| EN | 209-210 | carried | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Environment, package/interface inventory and discovery restart also listed in Diagnostics. |
| EN | 212-214 | carried | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Environment, package/interface inventory and discovery restart also listed in Diagnostics. |
| EN | 217-218 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 220-221 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 223-224 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 226-227 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 229-230 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 232-233 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 235-236 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 238-239 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 241-242 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 244-245 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 247-248 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Graph, type, publication, bandwidth and one-message diagnostics retained. |
| EN | 251-252 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 254-255 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 257-258 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 260-261 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 263-264 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 266-267 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 269-270 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 272-273 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 275-276 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 278-279 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 281-282 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 284-285 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 287-288 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 290-292 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 294-295 | carried | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Firmware-dependent topics retained through generic topic echo instructions and explicit topic list. |
| EN | 298-299 | deprecated | [manual-control-pc](../README.md#manual-control-pc); Raw cmd_vel/one-shot stop instructions bypass the current gate; replaced by keyboard, timeout and ownership commands. |
| EN | 301-305 | deprecated | [manual-control-pc](../README.md#manual-control-pc); Raw cmd_vel/one-shot stop instructions bypass the current gate; replaced by keyboard, timeout and ownership commands. |
| EN | 307-308 | deprecated | [manual-control-pc](../README.md#manual-control-pc); Raw cmd_vel/one-shot stop instructions bypass the current gate; replaced by keyboard, timeout and ownership commands. |
| EN | 310-311 | deprecated | [manual-control-pc](../README.md#manual-control-pc); Raw cmd_vel/one-shot stop instructions bypass the current gate; replaced by keyboard, timeout and ownership commands. |
| EN | 313-314 | deprecated | [manual-control-pc](../README.md#manual-control-pc); Raw cmd_vel/one-shot stop instructions bypass the current gate; replaced by keyboard, timeout and ownership commands. |
| EN | 316-317 | deprecated | [manual-control-pc](../README.md#manual-control-pc); Raw cmd_vel/one-shot stop instructions bypass the current gate; replaced by keyboard, timeout and ownership commands. |
| EN | 319-320 | deprecated | [manual-control-pc](../README.md#manual-control-pc); Raw cmd_vel/one-shot stop instructions bypass the current gate; replaced by keyboard, timeout and ownership commands. |
| EN | 322-323 | deprecated | [manual-control-pc](../README.md#manual-control-pc); Raw cmd_vel/one-shot stop instructions bypass the current gate; replaced by keyboard, timeout and ownership commands. |
| EN | 325-327 | deprecated | [manual-control-pc](../README.md#manual-control-pc); Raw cmd_vel/one-shot stop instructions bypass the current gate; replaced by keyboard, timeout and ownership commands. |
| EN | 330-331 | updated | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Dock/Undock retained with ownership restrictions; other action definitions are inspection only. |
| EN | 333-334 | updated | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Dock/Undock retained with ownership restrictions; other action definitions are inspection only. |
| EN | 336-337 | updated | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Dock/Undock retained with ownership restrictions; other action definitions are inspection only. |
| EN | 339-340 | updated | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Dock/Undock retained with ownership restrictions; other action definitions are inspection only. |
| EN | 342-344 | updated | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Dock/Undock retained with ownership restrictions; other action definitions are inspection only. |
| EN | 347-348 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Logs, USB, firewall and environment retained; stable serial-by-id replaces ttyUSB/ttyACM guesses. |
| EN | 350-351 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Logs, USB, firewall and environment retained; stable serial-by-id replaces ttyUSB/ttyACM guesses. |
| EN | 353-354 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Logs, USB, firewall and environment retained; stable serial-by-id replaces ttyUSB/ttyACM guesses. |
| EN | 356-357 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Logs, USB, firewall and environment retained; stable serial-by-id replaces ttyUSB/ttyACM guesses. |
| EN | 359-360 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Logs, USB, firewall and environment retained; stable serial-by-id replaces ttyUSB/ttyACM guesses. |
| EN | 362-363 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Logs, USB, firewall and environment retained; stable serial-by-id replaces ttyUSB/ttyACM guesses. |
| EN | 365-366 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Logs, USB, firewall and environment retained; stable serial-by-id replaces ttyUSB/ttyACM guesses. |
| EN | 368-369 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Logs, USB, firewall and environment retained; stable serial-by-id replaces ttyUSB/ttyACM guesses. |
| EN | 371-372 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Logs, USB, firewall and environment retained; stable serial-by-id replaces ttyUSB/ttyACM guesses. |
| EN | 374-375 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Logs, USB, firewall and environment retained; stable serial-by-id replaces ttyUSB/ttyACM guesses. |
| EN | 378-379 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Discovery/network checks retained; raw velocity stop and automatic shell overlay advice superseded by gate/fresh-shell rules. |
| EN | 381-385 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Discovery/network checks retained; raw velocity stop and automatic shell overlay advice superseded by gate/fresh-shell rules. |
| EN | 387-392 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Discovery/network checks retained; raw velocity stop and automatic shell overlay advice superseded by gate/fresh-shell rules. |
| EN | 394-401 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Discovery/network checks retained; raw velocity stop and automatic shell overlay advice superseded by gate/fresh-shell rules. |
| EN | 403-406 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Discovery/network checks retained; raw velocity stop and automatic shell overlay advice superseded by gate/fresh-shell rules. |
| EN | 408-410 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Discovery/network checks retained; raw velocity stop and automatic shell overlay advice superseded by gate/fresh-shell rules. |
| EN | 413-414 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Implemented A2M8/TF/mapping replace old proposals; base_link is incorrect here, RViz goal is staged, automatic mission startup omitted. |
| EN | 416-419 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Implemented A2M8/TF/mapping replace old proposals; base_link is incorrect here, RViz goal is staged, automatic mission startup omitted. |
| EN | 421-427 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Implemented A2M8/TF/mapping replace old proposals; base_link is incorrect here, RViz goal is staged, automatic mission startup omitted. |
| EN | 429-433 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Implemented A2M8/TF/mapping replace old proposals; base_link is incorrect here, RViz goal is staged, automatic mission startup omitted. |
| EN | 435-438 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Implemented A2M8/TF/mapping replace old proposals; base_link is incorrect here, RViz goal is staged, automatic mission startup omitted. |
| EN | 440-445 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Implemented A2M8/TF/mapping replace old proposals; base_link is incorrect here, RViz goal is staged, automatic mission startup omitted. |
| EN | 447-454 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Implemented A2M8/TF/mapping replace old proposals; base_link is incorrect here, RViz goal is staged, automatic mission startup omitted. |
| EN | 456-459 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Implemented A2M8/TF/mapping replace old proposals; base_link is incorrect here, RViz goal is staged, automatic mission startup omitted. |
| EN | 462-463 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Known LiDAR/PC/mount facts documented; firmware, graph, interfaces and live setup still require checks. Pi RAM historical ~8 GB is descriptive, not a runtime requirement. |
| EN | 465-473 | updated | [diagnostics-and-physical-checks](../README.md#diagnostics-and-physical-checks); Known LiDAR/PC/mount facts documented; firmware, graph, interfaces and live setup still require checks. Pi RAM historical ~8 GB is descriptive, not a runtime requirement. |
| EN | 476-477 | deprecated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Historical direct Twist motion does not validate current navigation; not presented as a current acceptance result. |
| EN | 479-480 | deprecated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Historical direct Twist motion does not validate current navigation; not presented as a current acceptance result. |
| EN | 482-486 | deprecated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Historical direct Twist motion does not validate current navigation; not presented as a current acceptance result. |
| EN | 489-490 | updated | [manual-control-pc](../README.md#manual-control-pc); Module scripts replace obsolete removable-drive paths; key focus, release timeout and running Pi gate retained. |
| EN | 492-492 | updated | [manual-control-pc](../README.md#manual-control-pc); Module scripts replace obsolete removable-drive paths; key focus, release timeout and running Pi gate retained. |
| EN | 494-494 | updated | [manual-control-pc](../README.md#manual-control-pc); Module scripts replace obsolete removable-drive paths; key focus, release timeout and running Pi gate retained. |
| EN | 496-496 | updated | [manual-control-pc](../README.md#manual-control-pc); Module scripts replace obsolete removable-drive paths; key focus, release timeout and running Pi gate retained. |
| EN | 498-504 | updated | [manual-control-pc](../README.md#manual-control-pc); Module scripts replace obsolete removable-drive paths; key focus, release timeout and running Pi gate retained. |
| EN | 506-506 | updated | [manual-control-pc](../README.md#manual-control-pc); Module scripts replace obsolete removable-drive paths; key focus, release timeout and running Pi gate retained. |
| EN | 508-508 | updated | [manual-control-pc](../README.md#manual-control-pc); Module scripts replace obsolete removable-drive paths; key focus, release timeout and running Pi gate retained. |
| EN | 510-512 | updated | [manual-control-pc](../README.md#manual-control-pc); Module scripts replace obsolete removable-drive paths; key focus, release timeout and running Pi gate retained. |
| CN | 1-1 | updated | [manual-control-pc](../README.md#manual-control-pc); Current-source values and SIMBA paths supersede historical summaries. |
| CN | 3-4 | updated | [manual-control-pc](../README.md#manual-control-pc); Current-source values and SIMBA paths supersede historical summaries. |
| CN | 6-6 | updated | [manual-control-pc](../README.md#manual-control-pc); Current-source values and SIMBA paths supersede historical summaries. |
| CN | 8-9 | updated | [manual-control-pc](../README.md#manual-control-pc); Current-source values and SIMBA paths supersede historical summaries. |
| CN | 11-12 | updated | [manual-control-pc](../README.md#manual-control-pc); Current-source values and SIMBA paths supersede historical summaries. |
| CN | 14-16 | updated | [manual-control-pc](../README.md#manual-control-pc); Current-source values and SIMBA paths supersede historical summaries. |
| CN | 18-23 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Audited isolated deployment replaces old direct rsync/symlink-install; dry-run is not a content audit. |
| CN | 25-25 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Audited isolated deployment replaces old direct rsync/symlink-install; dry-run is not a content audit. |
| CN | 27-28 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Audited isolated deployment replaces old direct rsync/symlink-install; dry-run is not a content audit. |
| CN | 30-31 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Audited isolated deployment replaces old direct rsync/symlink-install; dry-run is not a content audit. |
| CN | 33-34 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Audited isolated deployment replaces old direct rsync/symlink-install; dry-run is not a content audit. |
| CN | 36-41 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Audited isolated deployment replaces old direct rsync/symlink-install; dry-run is not a content audit. |
| CN | 43-43 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Audited isolated deployment replaces old direct rsync/symlink-install; dry-run is not a content audit. |
| CN | 45-48 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Audited isolated deployment replaces old direct rsync/symlink-install; dry-run is not a content audit. |
| CN | 50-50 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Audited isolated deployment replaces old direct rsync/symlink-install; dry-run is not a content audit. |
| CN | 52-52 | updated | [ssh-paths-and-source-updates-pc](../README.md#ssh-paths-and-source-updates-pc); Audited isolated deployment replaces old direct rsync/symlink-install; dry-run is not a content audit. |
| CN | 54-57 | updated | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Fresh-shell pinned install and DDS interface checks; historical DHCP address intentionally not hardcoded. |
| CN | 59-60 | updated | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Fresh-shell pinned install and DDS interface checks; historical DHCP address intentionally not hardcoded. |
| CN | 62-67 | updated | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Fresh-shell pinned install and DDS interface checks; historical DHCP address intentionally not hardcoded. |
| CN | 69-72 | carried | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); Independent native undock, mapping launch and explicit manual grant/revoke retained. |
| CN | 74-75 | carried | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); Independent native undock, mapping launch and explicit manual grant/revoke retained. |
| CN | 77-81 | carried | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); Independent native undock, mapping launch and explicit manual grant/revoke retained. |
| CN | 83-90 | updated | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); PC module replaces absolute RViz command; TF remaps and interface check retained. |
| CN | 92-99 | updated | [manual-control-pc](../README.md#manual-control-pc); Keyboard module preserves controls and actual source caps. |
| CN | 101-101 | updated | [manual-control-pc](../README.md#manual-control-pc); Keyboard module preserves controls and actual source caps. |
| CN | 103-107 | updated | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); New-name saving of four files and verified PC download; no overwrites. |
| CN | 109-110 | updated | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); New-name saving of four files and verified PC download; no overwrites. |
| CN | 112-113 | updated | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); New-name saving of four files and verified PC download; no overwrites. |
| CN | 115-116 | updated | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); New-name saving of four files and verified PC download; no overwrites. |
| CN | 118-119 | updated | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); New-name saving of four files and verified PC download; no overwrites. |
| CN | 121-123 | updated | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); New-name saving of four files and verified PC download; no overwrites. |
| CN | 125-127 | updated | [mapping-and-map-download-pc](../README.md#mapping-and-map-download-pc); New-name saving of four files and verified PC download; no overwrites. |
| CN | 129-133 | carried | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Stop teleop/launch, verify stationary, native dock; coverage uses manager handoff. |
| CN | 135-136 | carried | [robot-environment-and-basic-commands-pi](../README.md#robot-environment-and-basic-commands-pi); Stop teleop/launch, verify stationary, native dock; coverage uses manager handoff. |
| CN | 139-143 | updated | [offline-coverage-preview-pc-no-robot](../README.md#offline-coverage-preview-pc-no-robot); Isolated tests and explicit registered-start preview replace old paths; ideal coverage and no invented dock retained. |
| CN | 145-150 | updated | [offline-coverage-preview-pc-no-robot](../README.md#offline-coverage-preview-pc-no-robot); Isolated tests and explicit registered-start preview replace old paths; ideal coverage and no invented dock retained. |
| CN | 152-157 | updated | [offline-coverage-preview-pc-no-robot](../README.md#offline-coverage-preview-pc-no-robot); Isolated tests and explicit registered-start preview replace old paths; ideal coverage and no invented dock retained. |
| CN | 159-166 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Interfaces, gates, calibration, deadlines and hardware caveats retained; 95% changed to actual 90% policy; dock validation precedes full start. |
| CN | 168-171 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Interfaces, gates, calibration, deadlines and hardware caveats retained; 95% changed to actual 90% policy; dock validation precedes full start. |
| CN | 173-182 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Interfaces, gates, calibration, deadlines and hardware caveats retained; 95% changed to actual 90% policy; dock validation precedes full start. |
| CN | 184-194 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Interfaces, gates, calibration, deadlines and hardware caveats retained; 95% changed to actual 90% policy; dock validation precedes full start. |
| CN | 196-200 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Interfaces, gates, calibration, deadlines and hardware caveats retained; 95% changed to actual 90% policy; dock validation precedes full start. |
| CN | 202-207 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Historical 95% replaced by implemented 90% phase-end policy; actual pose measurement remains required. |
| CN | 209-210 | updated | [fixed-map-operation-and-todo-pcpihardware-unverified](../README.md#fixed-map-operation-and-todo-pcpihardware-unverified); Interfaces, gates, calibration, deadlines and hardware caveats retained; 95% changed to actual 90% policy; dock validation precedes full start. |
