# SIMBA validation

Local source was independently built and installed from SIMBA.

- 66 existing cases and 31 new cases passed; zero failures, errors or skips.
- Colcon reports 3 additional CTest wrapper results (100 total including wrappers).
- All 4,182 original regular files match their pre-migration SHA-256 values.
- All four selected map files match the originals.
- The 38-file ROS source manifest matches the install; installed executable/resource/module contents were checked.
- 176 legacy instruction blocks have migration dispositions; no pending blocks.
- Current-map review: 62.55 m, 100% ideal coverage, zero ideal residual. This is not measured robot coverage.
- A live shell retained its old release across an atomic install-link switch; a fresh shell loaded the new one.
- Deployment failures and cleanup use local fixtures and fake SSH/rsync, never the Pi.

The PC system still lacks some dependencies. Tests used a temporary isolated extracted
Ubuntu dependency environment (Matplotlib 3.6.3 and the recorded ROS message packages),
not an installed full Nav2 stack. No dependency binaries were copied into SIMBA. Use
the README installation steps and read-only checker for a normal persistent setup.
The test-only prefix is `/tmp/simba-test-runtime`; it is temporary, not a project dependency.

Machine-readable results: `reports/validation.json`, `reports/dependencies_pc_system.json`
and test XML. The reviewed figure is `reports/room_20260923_1338_review/preview.png`.
Generated reports are ignored by Git. Historical examples are separately labelled and
cannot replace these results. No robot motion, Pi deployment or GitHub creation occurred.

## First-install migration follow-up

Five additional local tests pass: read-only inspection, successful atomic legacy
migration, and injected build/content/exchange failures. They verify old installation,
source contents, build and maps are preserved; no SSH or hardware was used. These
checks are separate from the original 97-case migration validation.
