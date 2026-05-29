# SUBMISSION.md — "Implementation of PoC" (due Mon 22 Jun 2026 21:00)

Official rubric = the 3 DT-usage criteria × 4 = **12** (our target). **Take Option A** — Option B caps
at 3+2+2 = **7/12** (Bidirectional≤3, Sync≤2, Environmental≤2). Remaining points toward the
assignment's 30 come from the video/presentation — confirm on Canvas.

## What the zip must contain
1. **Complete workspace** — the whole `ros2_ws/src/algae_dt/` package (code, launch, config, worlds,
   maps, tests). Build clean: `colcon build --packages-select algae_dt`.
2. **Clear launch instructions** — `README.md` quickstart + `docs/SETUP.md` (home + lab) +
   `docs/SCENARIOS_{SIM,LAB}.md`.
3. **Lab-laptop group account** — username + password. **Put them in `CREDENTIALS.txt` (NOT in git)**;
   add it to the zip manually at submission time. Placeholder: `docs/CREDENTIALS.template.txt`.
4. **Demo video 2–3 min** — `docs/DEMO_SCRIPT.md` (must match the final presentation).

## Build the zip (excludes build artifacts + secrets-in-git)
```bash
cd ~/turtlebot3_ws/src      # or repo ros2_ws/src
zip -r algae_dt_submission.zip algae_dt -x '*/build/*' '*/install/*' '*/log/*' '*/__pycache__/*'
# then ADD the demo video + CREDENTIALS.txt to the zip before uploading.
```

## Demo recording (Week 9 — 20-min arena slot)
- You get **20 min in the wooden arena** to record the **2–3 min** clip. Be set up BEFORE your slot.
- Bring the `sim_only` baseline video already recorded (T6.3) as the guaranteed backup.
- Hit all three usages in one clean run (see `docs/DEMO_SCRIPT.md`); remove the robot at time's end.

## Final checklist (tick before upload)
- [ ] `colcon build` clean + `pytest` green on the submitted workspace.
- [ ] `bringup.launch.py mode:=both` runs; `mode:=sim_only` runs as fallback.
- [ ] Video shows: 2-way comms (incl. internal status topic), multi-state sync (incl. battery/mode
      affecting behavior), and a **live environment change** mirrored across both robots.
- [ ] `CREDENTIALS.txt` (real account) added to the zip; NOT committed to git.
- [ ] Launch instructions verified by a teammate from the zip on a clean checkout.
- [ ] Option A confirmed; submitted before **Mon 22 Jun 2026 21:00**.
