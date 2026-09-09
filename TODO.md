# Software Work To Be Done (vs. URBX 2026 Rulebook)

## Bugs (blocking — fix before further testing)

- **CSRT tracker crash on real detection.** `cv2.TrackerCSRT_create()` in
  [vision.py](vision.py) raises `AttributeError` — the installed OpenCV
  build doesn't have it. Need `opencv-contrib-python` installed (not plain
  `opencv-python` — they conflict), and possibly `cv2.legacy.TrackerCSRT_create()`
  depending on version. Verify with:
  `python3 -c "import cv2; print(cv2.__version__, hasattr(cv2,'TrackerCSRT_create'))"`

- **HSV range only matches one ball color.** `config.HSV_LOWER/UPPER` is a
  single yellow-ish range — red ball is invisible to detection entirely
  (hue ~0/170-180, not 20-35). This isn't a bug so much as the root cause
  of the next item.

## 1. Dual-ball detection (red + colored)

Field has two simultaneous balls (red = 1pt, colored = 2pts, §3/§5.2).
Current `BallVision`/`config.py` only support one HSV range and return one
undifferentiated ball.

- Add separate HSV ranges: `RED_HSV_LOWER/UPPER` (note: red wraps hue 0/180
  in OpenCV, needs two ranges OR'd together) and `COLORED_HSV_LOWER/UPPER`.
- Run HSV+Hough detection for both colors each frame.
- Change `detect()` to return `{"red": {...}, "colored": {...}}` instead of
  a single ball dict.
- Update FSM to consume the new dict shape.
- Do this before porting vision to the real camera (Day 8 in build roadmap)
  — it touches the vision/FSM interface, cheaper to change now.

## 2. Goal-direction awareness

FSM currently has no concept of which way the goal is — `_do_shoot` fires
as soon as the ball is centered/close, regardless of boat heading. Needed
before this can reliably score rather than just push the ball around.

- Either: detect goal markers/posts visually and align shot toward them, or
- Use IMU/compass heading (already in the hardware plan) + a known goal
  bearing fixed at match start.
- Required before Day 9 closed-loop integration.

## 3. Own-goal risk

§5.2: own goals award 1 point to the opponent. Direct consequence of #2 —
without goal-direction awareness there's real risk of scoring on the wrong
end. Covered by the same fix, called out separately since it's a rules/points
issue, not just missed opportunity.

## 4. Ball-selection strategy

Once both balls are detected (#1), FSM needs logic to choose a target —
e.g. prefer colored ball (2pts, ×3 if fully autonomous = 6pts) and fall
back to red if colored isn't visible. Simple "prefer colored" rule is
enough for v1; revisit if match play reveals better tactics.

## 5. Autonomy-class bookkeeping

§4.3: navigation/propulsion and ball-handling can be independently
autonomous or manual, and boats must be pre-declared as Fully or
Semi-Autonomous. Nothing in the code currently tracks or logs which mode
it's running in, and it's worth confirming `motor_control.py` cleanly
separates nav/propulsion control from ball-handling control so the
Semi-Autonomous fallback (manual ball handling) stays possible without
code changes under time pressure.

## 6. E-stop is a stub

`_read_estop_pin()` in [main.py](main.py) is a `TODO`, not wired to real
GPIO. §4.4 requires both an onboard and a remote kill switch that cut
power to propulsion; §2.2 requires demonstrating this on video. Needs
real hardware wiring and a test showing power actually cutting (not just
a software flag) — ideally verified at the Day 5 bench test in the build
roadmap, well before the Day 7 water test.

## 7. Multi-ball target confusion (watch item, not urgent)

Both balls are the same 25cm diameter, so once dual-ball detection (#1) is
live, sanity-check that FSM target-selection doesn't flicker between two
simultaneously-tracked circles of similar apparent size. Worth a quick
two-circle bench test once #1 lands.

---

## Suggested order

1. Fix CSRT crash + confirm opencv-contrib install (blocking any further testing)
2. Dual-ball HSV detection (#1) — before Day 8 real-camera vision port
3. Real e-stop GPIO wiring (#6) — Day 5 bench test already scheduled for this
4. Goal-direction awareness (#2/#3) — before Day 9 closed-loop test
5. Ball-selection strategy (#4)
6. Autonomy-mode bookkeeping (#5) — lower urgency
