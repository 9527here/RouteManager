---
name: simpeople-scene-attach
description: Attach SimPeople native people to an existing RouteManager Isaac Sim scene, verify real walking and cancellation, and preserve robot control. Use for people patrol integration or attached-runtime startup failures; no independent simulator launch.
---

# SimPeople scene attachment

Use `scripts/simpeople_patrol.py` and `scripts/simpeople_scene.py` as canonical code. Read `docs/SimPeople_场景附着与单人巡检.md` for the measured local scene and acceptance scope.

- Inspect live robot identity, fixed-base status, physical feedback, current task ownership and scene geometry before authoring anything. Match the user's selected Isaac version.
- Keep controller dependencies in a separate venv. Use the official scene-attaching Kit extension, session overlay and native Behavior; never fake progress with root teleports or another bone writer.
- Load animation runtime before the first full Play where possible. A late load into a paused, previously played scene can leave motion matching uninitialized despite valid assets/path/preflight. Diagnose actual joint/root motion and native PLAYINFO. Any required backend lifecycle recovery needs a complete measured checkpoint and explicit verification of restored targets, gains, body poses and real physical frames.
- Extend zero-length scene time ranges before physical construction. Do not change timeline frame rate/defaultPrim during a running articulation to troubleshoot animation.
- Verify stage units and a baked reachable NavMesh, then one actor's actual A/B arrivals and cancellation settlement before combining with a robot action.
- Validate actor deletion/extension unload without a scene reset: this project's legacy Articulation can lose its physics view on unrelated prim deletion. Rebind only when the existing backend is valid, and compare actual state/targets/gains.
- For symmetric robot grasps, compare equivalent IK solutions against the current measured joint pose; the first feasible solution may introduce a large elbow/wrist branch change. Retain the original collision, tracking and contact gates.
- Keep failed native trials and continuous viewport footage with timestamps. Animation rendering does not certify physical human contact, robot avoidance, sensor detection, shared physics-step timing or real-time crowd capacity.

Learned from the 2026-11-03 RouteManager attachment. The current-session acceptance is measured; the optional cold-start preloader was syntax checked only.
