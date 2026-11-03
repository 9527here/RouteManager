---
name: simscene-attached-scene
description: Use SimScene asset search, acquisition, measurement and saved-scene editing with an existing RouteManager Isaac session. Import static visual overlays through attached tools without launching a second simulator or replacing robot controllers.
---

# SimScene attached scene tools

Read `docs/SimScene_资产工具与当前场景接入.md` and `memory/simscene-attached-tools.md` for current setup/evidence.
Canonical adapters: `scripts/simscene_mcp.py` (controller/MCP) and `scripts/simscene_live.py` (existing Kit main thread).

- Use the `simscene` MCP tools for discovery and editable specifications. Keep controller dependencies in the separate SimScene venv, not Isaac Python.
- This user's configured mode is the existing RouteManager session. Do not call the upstream `run`, `simulation_start` or robot-profile worker to add an asset. SimPeople keeps ownership of people, RouteManager keeps ownership of RM65/valve control.
- Search metadata/previews, inspect selected assets and acquire only those needed. Check dependencies, actual units/up-axis, object identity and license; indexed captions and download completion do not certify usability.
- Use revisioned `scene_open/get/apply`, then `attached_scene_preview`. Read `attached_scene_info`; require the expected stage identity, a paused scene and no active robot motion before import.
- The attached importer currently supports only static visual boxes/spheres/assets. Set asset physics to `visual_only` and asset lights to `disabled`; cameras, active lights, physics schemas and controllers are rejected. Do not silently downgrade a requested physical mechanism to scenery.
- Import through `attached_scene_import`, never by editing the running stage from an MCP worker thread. Its conservative checks use current real shape bounds and protect the patrol NavMesh. Those checks do not certify future robot paths.
- Remove only owned overlays through `attached_scene_remove`. Preserve native feedback, targets, gains and paused state. A deleted legacy articulation view may be rebound only against the still-valid backend; never automatically reset or resume.
- Keep failed evidence and separate MCP registration, stdio protocol acceptance, native asset rendering, physical acceptance and natural-language host acceptance. Do not claim marketplace installation.

Learned and exercised in RouteManager on 2026-11-03 with Isaac Sim 6.2 and a one-person SimPeople patrol.
