#!/usr/bin/env python3
"""在独立 Isaac Sim 进程预览导入的 USDZ，不改运行中的人形场景。"""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = args.manifest.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    specification = json.loads(manifest.read_text())
    items = specification['assets']
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 1280, 'height': 960,
                         'renderer': 'RayTracedLighting'})
    try:
        import omni.usd
        from pxr import UsdGeom
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        context = omni.usd.get_context()
        context.open_stage(str(manifest.parent / specification.get('scene', 'gallery.usda')))
        for _ in range(120):app.update()
        stage = context.get_stage()
        viewport = get_active_viewport()
        viewport.set_texture_resolution((1280,960))
        set_camera_view(eye=[6.5,3.5,3.8],target=[0,1.55,1.8])
        for _ in range(90):app.update()
        capture_viewport_to_file(viewport,str(output/'gallery.png'))
        for _ in range(30):app.update()
        for item in items:
            for other in items:
                imageable = UsdGeom.Imageable(stage.GetPrimAtPath(other['gallery_prim']))
                imageable.MakeVisible() if other is item else imageable.MakeInvisible()
            lo,hi,position=item['bounds_min'],item['bounds_max'],item['gallery_translate']
            center=[(a+b)/2+p for a,b,p in zip(lo,hi,position)]
            size=max(item['size_m_authored'])
            set_camera_view(eye=item.get('preview_eye', [center[0]+size*2.4,center[1]-size*0.55,center[2]+size*0.25]),
                            target=item.get('preview_target', center))
            for _ in range(60):app.update()
            capture_viewport_to_file(viewport,str(output/(item['id']+'.png')))
            for _ in range(30):app.update()
            print('PREVIEW',item['id'],flush=True)
        for item in items:
            UsdGeom.Imageable(stage.GetPrimAtPath(item['gallery_prim'])).MakeVisible()
        print('PREVIEW_COMPLETE',flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
