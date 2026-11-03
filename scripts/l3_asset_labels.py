"""L3机构的中文可读名称；按当前外观与仿真机构命名，不推定实物电气功能。"""


# 名称依据同时区分原GLB外观和参考规格改造后的仿真机构，防止把参考档位当成实物标定。
_COMMON_BASIS = '源GLB正面外观、现有可操作USD机构与catalog；型号及电气用途待确认'
_PENDING_FUNCTION = '待确认；不将颜色或角度直接等同启动、停止、合闸、分闸、远方或就地'


def _entry(name, short, description):
    return {'asset_name': name, 'short_label': short, 'description': description,
            'name_basis': _COMMON_BASIS, 'electrical_function_status': _PENDING_FUNCTION,
            'manufacturer_model': None}


LABELS = {
    'object_0': _entry('绿色带灯自复位按钮（方座）', '绿色带灯按钮·方座',
        '绿色圆形按钮帽与方形安装件外观；当前参考机构为按下触发、撤力弹簧回位。'),
    'object_0_variant': _entry('绿色带灯自复位按钮（圆座）', '绿色带灯按钮·圆座',
        '绿色圆头、圆形底座；现有RM65右手接触演示使用此件。'),
    'object_1': _entry('红色带灯自复位按钮', '红色带灯按钮',
        '红色圆头按钮外观；当前机构为自复位按钮，不能据红色认定为急停。'),
    'object_2': _entry('白色带灯自复位按钮', '白色带灯按钮',
        '白色圆头按钮外观；当前参考机构提供按下和释放状态及配套灯反馈。'),
    'object_3': _entry('圆座长柄两档选择开关', '圆座长柄两档开关',
        '独立圆座和长柄旋钮外观；仿真参考档位为0°与90°。'),
    'object_4': _entry('圆座扁柄三档选择开关', '圆座扁柄三档开关',
        '独立圆座、较扁的双向握柄外观；仿真参考档位为-45°、0°、45°。'),
    'object_5': _entry('圆座凸柄三档选择开关', '圆座凸柄三档开关',
        '独立圆座、向外凸出的握柄外观；仿真参考档位为-45°、0°、45°。'),
    'object_6': _entry('方座两档选择开关A', '方座两档开关A',
        '黑色旋钮配浅色方形面板；与同型副本以A/B区分，A不是设备型号或电气功能。'),
    'object_6_variant': _entry('方座两档选择开关B', '方座两档开关B',
        '黑色旋钮配浅色方形面板；源文件为同型变体，以B区分，电气功能待确认。'),
    'object_19': _entry('方座三档选择开关', '方座三档开关',
        '黑色旋钮配浅色方形面板；面板文字失真，仿真仅按三个参考角度档位使用。'),
    'object_13': _entry('红色螺丝压板（13）', '红色螺丝压板·13',
        '红色绝缘握持部、上下螺母外观；13仅为同色相似资产的原始来源副标识。'),
    'object_14': _entry('红色螺丝压板（14）', '红色螺丝压板·14',
        '红色绝缘握持部、上下螺母；原模型四个主要连通部件用于拆分改造，14为来源副标识。'),
    'object_15': _entry('红色螺丝压板（15）', '红色螺丝压板·15',
        '红色绝缘握持部、上下螺母外观；15仅为同色相似资产的原始来源副标识。'),
    'object_16': _entry('红色螺丝压板（16）', '红色螺丝压板·16',
        '红色绝缘握持部、上下螺母外观；16仅为同色相似资产的原始来源副标识。'),
    'object_111': _entry('黄色螺丝压板', '黄色螺丝压板',
        '黄色双螺母和中央绝缘握持部；当前参考机构支持旋松、拨开、拨回和旋紧。'),
}


def asset_metadata(asset_id, catalog_item=None):
    """返回副本供界面与采集器使用；内部稳定ID和源文件名仍单独保存。"""
    if asset_id in LABELS:
        return dict(LABELS[asset_id])
    item = catalog_item or {}
    name = item.get('asset_name') or item.get('display_name_zh')
    if not name:
        name = {'button': '待命名按钮', 'selector': '待命名选择开关',
                'plate': '待命名螺丝压板'}.get(item.get('kind'), '待命名操作资产')
    return _entry(name, name, '缺少经过核对的中文名称，请补充外观或实物资料。')


def asset_label(asset_id, catalog_item=None):
    return asset_metadata(asset_id, catalog_item)['asset_name']


def short_label(asset_id, catalog_item=None):
    return asset_metadata(asset_id, catalog_item)['short_label']


def apply_scene_labels(stage, prefix, catalog):
    """只改USD展示名，稳定prim路径和物理引用保持可追溯。"""
    for asset_id, item in catalog.items():
        prim = stage.GetPrimAtPath(prefix+'/'+asset_id)
        if prim:
            prim.SetDisplayName(asset_label(asset_id, item))
