# 参考规格与来源

2026-10-30。外观来源是用户本地 L3 GLB，未获得厂家型号对应关系。以下只为临时仿真规格参考。

1. Schneider Harmony XB4BA31（22mm平头按钮）：官方产品页 https://www.se.com/us/en/product/XB4BA31/ 。参考数据表镜像 https://www.coelpuebla.com.mx/files/producto/f_109_descargas_9h0n2ctyc_734.pdf 。借用：22mm安装规格、30mm头部外廓、4.3mm总行程、2.6mm触发行程、3.8N触发力、80g整件质量。cap质量15g、cap直径22mm、预载1N、线性弹簧及阻尼是工程假设，不是数据表参数。搜索证据 `outputs/l3_operable/reference_button*.txt`。
2. Schneider Harmony 选择开关产品系列 https://www.se.com/ca/en/product/XB4BK123M5/ 。借用系列可选的90°两档、±45°三档保持形式。没有把该页面型号认定为L3型号；48mm面板、32mm裸柄、质量、阻力和阻尼均为工程参考。搜索证据 `reference_knob.txt`。
3. 螺丝压板：未找到可确认对应型号的尺寸/扭矩数据表。80mm外高、M6×1等效螺旋副、2圈解锁/最大2.5圈、90°摆角与全部力学值均标注为工程参考。
4. 接触：静/动摩擦0.6/0.5、恢复系数0、接触偏移0.3mm。此为调试参考，未标定特定塑料与人形指垫配对。

参考规格不覆盖L3源资产的使用授权；源资产仍按用户提供数据使用，不在本次操作中对外发布。
