# STM32 载板 A6：C5 / C6 / L1 贴装碰撞修复

> **本版为历史快照：U4 的实际焊盘与 TI DDC0006A 推荐图存在差异，A6 内仍未修正；后续 [A7](../stm32PCB-A7/README.md) 已修正实际焊盘。** A6 两排焊盘中心距约 2.296 mm，TI 推荐值为 2.70 mm；单焊盘尺寸也有差异。A7 同时换用基础库矮按键并复核核心板丝印，PCB DRC 为 0，但保留一条 U4 原理图属性匹配警告。不能凭 A6 的 DRC 通过直接忽略封装问题下单。见 [U4 选料提示核查](../../docs/U4-SMT-package-warning.md)。

2026-10-17，在嘉立创 EDA 专业版中，从当前打开的 A5 原生工程建立独立副本并修复。入口为 [stm32PCB/stm32PCB.eprj3](stm32PCB/stm32PCB.eprj3)，打包工程为 [stm32PCB-A6-project.zip](stm32PCB-A6-project.zip)。旧 A5 工程与根目录 V5 制造包保留。

## 修改与间距

将 L1 中心 X 从 15.9494 mm 改为 11.9380 mm，向左移动约 4.0114 mm；Y 与旋转保持原设计。C5、C6 保持原位置。重接四段 SW / VM 铜路端点，保留各段线宽，包括 3.1 mm 宽的 SW / VM 主段；更新顶层地铜对新焊盘和铜路的避让。顶层版本丝印更新为 `REAR / STM32 CAR A6`。

| 检查位置 | A5 名义本体关系 | A6 名义本体最短距离 | A6 按最大本体尺寸计算 |
| --- | --- | ---: | ---: |
| C5 ↔ L1 | 纵向边缘间隙约 0.05 mm | 约 2.555 mm | 约 2.304 mm |
| C6 ↔ L1 | X 投影重叠约 1.456 mm，Y 也相交 | 约 2.556 mm | 约 2.306 mm |

最大尺寸计算使用 Bourns L1 的 11.0 ± 0.5 × 10.0 ± 0.3 mm，以及 Samsung 电容的 3.20 ± 0.20 × 1.60 ± 0.20 mm。以中心坐标和当前旋转构造保守矩形包络；结果覆盖器件本体尺寸公差，不包含工厂贴装位置偏差，不能替代工厂工艺确认。

来源：[Bourns SRP1038A 手册第 1 页](https://bourns.com/docs/Product-Datasheets/SRP1038A.pdf)、[Samsung CL31A226KAHNNN# 原厂规格，包含 E 包装后缀](https://product.samsungsem.com/mlcc/CL31A226KAHNNN.do)。工程原有电容 3D 模型名的高度为 1.3 mm，而厂家厚度为 1.60 ± 0.20 mm；本次平面间距核查使用厂家平面最大尺寸，不以原有 3D 模型外观代替机械验证。

## 已完成的验证

- 在 A6 EDA 会话中运行严格 PCB DRC，返回空数组；保存并重新打开工程后再检查，仍为空数组，L1 新位置已落盘。见 `pcb-drc.json` 与 `reopened-check.json`。
- 对照原理图存档网表核对 32 个器件、143 个焊盘网络。三页原理图文件与 A5 当前原生工程逐字相同；本次没有重新执行原理图 DRC。
- 独立读取原生地铜完整孔洞轮廓并检查铜几何：48 个网络连通，无异网短路，六个 VIN 并联过孔均在顶底两层接入。见 `copper-verification.json`。
- 设计规则与 A5 存档一致。相对修复前实时器件读数，只有 L1 位置发生变化；既有器件属性及用户已有的丝印内容保留，另更新 A6 版本文字。`baseline-live-parts.json` 保存修复前实时器件属性，避免把存档与实时工程间已有差异归入本次修改。
- 从同一 A6 PCB 导出 Gerber、BOM 和贴片坐标；核查 Gerber 中 L1 两个新焊盘的坐标、贴片表全部 32 个器件位置、100 × 90 mm 板框，以及工程 ZIP 与磁盘原生文件一致。

复核入口：

```sh
python3 hardware/stm32PCB-A6/verify_design.py
# 需要 Shapely 2.x；本次使用已有的本地主机环境执行
python3 hardware/stm32PCB-A6/verify_copper.py
```

## 文件与交付边界

- [原生工程 ZIP](stm32PCB-A6-project.zip)：解压后打开 `stm32PCB/stm32PCB.eprj3`。
- [审阅 Gerber](stm32PCB-A6-REVIEW-ONLY-Gerber.zip)：用于工厂重新评审，未提交订单。
- [BOM](stm32PCB-A6-BOM.csv) 和 [贴片坐标](stm32PCB-A6-PickAndPlace.csv)：EDA 原生导出，实际编码为 UTF-16、字段用制表符分隔，保留导出的 `.csv` 文件名。BOM 保留工程原料号；此前订单对 J6/J7 的替代料不自动写入本修订。
- [EDA 整板预览](PCB-board-A6.png)：原生画布输出，包含用户已有丝印；从顶面观察双层铜，不能当作底面镜像焊接图。
- `verification.json`：网络、修订范围、间距及制造输出核查结果。

本次修复并核查 C5 / C6 / L1 区域的平面本体干涉；未完成全板三维装配、插接、贴装实物、电源负载/温升或实车验证。L1 移位使部分 SW 铜路加长，电源纹波与工作效果仍需后续实测。此前记录的 J13 槽孔与供货机械图差异仍需按 [BOM 核查](../../docs/BOM_5406283A_Y1-review.md)处理，本次未改其封装。

实际旧订单绑定的 Gerber 仍未确定；A6 以当前 A5 可编辑工程为基准。下次提交评审应成套使用本目录的 A6 工程与制造文件，并让工厂确认贴装间距；不要混入根目录 V5 或历史版本的坐标表。本次没有重新下单、烧录或操作硬件。
