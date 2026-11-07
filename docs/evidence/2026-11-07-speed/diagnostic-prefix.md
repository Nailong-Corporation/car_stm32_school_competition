### 诊断实验：路口证据距离窗口 ×3 后重跑（config 临时修改，未包含在补丁中）

| 巡航 cm/s | 达到步索引 | 终态 | 失败 | 峰值轮速 | 失败位置 |
|---:|---:|---|---|---:|---|
| 100 | 3 | ROUTE_FAILED / LINE_STOPPED | ROUTE_FAILURE_EXIT_LOST | 120.7 | (249,140) 495 cm |
| 150 | 3 | ROUTE_FOLLOW / LINE_STOPPED | ROUTE_FAILURE_NONE | 165.0 | (144,141) 387 cm |
| 200 | 3 | ROUTE_FOLLOW / LINE_STOPPED | ROUTE_FAILURE_NONE | 165.0 | (144,141) 387 cm |
