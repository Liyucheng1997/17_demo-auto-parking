# AutoPark 自动泊车规划与控制仿真平台

一个纯前端、零依赖的自动泊车工程项目：从**车辆与环境建模**出发，完整实现
**Hybrid A\* + Reeds-Shepp 路径规划 → 路径平滑 → 速度规划 → 闭环轨迹跟踪控制**，
并把每个算法阶段的中间结果可视化出来。

当前版本：`v1.1.0` · 在线演示：<https://liyucheng1997.github.io/17_demo-auto-parking/>

## 相比 v1.0 的变化

| | v1.0 | v1.1 |
|---|---|---|
| 单位与坐标 | 像素 | SI 单位（m / rad / s），世界坐标系 |
| 车辆模型 | 矩形贴图沿曲线滑动 | 运动学自行车模型 + 转向/纵向执行器动力学 + 阿克曼几何 |
| 环境模型 | 粗栅格 | 凸多边形障碍物 + 占据栅格 + 欧氏距离场；社会车辆尺寸/姿态随机扰动 |
| 碰撞检测 | 栅格膨胀 | 车身 OBB + 安全裕度，距离场快速通道 + 空间哈希 + SAT |
| 路径规划 | 2D 栅格 A\* + Catmull-Rom | Hybrid A\*（前进/倒车运动基元）+ Reeds-Shepp 解析扩展 |
| 倒车 / 换挡 | 不支持 | 支持多段揉库，换挡点可视化 |
| 速度 | 固定时长动画 | 五类约束的速度规划 |
| 控制 | 无（按路径插值） | 后轮反馈 / 纯跟踪 + 纵向控制，50 Hz 闭环仿真 |
| 传感器 | 装饰性圆圈 | 12 路超声波雷达射线模型 + AEB |
| 工程化 | 单文件 | ES Module 分层架构、Web Worker、39 个测试、基准脚本、CI |

## 功能

- **可视化算法流水线**：点击右侧 7 个阶段，地图自动切换到对应图层并显示原理与公式
  - 代价地图（距离场）、2D 启发函数热力图（含等值线）
  - Hybrid A\* 搜索树逐节点回放（蓝：前进扩展，橙：倒车扩展）
  - Reeds-Shepp 解析扩展尝试（红色虚线为碰撞失败，× 为碰撞点）
  - 原始路径 vs 平滑路径、速度着色、车身扫掠包络
  - 实时跟踪细节：投影点、纯跟踪前视点、瞬时转向中心与转弯圆
- **三种泊车**：垂直倒车入库、垂直车头入库、侧方停车
- **闭环遥测**：车速（参考/实际）、前轮转角（指令/实际）、横向误差、航向误差，支持悬停读数
- **驾驶舱**：方向盘转角、挡位 PRND、车速、12 路雷达状态
- **评估面板**：规划耗时、扩展节点、换挡次数、终点位置/航向误差、最小障碍距离、碰撞/AEB
- **交互式场景**：
  - 编辑模式下点击车位切换占用、点击空地放置/移除锥桶，立即重新规划
  - 拖动本车修改起点、滚轮旋转航向；泊车完成后可从当前位置继续（泊出 + 再泊入）
  - 可调最大转角、规划转角利用率、安全裕度、定位噪声，观察对规划与控制的影响
  - 时间轴回看、0.5×–8× 倍速、跟随视角、缩放平移

## 运行

ES Module 与 Web Worker 需要通过 HTTP 加载（不能直接双击 `index.html`）：

```bash
npm start
```

然后访问 <http://localhost:4173>。也可以使用任意静态服务器，例如 `python -m http.server 4173`。

## 测试与基准

```bash
npm test
```

```bash
npm run bench
```

测试覆盖几何/SAT、距离变换（与暴力解对比）、车辆模型、Reeds-Shepp（随机 500 组终点精确性、
对称性、三角不等式）、全部空闲车位的规划安全性（0 裕度 SAT 独立逐点校验）以及两种控制器的闭环精度。

默认场景基准结果（40 组：12 个空闲车位 × 泊车方式 × 2 种控制器）：

| 指标 | 后轮反馈 | 纯跟踪 |
|---|---|---|
| 成功率 | 20 / 20 | 20 / 20 |
| 平均终点位置误差 | 1.27 cm | 1.99 cm |
| 平均终点航向误差 | 0.13° | 0.94° |
| 平均最大横向跟踪误差 | 1.0 cm | 5.6 cm |

平均规划耗时约 350 ms（倒车入库 13–64 ms，车头入库与紧凑侧方车位 0.2–1.6 s）。

## 项目结构

```text
.
├── index.html                  页面结构
├── styles.css                  样式
├── src
│   ├── core
│   │   ├── math.js             角度、坐标变换、最小堆、可复现随机数
│   │   └── geometry.js         凸多边形、OBB、SAT、射线求交
│   ├── model
│   │   ├── vehicle.js          车辆参数、自行车模型、执行器动力学、阿克曼、雷达布置
│   │   ├── parkingLot.js       停车场环境模型
│   │   └── gridMap.js          占据栅格 + 欧氏距离变换
│   ├── perception
│   │   └── ultrasonic.js       超声波雷达射线模型
│   ├── planning
│   │   ├── planner.js          规划流水线入口 planParking()
│   │   ├── goal.js             目标位姿 / 预目标
│   │   ├── collision.js        三级碰撞检测器
│   │   ├── heuristic.js        2D Dijkstra 启发函数
│   │   ├── reedsShepp.js       Reeds-Shepp 曲线（48 种路径词）
│   │   ├── hybridAStar.js      Hybrid A* 搜索
│   │   ├── pathUtils.js        分段、重采样、Menger 曲率
│   │   ├── smoother.js         约束梯度平滑
│   │   └── speedProfile.js     速度规划
│   ├── control
│   │   └── controllers.js      后轮反馈 / 纯跟踪
│   ├── sim
│   │   └── simulator.js        闭环仿真状态机
│   ├── workers
│   │   └── planner.worker.js   规划 Web Worker
│   └── ui
│       ├── app.js              界面状态机与交互
│       ├── renderer.js         地图渲染（分层缓存）
│       ├── charts.js           遥测曲线
│       ├── dashboard.js        驾驶舱仪表
│       └── stages.js           流水线阶段说明与图层预设
├── tests                       node:test 单元 / 集成测试
├── scripts
│   ├── serve.mjs               零依赖静态服务器
│   └── benchmark.mjs           规划 + 闭环基准
└── docs
    └── ALGORITHM.md            建模与算法详细说明
```

规划与仿真模块不依赖 DOM，可直接在 Node.js 中运行（测试与基准即如此）。

## 文档

建模与算法的公式推导、参数取值和设计取舍见 [docs/ALGORITHM.md](docs/ALGORITHM.md)。

## 技术栈

HTML5 · CSS3 · JavaScript (ES2022 Modules) · Canvas 2D · Web Worker · Node.js 内置测试框架
