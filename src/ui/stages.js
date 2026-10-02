/**
 * 算法流水线各阶段的说明文字与对应的图层预设。
 * 点击流水线中的阶段时，地图切换到该阶段最能说明问题的图层组合。
 */

export const STAGES = [
  {
    key: "env",
    name: "环境建模",
    subtitle: "障碍物多边形 → 占据栅格 → 欧氏距离场",
    body:
      "停车场被抽象为一组凸多边形障碍物：围墙、结构立柱、绿化带、社会车辆与锥桶。社会车辆的车型尺寸与停放姿态带有随机扰动（横向 ±12 cm、偏航 ±2°），更贴近真实场景。" +
      "随后以 0.2 m 分辨率光栅化为占据栅格，并用 Felzenszwalb 线性时间算法求欧氏距离场。距离场为碰撞检测提供“快速通道”，也是代价地图（越红越靠近障碍物）与 2D 启发函数的基础。",
    formula: "d(p) = min₍ₒ∈O₎ ‖p − o‖          // 可分离平方距离变换，O(N)\n碰撞检测：距离场快速排除 → 空间哈希粗筛 → SAT 分离轴精检",
    layers: ["costmap", "path"],
  },
  {
    key: "goal",
    name: "目标位姿",
    subtitle: "车位几何 + 泊车方式 → 后轴中心位姿",
    body:
      "运动学模型以后轴中心为参考点，因此目标也用后轴中心位姿表示。倒车入库时车头朝外、车尾距车位底线 0.35 m；车头入库则相反；侧方车位沿道路通行方向居中停放。" +
      "垂直车位额外设置“预目标”：先规划到车位外 1 m 处，最后一段直线入库，使航向误差在停车前收敛。图中虚线框为目标车身轮廓，淡色框为预目标。",
    formula: "倒车入库：p_goal = p_open + (D − c − l_rear)·û_in ,  θ_goal = θ_in + π\n预目标：  p_pre = p_goal − dir·1.0 m·(cos θ_goal, sin θ_goal)",
    layers: ["path", "sweep"],
  },
  {
    key: "heuristic",
    name: "启发函数",
    subtitle: "有障碍 2D Dijkstra × 无障碍 Reeds-Shepp",
    body:
      "Hybrid A* 同时使用两个互补的启发函数并取最大值：① 忽略运动学、考虑障碍物的 2D 最短距离（从目标做 8 邻域 Dijkstra，图中越亮离目标越近，每 5 m 一条等值线），能提前发现死胡同与绕行；" +
      "② 忽略障碍物、考虑最小转弯半径的 Reeds-Shepp 最短路径长度，能正确估计“掉头”“揉库”等非完整约束代价。",
    formula: "h(n) = ε · max( h_RS(n), h_2D(n) ),   ε = 1.2",
    layers: ["heuristic", "path"],
  },
  {
    key: "search",
    name: "Hybrid A* 搜索",
    subtitle: "连续位姿 + 运动基元 + 解析扩展",
    body:
      "节点保存连续位姿 (x, y, θ)，仅用 0.4 m × 0.4 m × 5° 的离散栅格判重。每个节点按 {前进, 倒车} × 5 个前轮转角生成 10 条定长 0.9 m 的圆弧基元（由运动学模型精确积分），天然满足最小转弯半径。" +
      "代价函数惩罚倒车、换挡、大转角和转角突变。接近目标时周期性尝试 Reeds-Shepp 解析扩展：按代价排序检测多条 RS 曲线，红色虚线为碰撞失败的尝试（× 为碰撞点），绿色为命中目标的曲线。" +
      "蓝色为前进扩展，橙色为倒车扩展。",
    formula:
      "g' = g + l·(1 | w_rev) + w_gear·[换挡] + w_δ·|δ|/δ_max·l + w_Δδ·|Δδ|/δ_max\nf  = g + h ,  w_rev = 1.6, w_gear = 4.0, w_δ = 0.25, w_Δδ = 0.5",
    layers: ["tree", "rs", "raw"],
  },
  {
    key: "smooth",
    name: "路径平滑",
    subtitle: "按挡位分段 · 梯度下降 · 约束校验",
    body:
      "搜索结果由离散转角的圆弧拼接而成，拼接处转角突变。平滑以换挡点为界分段进行：首尾端点固定、平滑权重在两端线性渐入，以保证换挡点位姿不变。" +
      "平滑后逐点做全车身碰撞检测与曲率校验 (|κ| ≤ κ_max)，不满足则减弱平滑强度重试，最终可回退到原始路径——平滑永远不会让路径变得不安全。白色虚线为原始搜索路径。",
    formula: "J = w_s·Σ‖x_{i−1} − 2x_i + x_{i+1}‖² + w_d·Σ‖x_i − x_i⁰‖²\ns.t.  footprint(x_i) ∩ O = ∅ ,  |κ_i| ≤ κ_max",
    layers: ["raw", "path"],
  },
  {
    key: "speed",
    name: "速度规划",
    subtitle: "多约束速度上限 + 前后向扫描",
    body:
      "每个行驶段起止速度为 0（换挡需停车）。速度上限综合五类约束：挡位限速、横向加速度、曲率减速、障碍物距离、以及泊车中最关键的转向速率约束——曲率变化越剧烈，车辆越要慢下来让转向机“打得过来”，必要时几乎原地打方向。" +
      "前向扫描满足加速度约束，后向扫描满足减速度约束。轨迹颜色越亮表示速度越高。",
    formula: "v_max(s) = min( v_gear, √(a_lat/|κ|), v_gear·(1 − c·|κ|/κ_max), δ̇_max·η / |dδ/ds|, v₀ + k·d_obs )\nv_i = min(v_max,i , √(v_{i−1}² + 2a·Δs)) ,  v_i = min(v_i , √(v_{i+1}² + 2b·Δs))",
    layers: ["speedColor", "path"],
  },
  {
    key: "control",
    name: "轨迹跟踪",
    subtitle: "横向控制 + 纵向控制 + 执行器动力学",
    body:
      "闭环仿真以 50 Hz 运行：控制器根据车辆状态输出期望车速与前轮转角，经转向机（一阶惯性 + 速率限幅）和纵向执行器（一阶惯性 + 加减速限幅）作用于运动学模型。" +
      "后轮反馈控制基于误差动力学的李雅普诺夫设计，充分利用参考曲率；纯跟踪以前视点（蓝点）几何求曲率，直观但会“切弯”。规划只使用 88% 的最大转角，为控制器预留纠偏余量。黄色虚线圆为当前瞬时转向中心对应的后轴轨迹圆；超声波雷达距离过近会触发 AEB。",
    formula:
      "后轮反馈：κ = κ_r·cos e_ψ /(1 − κ_r·e_y) − k_ψ·e_ψ − k_y·e_y·sin(e_ψ)/e_ψ ,  δ = atan(L·κ)\n纯跟踪：  κ = 2·sin α / L_d ,  L_d = clamp(1.0 + 0.6·|v|, 1.0, 2.6)\n纵向：    v_cmd = min( v_ref(s), √(2·b·s_remain) )",
    layers: ["path", "tracking", "sensors"],
  },
];

export const LAYERS = [
  { key: "costmap", label: "代价地图", color: "#ff9a4a" },
  { key: "heuristic", label: "启发函数", color: "#5aa0ff" },
  { key: "tree", label: "搜索树", color: "#5aa0ff" },
  { key: "rs", label: "RS 解析扩展", color: "#ff6b66" },
  { key: "raw", label: "原始路径", color: "#ffffff" },
  { key: "path", label: "规划轨迹", color: "#45e6a7" },
  { key: "speedColor", label: "速度着色", color: "#7dffc7" },
  { key: "sweep", label: "车身扫掠", color: "#45e6a7" },
  { key: "sensors", label: "超声波雷达", color: "#f4c86a" },
  { key: "tracking", label: "跟踪细节", color: "#f4c86a" },
];
