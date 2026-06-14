# AutoPark 智能泊车系统

一个纯前端、无依赖的网页版自动泊车项目。用户可以选择空闲车位，由系统规划避障路线并演示完整的自动泊车过程。

当前版本：`v1.0.0`

## 功能

- Canvas 实时绘制停车场、车位、车辆与传感器效果
- 点击选择任意空闲车位
- 使用 A* 算法搜索避障路线
- 路径简化与曲线平滑
- 根据路径切线实时调整车辆朝向
- 支持垂直泊车和侧方泊车
- 可调节泊车动画速度
- 响应式车载控制台界面

## 运行

直接打开 `index.html`，或在当前目录启动本地静态服务器：

```powershell
python -m http.server 4173
```

然后访问 <http://localhost:4173>。

## 项目结构

```text
.
├── index.html
├── styles.css
├── app.js
└── README.md
```

## 技术栈

- HTML5
- CSS3
- JavaScript
- Canvas 2D
- A* 路径规划
