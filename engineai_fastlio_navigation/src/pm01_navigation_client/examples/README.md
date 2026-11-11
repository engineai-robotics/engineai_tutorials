# Python 客户端示例

简体中文 | [English](README.en.md)

| 想做什么 | 示例 |
| --- | --- |
| 导航到一个目标并查看最终结果 | [`single_goal.py`](single_goal.py) |
| 依次到达多个目标 | [`continuous_goals.py`](continuous_goals.py) |
| 查看剩余距离并取消当前目标 | [`feedback_cancel.py`](feedback_cancel.py) |
| 在 `saved` 模式下设置机器人在旧图中的初始位姿 | [`initial_pose.py`](initial_pose.py) |
| 保存当前地图并获取 YAML 路径 | [`save_map.py`](save_map.py) |
| 在仿真中接通、释放运动输出并查看状态 | [`motion_output.py`](motion_output.py) |

按[实操教程](../../../docs/tutorial.md)启动导航或建图。在开发电脑的仓库根目录加载对应环境，仿真用 `source scripts/env.sh`，真机用 `source ~/.config/pm01_navigation/active.env`。保存当前地图可运行以下命令。

```bash
python3 src/pm01_navigation_client/examples/save_map.py
```

运行导航示例前，先把脚本中的目标坐标改为当前地图上的可达位置，再接通运动输出。设置初始位姿时，将脚本中的坐标和朝向改为机器人在旧图中的实际位姿。
