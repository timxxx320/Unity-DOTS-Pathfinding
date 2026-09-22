# 资源依赖

仓库包含核心算法、场景和配置；原工程的 `Assets/Models` 中角色模型、贴图与动作没有随源码发布。

若只阅读 A*、KD-Tree 或 ORCA，可直接查看代码。若运行示例，请恢复自己持有使用权限的模型资源，或在代理预制体中改用自有模型/Unity 基础几何体，并检查场景与 SubScene 的 Prefab 引用。

Unity 包通过 Packages 下的清单恢复。Library、Temp、Logs、UserSettings 和本地 IDE 配置均不属于源码内容。
