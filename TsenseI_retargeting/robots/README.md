# robots/

本仓库**不再**在此目录存放 URDF / meshes / keypoints。

唯一手部资源目录：

```text
../../assets/hands/TsenseI_hand/
```

`TopoRetargetConfig` 默认 `robot_root` 已指向该路径。独立使用本包时，请把该
目录一并拷贝，或显式传入 `TopoRetargetConfig(robot_root=...)`。
