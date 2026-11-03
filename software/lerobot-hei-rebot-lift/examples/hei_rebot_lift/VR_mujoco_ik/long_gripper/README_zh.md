# HEI ReBot Lift 长夹爪独立模型

本目录是原 `HEI_robot_urdf` 的独立长夹爪变体。原模型和原控制程序保持不变。

## 模型差异

- 双臂、底盘、升降、相机、关节名称和关节范围继承原模型。
- 四个手指使用 `HEI_robot_urdf_6` 导出的长指 STL。
- 夹爪继续使用原版 `prismatic + mimic` 结构，单侧开合范围保持 `0~0.05 m`。
- 四片长指采用新导出的质量、质心和惯量；单片质量为 `0.072795 kg`。
- 左右 TCP 仍位于两指尖端中心，相对各自 `end_link` 的偏移为
  `xyz="0.10510 0 0"`。
- 相比原指尖 `0.0709093213 m`，TCP 向前增加约 `34.19 mm`。

新导出的原始 URDF 把夹爪关节标记为 `fixed`，因此本目录没有直接采用该URDF；
这里以已经验证的原模型为基础，只合并长指几何、惯量和TCP变化。

四个长指网格来自本机 `HEI_robot_urdf_6/meshes`，导入时的 SHA-256 为：

```text
d86df5428a53dc03f1ef7817ce2aa726a6b27efd9e446cc373e8c03b1c72c716  a_right_end_link_L_finger.STL
152ed391608a04b35d0e9e632a40b41a8096268bd6709d62b34e8d3be2d3868a  a_right_end_link_R_finger.STL
43cbef8d55a6050628647157425f8fe48c7657bfaa788e7855690b40be85bf9b  b_left_end_link_L_finger.STL
03609395cdef1d81fe767f68c1a611874cae9899fe00a3af44f7aa9719f71d78  b_left_end_link_R_finger.STL
```

## 独立入口

所有长夹爪文件都位于 `VR_mujoco_ik/long_gripper`。从 `VR_mujoco_ik`
目录运行：

```bash
./long_gripper/run_vr_sim.sh
```

```bash
./long_gripper/run_vr_real.sh --enable-real-publish
```

真机无窗口模式：

```bash
./long_gripper/run_vr_real.sh --enable-real-publish --no-viewer
```

物理抓取仿真：

```bash
./long_gripper/run_vr_physics.sh
```

物理版本保持原夹持垫的近端边界和指尖余量，只将夹持垫远端增加
`34.19 mm`。这样不会改变原短夹爪物理程序。

## 安全说明

长夹爪扩大了末端扫掠范围。首次真机测试应使用低速、清空工作区并保持急停可用；
先分别验证左右臂小范围运动，再测试双臂和靠近机身的姿态。
