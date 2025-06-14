# -*- coding: utf-8 -*-
from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})
from pxr import UsdPhysics, Gf, Sdf, Tf
import omni.usd

usd_path = "/home/tao/IsaacLab_dex/test_urdf/new_hand1.usd" 
omni.usd.get_context().open_stage(usd_path)

stage = omni.usd.get_context().get_stage()


empty_axis_joints = []
for prim in stage.Traverse():
    joint = UsdPhysics.RevoluteJoint.Get(stage, prim.GetPath()) \
         or UsdPhysics.PrismaticJoint.Get(stage, prim.GetPath())
    if not joint:
        continue

    axis_val = joint.GetAxisAttr().Get()
    if isinstance(axis_val, str) and axis_val.strip() == "":
        empty_axis_joints.append(str(prim.GetPath()))

# 3️⃣ 打印结果
if empty_axis_joints:
    print("⚠️ 以下关节的 axis 为空字符串 (''):")
    for p in empty_axis_joints:
        print("  -", p)
else:
    print("✅ 没有检测到 axis 为空的关节")

simulation_app.close()

