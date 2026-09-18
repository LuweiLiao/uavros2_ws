# 可选私有模型子模块

私有仓库：`git@github.com:LuweiLiao/uavros2_model_prviate.git`。
挂载位置：`private/uavros2_model_prviate`。

私有模型为 Scorpio、整个 tsd_model、usl_quadruped2、
usl_quadruped2_bicopter、usl_quadruped3_bicopter，以及从 tsd_model 派生的
输电塔 steel.stl 和 insulators.stl。原模型目录使用相对符号链接，保持
Gazebo 模型 URI 和已有启动文件不变。此调整是用户明确授权的仓库拆分。

普通用户直接克隆主仓库即可，不要加 `--recurse-submodules`。
未初始化私有子模块时，uav_gazebo 仍可构建公共模型；私有机型和使用其
网格的输电塔场景不可用。已有 install 目录可能残留旧模型，公开发行或
验证必须使用新的 build/install 目录，不能把旧安装目录直接打包发布。

有权限用户先安装 Git LFS，再在主仓库执行：

```bash
git lfs install
git submodule update --init private/uavros2_model_prviate
git -C private/uavros2_model_prviate lfs pull
colcon build --packages-select uav_gazebo --symlink-install
```

访问权限由 GitHub 私有仓库的 Collaborators 管理；主仓库公开不会授予
私有仓库读取权限。公开主仓库仍会显示子模块地址、模型名称和提交号。
不要把私人访问令牌写入 .gitmodules，也不要上传私有模型的打包副本。

本次按用户选择只迁移当前版本，不清理旧分支、标签或提交历史。
因此旧公开历史仍可读取这些模型；本次不能视为对已公开内容的撤回。
共享控制插件、既有演示媒体、旧文档及其他仓库未纳入本次私有目录名单。
