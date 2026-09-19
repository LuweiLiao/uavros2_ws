# 可选私有模型子模块

公开的 `uav_gazebo/models` 仅保留 `uni350`；其余模型目录（含 `uni350_multi`、
`common`、`ground` 和 `powerline_perching`）及非 uni350 场景都放在私有仓库中。
上游 `rotors_description` 示例与共享控制插件不属于本次迁移范围。
原目录用相对符号链接保持模型 URI 和启动文件路径。

私有仓库：`git@github.com:LuweiLiao/uavros2_model_prviate.git`。
子模块路径：`private/uavros2_model_prviate`。

普通克隆不需要 `--recurse-submodules`，无需私有权限即可构建和运行 uni350，
步骤见 [uni350 运行说明](uni350-sitl.md)。有权限用户执行：

```bash
git submodule update --init private/uavros2_model_prviate
git -C private/uavros2_model_prviate lfs pull
colcon build --packages-select uav_gazebo --symlink-install
```

安装规则只公开安装 uni350；私有子模块可用时追加安装其他模型和场景。
旧 install 目录可能残留原有模型，公开发布或检查请使用全新的安装目录。
不要分发私有资源、访问令牌或本机旧历史备份。

此前已清理五组私有模型及输电塔派生网格的公开分支历史。本次扩大迁移范围，
新增迁移资源仍存在于旧提交中；本次目录迁移不等同于再次清理历史。
演示媒体、共享代码与上游示例仍保留。
