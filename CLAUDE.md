# CLAUDE.md — liangzhourenwu

> **占位文件**：项目尚未开工，这份只是骨架。等确定要做什么之后重写。

## 继承的规则

本分支基于 `master`，**继承仓库级 `CLAUDE.md`**（工作区在
`~/projects/Payload-SDK/CLAUDE.md`）——设备约束、硬规则、worktree 布局都在
那一份里，**开工前先读它**。

本机专属约定（设备连接方式、ssh 凭据）在 `~/projects/CLAUDE.md`。

## 本工作区状态

- 分支：`feature/liangzhourenwu`，基于 `master`（= 上游 v3.16.0）
- 工作区：`~/projects/liangzhourenwu/`
- **稀疏检出**，工作区实际只有：
  - `psdk_lib/`（头文件 + aarch64/x86_64 静态库）
  - `samples/sample_c/platform/linux/{manifold3,common}`（编译必需的平台适配层）
  - `doc/`、`tools/`（含 dpk 打包脚本）
- **没有** `samples/sample_c++/`、`samples/sample_c/` 的其余部分、`wt_inspection/`
- 没有 `.gitignore`（master 上也没有）

按需取回样例：

```bash
git sparse-checkout add samples/sample_c++/platform/linux/manifold3
git sparse-checkout list    # 看当前稀疏了什么
```

## 待确定

- 这个项目要做什么？
- 是否需要像 `wt_inspection` 那样分「纯算法库 + PSDK 应用」两层
  （见 `~/projects/wt_inspection/wt_inspection/CMakeLists.txt` 的模式）？
- 是否需要复用 `tools/build_dpk/build_dpk.sh` 的打包链路？
