# CLAUDE.md

本仓库的 agent 说明**统一维护在仓库根目录的 [AGENTS.md](../AGENTS.md)**，不再在
本文件里复制一份——两份曾经内容重复又各自漂移过。

## 开工前请按顺序读

1. [README.md](../README.md) —— 项目定位与人类上手流程
2. [AGENTS.md](../AGENTS.md) —— 硬性约束、踩坑清单、改了怎么验、代码地图
3. [memory/MEMORY.md](../memory/MEMORY.md) —— 项目自带记忆索引，实测结论都在这

## Claude Code 相关

- 仓库内 `memory/` 是**项目自带**记忆，跟着 git 走；`~/.claude/projects/-home-jy-RouteManager/memory/`
  是 Claude Code 的全局 auto-memory，在用户目录里。**两者不要混**。
- `.claude/skills/image-analyzer/` 供没有视觉能力的模型看图；`.agents/skills/` 下是同一份，
  给别的 agent 工具用。
- 个人设置写 `.claude/settings.local.json`（已被 gitignore）；共享设置才写 `.claude/settings.json`。
