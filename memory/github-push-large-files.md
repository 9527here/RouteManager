# GitHub 推送链路与历史大文件清洗（2026-10-12）

## 推送通道（已持久化到 ~/.ssh/config）

GitHub SSH:22 被墙，可用通道 = ssh.github.com:443 + SOCKS5：

```
Host github.com
    HostName ssh.github.com
    Port 443
    User git
    IdentityFile ~/.ssh/id_ed25519_github
    IdentitiesOnly yes
    ProxyCommand nc -X 5 -x 127.0.0.1:7897 %h %p
```

原来用的 /tmp/gh_ssh443 是临时文件，重启丢失，现在不用了。
克隆/拉取 HTTP 资源仍需三代理环境变量（见 [[network-proxy-7897]]）。

**注意**：443+SOCKS5 通道上传 ~20KB/s 且大流量会中途 Broken pipe（8MB 左右断过一次）。
大推送用分批：`git push origin <中间提交>:refs/heads/main` 逐提交推，断点小。

## 历史里的大文件（GH001 100MB 拒收）

`git filter-repo --force --invert-paths --path <路径>` 从全部历史剔除。
本仓库剔过：humanoid_upper_body_v3/usd/、两个 urdf/*/configuration/（URDF 导入产物
各 290~305MB）、meshes/visual/base_link.STL（117MB）。.git 355MB→38MB。
filter-repo 要求 fresh clone（本地操作后在克隆里做，bundle 备份先行）。
事后 reflog expire + gc 才回收旧对象。

.gitignore 已加：humanoid_upper_body_v3/usd/、humanoid_upper_body_v3/urdf/*/configuration/、
humanoid_upper_body_v3/meshes/visual/base_link.STL、assets/RM65-6FB-V/urdf/configuration/。
**SX_Link.STL 34MB 保留在库里**（低于限制，URDF 渲染要用）。

## commit vs push

VSCode/命令行 commit 只是本地写对象（大文件慢但能成）；push 才会被 GitHub
pre-receive hook 以 GH001 拒（单文件 >100MB 硬限）。文件进了历史后 .gitignore 无效。
VSCode ✓"提交"≠上传，必须再"同步更改"。

## 双远端（2026-10-14 起）

- origin 配了双 push URL：`git push origin main` 先推内网 GitLab
  (git@8.137.122.254:embodied-ai/vla/RouteManager.git) 再推 GitHub；
  另有独立 gitlab remote。内网 SSH 22 直连不走代理，账号 @linjunyi。
- 项目路径是 **embodied-ai/vla/RouteManager**（vla 是子组，不是 linjunyi 名下）。
- **GitLab main 默认受保护禁止 force push**：改写历史后要去
  Settings → Repository → Protected branches 临时 Unprotect，推完再开。
- filter-repo 会清空 remote 和远程跟踪引用，重写后要重配 remote +
  branch.main.remote/merge。
- 改历史提交人：`--mailmap` 文件格式
  `新名 <新邮箱> <旧邮箱>`（只改名邮箱不变也这么写）；
  本仓提交人 2026-10-14 由"眉山大学"改为 o_range（邮箱 gitee noreply 不变），
  user.name 同步改 o_range。
