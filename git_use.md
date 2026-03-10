---
title: git操作命令说明
date: 2026-01-14T11:16:43.266Z
updated: 2026-01-14T11:17:26.959Z
---

# git操作命令
- 配置 Git 关联 GitHub 账号  
> git config --global user.name xytengc  
> git config --global user.email 1300529055@qq.com  


- 本地没有仓库则执行克隆命令
> git clone https://github.com/xytengc/xytengc.github.io.git
- 本地修改文件  
> Git Bash 可以实时查看网页修改：
进入博客仓库目录
cd E:/hducc/git_process/xytengc.github.io
安装项目依赖（管理员权限下执行）
bundle install
启动本地服务
bundle exec jekyll serve --trace

### 注意
Git 全新仓库在「首次提交前」不会自动创建分支（包括 main/master），必须先通过 
git checkout -b main 创建分支，再完成提交，最后推送。


### 完整修改提交步骤
- 进入你的博客项目目录
cd E:/hducc/git_process/xytengc.github.io

- 初始化 Git 仓库（如果还没初始化，必须执行这一步）
git init 

- 先查看本地分支状态（确认在 main 分支）
git branch  # 输出 * main 表示正确

如果没有分支，创建并切换到 main 分支（关键：手动创建分支，解决git branch为空的问题）
git checkout -b main

- 拉取远程 main 分支的最新内容（无冲突时直接同步）
git pull origin main
正常情况：  
Already up to date.  # 本地已是最新，无需同步  
或  
Updating 789abc0..0123456  # 拉取到远程的最新修改  
Fast-forward
 _posts/2026-01-09-fpga-note.md | 2 +-
 1 file changed, 1 insertion(+), 1 deletion(-)

修改ING....
修改完成后
- 输入以下命令，查看哪些文件被修改 / 新增 / 删除，确保是你预期的内容：
> git status

终端会显示红色 / 绿色的文件列表（红色 = 待提交，绿色 = 已暂存），比如：
- 暂存所有修改的文件（包括你的_config.yml、_posts、_includes等）
将所有修改的文件加入 Git 暂存区（准备提交）：
> git add .

- 将暂存的修改提交到本地 Git 仓库，必须添加 “提交信息”（描述这次修改的内容）：
> git commit -m "清理他人内容，替换为个人博客信息，准备发布"

- 首次推送main分支到远程（-u 绑定上游分支，后续推送可直接git push）
git push -u origin main

# Over

- 本地修改后想撤销：  
如果还没执行 git add：直接删除修改的文件，或恢复到原来的内容；  
如果已经 git add 但没 commit：执行 git reset 撤销暂存；  
如果已经 commit 但没 push：执行 git reset --hard HEAD^ 撤销最后一次提交。  