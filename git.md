初始化与基础配置
初始化： 在项目根目录下运行 git init 将项目纳入版本控制。
配置信息：设置用户名和邮箱：git config --global user.name "Your Name" 和 git config --global user.email "you@example.com"。
忽略文件： 创建 .gitignore 文件，列出不想追踪的文件（如 node_modules/, *.log）。

核心工作流：提交流程
查看状态： git status 随时检查工作区和暂存区状态。
添加到暂存区： git add <file> 或 git add . 将修改添加到暂存区。
提交至仓库： git commit -m "提交说明" 将暂存区内容保存到本地历史记录。 

分支管理 (Branching)
创建分支： git branch <branch-name> 用于开发新功能。
切换分支： git checkout <branch-name> 或 git switch <branch-name>。
合并分支： 将特性分支合并到主分支（master/main）：git merge <branch-name>。 

远程仓库协作
关联远程库： git remote add origin <url>。
拉取代码： git pull origin <branch-name> 获取最新代码。
推送代码： git push -u origin <branch-name> 将本地提交同步到远端。 

常用进阶操作
查看日志： git log 查看提交历史。
版本回退： git reset --hard <commit-id> 回到特定版本。
解决冲突： 当合并分支出现冲突时，手动编辑冲突文件，然后进行 add 和 commit。 