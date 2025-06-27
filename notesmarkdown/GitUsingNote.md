# git 的使用指南

## 状态种类
U Untracked 未跟踪
M Modified 已经修改但是未提交
Committed 已经提交

## 主要操作
git init 新建git仓库
git add <filename> 将具体的文件放入库，进行跟踪
git add . 将文件夹里的所有文件都放入库，进行跟踪
git commit -m "<message_you_want_to_say>" 提交文件并且备注信息
git log 显示提交日志
git status 查看当前库的状态
git checkout <index> 填入log中的唯一序列号跳转任意版本
git checkout master 跳转到最新的版本
git checkout -b "<branch_name>" 新建一个分支
> 分支的存在意义更多是分工，版本回溯也可以用分支
git checkout "<branch_name>" 回溯到一个分支的最新版本
git merge "<branch_name>" 将指定分支与当前分支合并

## 与GitHub的交互
git clone <link> 将网上的库下载到本地
git remote -v 显示相关信息
git push 上传到默认分支
git pull 将远程的更改同步到这个设备的本地