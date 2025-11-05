# 電子情報通信設計製図 16班

## raspiにファイルおくりたくねー？

# git 操作
1. リポジトリを取得（初回のみ）
- git clone git@github.com:YukiOgata-tech/group-16.git
- cd group-16

- 🔁 GitHubの変更を取り込む（他で編集されたとき）
- git pull origin master --rebase


2. 💬 Tips
現在のブランチ確認 → git branch
リモート先確認 → git remote -v
ログ確認 → git log --oneline
全変更を一括pushしたい場合 →
git add . && git commit -m "update" && git push origin master
