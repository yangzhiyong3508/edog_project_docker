#!/bin/bash
#
# 改动前快照备份工具。
# 用法：
#   ./tools/backup_before_change.sh <描述> [文件1 文件2 ...]
#
# 行为：
#   - 在 .bak_before_<时间戳>_<描述>/ 下复制指定文件的当前内容。
#   - 不指定文件时，备份整个 edog_project（排除 .git、已有备份目录、tools/__pycache__）。
#   - 备份目录已被 .gitignore 忽略，不会入库。
#   - 末尾打印备份目录路径，供回滚时使用。
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_DIR"

DESC="${1:-unnamed}"
shift || true
TS="$(date +%Y%m%d_%H%M%S)"
# 只保留字母数字下划线，避免目录名里出现空格/斜杠
SAFE_DESC="$(printf '%s' "$DESC" | tr -c 'A-Za-z0-9_' '_' | sed 's/__*/_/g; s/^_//; s/_$//')"
[ -z "$SAFE_DESC" ] && SAFE_DESC="unnamed"
BACKUP_DIR=".bak_before_${TS}_${SAFE_DESC}"

mkdir -p "$BACKUP_DIR"

if [ "$#" -gt 0 ]; then
    for f in "$@"; do
        if [ -e "$f" ]; then
            mkdir -p "$BACKUP_DIR/$(dirname "$f")"
            cp -a "$f" "$BACKUP_DIR/$f"
        else
            echo "[backup] 跳过不存在的文件: $f" >&2
        fi
    done
else
    # 整体快照：排除 git、所有备份目录、python 缓存
    rsync -a --exclude='.git/' \
              --exclude='.codex_backup_*/' \
              --exclude='.bak_before_*/' \
              --exclude='tools/__pycache__/' \
              --exclude='docs/Application/tmp-*' \
              ./ "$BACKUP_DIR/" 2>/dev/null || {
        # 没有 rsync 时退回 cp
        find . -mindepth 1 -maxdepth 1 \
            ! -name '.git' \
            ! -name '.codex_backup_*' \
            ! -name '.bak_before_*' \
            -exec cp -a {} "$BACKUP_DIR/" \;
    }
fi

# 记录备份清单，便于回滚时核对
{
    echo "# backup created: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "# desc: $DESC"
    echo "# files:"
    (cd "$BACKUP_DIR" && find . -type f | sed 's|^\./||' | sort)
} > "$BACKUP_DIR/MANIFEST.txt"

echo "[backup] 已创建快照: $BACKUP_DIR"
echo "[backup] 回滚命令: cp -a $BACKUP_DIR/. ./   # 在项目根执行"
