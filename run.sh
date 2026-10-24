#!/bin/bash
# ホストの端末で配置番号を選び、その番号を渡して docker compose up する。
# docker compose up 自身はキー入力をコンテナへ渡さない。
set -euo pipefail
cd "$(dirname "$0")"

if [ -z "${D1_LAYOUT:-}" ]; then
  # shellcheck disable=SC1091
  source app/select_world.sh
fi
export D1_LAYOUT

exec docker compose up --build "$@"
