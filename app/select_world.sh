#!/bin/bash
# 番号から world と初期姿勢を決める。

prompt_layout() {
  echo "部屋の配置を番号で選んでください。番号が決まるまで Gazebo は起動しません。"
  echo
  echo "  1) 床の小物を回避"
  echo "     椅子は正面に見えます。経路上に LiDAR より低い小物があります。"
  echo "  2) 椅子と木箱の見分け"
  echo "     正面の近い位置に木箱、左奥に椅子。どちらも正面カメラに映ります。"
  echo
  while true; do
    if ! read -r -p "番号を入力 [1-3]: " D1_LAYOUT; then
      echo "番号が入力されなかったため終了します。" >&2
      exit 1
    fi
    case "$D1_LAYOUT" in
      1|2|3) return 0 ;;
      *) echo "1 から 2 の番号を入力してください。" ;;
    esac
  done
}

if [ -z "${D1_LAYOUT:-}" ]; then
  if [ -t 0 ]; then
    prompt_layout
  else
    echo "この起動方法では番号を入力できません。docker compose up はキー入力をコンテナに渡しません。" >&2
    echo "リポジトリ直下で ./run.sh を実行してください。" >&2
    exit 1
  fi
fi

world_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/worlds
case "$D1_LAYOUT" in
  1)
    WORLD="${world_dir}/floor_object.world"
    SPAWN_X=-1.55
    SPAWN_Y=0.00
    SPAWN_YAW=0
    ;;
  2)
    WORLD="${world_dir}/crate.world"
    SPAWN_X=-1.55
    SPAWN_Y=0.00
    SPAWN_YAW=0
    ;;
  *)
    echo "配置番号 ${D1_LAYOUT} は 1 から 2 ではありません。" >&2
    exit 1
    ;;
esac

echo "配置 ${D1_LAYOUT} を読み込みます: ${WORLD}"
