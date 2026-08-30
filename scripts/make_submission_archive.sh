#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ -n "$(git -C "${PROJECT_ROOT}" status --porcelain)" ]]; then
  echo "커밋되지 않은 변경사항이 있습니다. 먼저 검증 후 커밋하세요." >&2
  exit 1
fi

python3 "${SCRIPT_DIR}/verify_submission.py"

COMMIT="$(git -C "${PROJECT_ROOT}" rev-parse --short=12 HEAD)"
OUTPUT="${1:-${PROJECT_ROOT}/../netclean_submission_${COMMIT}.zip}"

if [[ -e "${OUTPUT}" ]]; then
  echo "기존 파일을 덮어쓰지 않습니다: ${OUTPUT}" >&2
  exit 1
fi

git -C "${PROJECT_ROOT}" archive \
  --format=zip \
  --prefix=netclean_project/ \
  --output="${OUTPUT}" \
  HEAD

echo "제출 ZIP 생성 완료: ${OUTPUT}"
