#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 1 || -z "$1" ]]; then
  printf 'Usage: bash scripts/export-public.sh NEW_DIRECTORY\n' >&2
  exit 64
fi
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
destination=$1
while [[ "$destination" == */ && "$destination" != / ]]; do
  destination=${destination%/}
done
if [[ -e "$destination" || -L "$destination" ]]; then
  printf 'Export destination must not already exist.\n' >&2
  exit 64
fi
mkdir -p -- "$(dirname -- "$destination")"
temporary=$(mktemp -d "$(dirname -- "$destination")/.navigation-export.XXXXXXXX")
trap 'rm -rf -- "$temporary"' EXIT
while IFS= read -r relative || [[ -n "$relative" ]]; do
  case "$relative" in
    ""|\#*) continue ;;
    /*|../*|*/../*|*/..|*\\*|docs/PRIVATE*|*/bin/*|*/obj/*|artifacts/*|*.7z|.git/*)
      printf 'Unsafe export entry: %s\n' "$relative" >&2
      exit 1 ;;
  esac
  if [[ ! -f "$project_root/$relative" || -L "$project_root/$relative" ]]; then
    printf 'Missing file or symlink in allowlist: %s\n' "$relative" >&2
    exit 1
  fi
  source_parent=$(dirname -- "$project_root/$relative")
  while [[ "$source_parent" != "$project_root" ]]; do
    if [[ -L "$source_parent" || "$source_parent" != "$project_root/"* ]]; then
      printf 'Symlink or invalid ancestor in allowlist: %s\n' "$relative" >&2
      exit 1
    fi
    source_parent=$(dirname -- "$source_parent")
  done
  mkdir -p -- "$temporary/$(dirname -- "$relative")"
  cp -p -- "$project_root/$relative" "$temporary/$relative"
done < "$project_root/PUBLIC_EXPORT.txt"
mv -- "$temporary" "$destination"
trap - EXIT
printf 'Created local allowlisted export: %s\n' "$destination"
