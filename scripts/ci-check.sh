#!/usr/bin/env bash
# Runs one check on GitHub Actions and makes its failures visible without
# opening the log: every "FAIL …" line (or, if the check crashed, its last
# lines) becomes an error annotation on the run's Summary page, and the whole
# output goes into the step summary.
#
#   scripts/ci-check.sh "<title>" <command> [args…]
set -uo pipefail
title="$1"
shift
out="$(mktemp)"
"$@" 2>&1 | tee "$out"
status=${PIPESTATUS[0]}

fails="$(grep -E '^FAIL' "$out" || true)"
if [ "$status" -ne 0 ]; then
  if [ -n "$fails" ]; then
    while IFS= read -r line; do
      echo "::error title=${title}::${line}"
    done <<< "$fails"
  else
    # No FAIL line: the check stopped early (an exception, or Chrome did not start).
    echo "::error title=${title} crashed::$(tail -n 8 "$out" | tr '\n' ' ' | cut -c1-900)"
  fi
fi

if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
  {
    if [ "$status" -eq 0 ]; then echo "### ✅ ${title}"; else echo "### ❌ ${title}"; fi
    echo '```'
    if [ "$status" -eq 0 ]; then tail -n 3 "$out"; else tail -n 60 "$out"; fi
    echo '```'
  } >> "$GITHUB_STEP_SUMMARY"
fi
rm -f "$out"
exit "$status"
