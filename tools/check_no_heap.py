#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fail if tml_core plan/step sources call malloc/new."""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "cpp" / "src" / "core"
PAT = re.compile(r"\b(new|malloc|calloc|realloc)\s*[\(<]")

def main() -> int:
    bad = []
    for path in CORE.rglob("*.cpp"):
        text = path.read_text(encoding="utf-8")
        for i, line in enumerate(text.splitlines(), 1):
            if "check_no_heap" in line:
                continue
            if PAT.search(line):
                bad.append(f"{path}:{i}: {line.strip()}")
    if bad:
        print("heap calls in tml_core:\n" + "\n".join(bad))
        return 1
    print("ok: no malloc/new in tml_core")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
