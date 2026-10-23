#!/usr/bin/env python3
"""Verify and unpack the pinned test wheels without invoking pip."""

from __future__ import annotations

import argparse
import hashlib
import re
import shutil
import zipfile
from pathlib import Path


HASH_PATTERN = re.compile(r"--hash=sha256:([0-9a-f]{64})", re.IGNORECASE)


def digest(path: Path) -> str:
    """Return the SHA-256 digest of one file."""
    value = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            value.update(chunk)
    return value.hexdigest()


def safe_extract(archive: zipfile.ZipFile, target: Path) -> None:
    """Extract a pure-Python wheel without allowing path traversal."""
    for member in archive.infolist():
        member_path = Path(member.filename)
        if member_path.is_absolute() or ".." in member_path.parts:
            raise ValueError(f"unsafe wheel member: {member.filename}")
        archive.extract(member, target)


def main() -> int:
    """Validate wheel hashes and extract them into the isolated path."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--wheel-dir", required=True)
    parser.add_argument("--requirements", required=True)
    parser.add_argument("--target", required=True)
    args = parser.parse_args()

    wheel_dir = Path(args.wheel_dir)
    requirements = Path(args.requirements)
    target = Path(args.target)
    expected_hashes = set(
        HASH_PATTERN.findall(requirements.read_text(encoding="utf-8"))
    )
    wheels = sorted(wheel_dir.glob("*.whl"))
    if not wheels:
        raise ValueError("no wheel files found")

    shutil.rmtree(target, ignore_errors=True)
    target.mkdir(parents=True, exist_ok=True)
    observed_hashes = set()
    for wheel in wheels:
        wheel_hash = digest(wheel)
        if wheel_hash not in expected_hashes:
            raise ValueError(f"wheel hash is not pinned: {wheel.name}")
        observed_hashes.add(wheel_hash)
        with zipfile.ZipFile(wheel) as archive:
            safe_extract(archive, target)

    missing = sorted(expected_hashes - observed_hashes)
    if missing:
        raise ValueError(f"missing pinned wheels: {missing}")
    print(f"unpacked {len(wheels)} wheels into {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
