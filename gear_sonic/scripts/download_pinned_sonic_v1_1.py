#!/usr/bin/env python3
"""Download the exact SONIC v1.1 deployment artifacts used by this project."""

from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path

from huggingface_hub import hf_hub_download

REPO_ID = "nvidia/GEAR-SONIC"
REVISION = "6733128a3d8a523b1418b06bca3cdf61c8b0987f"
FILES = {
    "sonic_v1_1/model_encoder.onnx": (
        "gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx",
        "fb97de22819b2057b41459802128d91723d91a25f0ad73e7bfc41a9cf8365bae",
    ),
    "sonic_v1_1/model_decoder.onnx": (
        "gear_sonic_deploy/policy/sonic_v1_1/model_decoder.onnx",
        "34bae8570d4a4421a5391a5c2befd745d4a02d182ec539e5f9da44c091c67509",
    ),
    "sonic_v1_1/observation_config.yaml": (
        "gear_sonic_deploy/policy/sonic_v1_1/observation_config.yaml",
        "4a67713b310932e50aca81f19188c8d76013148e98b15c8b5bbea995f12e59f0",
    ),
    "planner_sonic.onnx": (
        "gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx",
        "39b553e197f62f077975ba38512bc04781a3fc37c2af7c6756e04629f760edea",
    ),
}


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 << 20), b""):
            result.update(chunk)
    return result.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root", type=Path, default=Path(__file__).resolve().parents[2]
    )
    parser.add_argument("--token")
    parser.add_argument(
        "--verify-only", action="store_true", help="Do not download missing files"
    )
    args = parser.parse_args()
    root = args.repo_root.resolve()

    for source, (destination, expected) in FILES.items():
        output = root / destination
        actual = digest(output) if output.is_file() else None
        if actual != expected:
            if args.verify_only:
                raise SystemExit(
                    f"checksum mismatch or missing: {destination}\n"
                    f"expected={expected}\nactual={actual}"
                )
            cached = Path(
                hf_hub_download(
                    repo_id=REPO_ID,
                    filename=source,
                    revision=REVISION,
                    token=args.token,
                )
            )
            output.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(cached, output)
            actual = digest(output)
        if actual != expected:
            raise SystemExit(
                f"downloaded checksum mismatch: {destination}\n"
                f"expected={expected}\nactual={actual}"
            )
        print(f"OK {expected}  {destination}")
    print(f"SONIC v1.1 artifacts verified at {REPO_ID}@{REVISION}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
