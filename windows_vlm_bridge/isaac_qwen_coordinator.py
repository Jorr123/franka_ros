"""Windows-side static-image coordinator for the plan-only FR3 supervisor."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import mimetypes
from pathlib import Path
import sys
import time
from typing import Any, Mapping
import urllib.error
import urllib.request

from fr3_action_protocol import (
    DEFAULT_SECRET_ENV,
    SIGNATURE_HEADER,
    canonical_json_bytes,
    create_action_envelope,
    load_shared_secret,
    make_signature,
)


DEFAULT_QWEN_URL = "http://127.0.0.1:8000/v1/chat/completions"
DEFAULT_UBUNTU_PLAN_URL = "http://10.10.10.1:8080/v1/actions/plan"
DEFAULT_MODEL_ID = "Qwen/Qwen2.5-VL-7B-Instruct"
FR3_JOINTS = {f"fr3_joint{index}" for index in range(1, 8)}
ALLOWED_IMAGE_TYPES = {"image/jpeg", "image/png", "image/webp"}
MAX_IMAGE_BYTES = 10 * 1024 * 1024


class NoAction(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run Qwen on a Windows image and optionally send one authenticated "
            "proposal to Ubuntu for MoveIt planning only."
        )
    )
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument(
        "--task",
        default=(
            "Inspect the image and report whether the red marker center is left "
            "or right of the vertical dashed center line. Report unclear if it "
            "cannot be determined."
        ),
    )
    parser.add_argument("--frame-id", default="")
    parser.add_argument("--model-id", default=DEFAULT_MODEL_ID)
    parser.add_argument("--qwen-url", default=DEFAULT_QWEN_URL)
    parser.add_argument("--ubuntu-plan-url", default=DEFAULT_UBUNTU_PLAN_URL)
    parser.add_argument("--qwen-timeout", type=float, default=180.0)
    parser.add_argument("--ubuntu-timeout", type=float, default=90.0)
    parser.add_argument("--shared-secret-env", default=DEFAULT_SECRET_ENV)
    parser.add_argument("--min-confidence", type=float, default=0.70)
    parser.add_argument("--max-delta", type=float, default=0.03)
    parser.add_argument("--max-total-delta", type=float, default=0.03)
    parser.add_argument("--max-moving-joints", type=int, default=1)
    parser.add_argument("--envelope-output", type=Path)
    parser.add_argument(
        "--send-plan",
        action="store_true",
        help="Send the proposal to Ubuntu. Without this flag, Windows only prints it.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not args.image.is_file():
        raise RuntimeError(f"Image does not exist or is not a file: {args.image}")
    for name in (
        "qwen_timeout",
        "ubuntu_timeout",
        "max_delta",
        "max_total_delta",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if not 0.0 < args.min_confidence <= 1.0:
        raise RuntimeError("--min-confidence must be in (0, 1].")
    if args.max_moving_joints <= 0:
        raise RuntimeError("--max-moving-joints must be positive.")


def load_image(path: Path) -> tuple[str, str, str]:
    try:
        image_bytes = path.read_bytes()
    except OSError as error:
        raise RuntimeError(f"Could not read image {path}: {error}") from error
    if not image_bytes or len(image_bytes) > MAX_IMAGE_BYTES:
        raise RuntimeError(
            f"Image must be between 1 byte and {MAX_IMAGE_BYTES // (1024 * 1024)} MiB."
        )

    mime_type = (mimetypes.guess_type(path.name)[0] or "").lower()
    if mime_type not in ALLOWED_IMAGE_TYPES:
        raise RuntimeError("Image must be JPEG, PNG, or WebP.")

    digest = hashlib.sha256(image_bytes).hexdigest()
    encoded = base64.b64encode(image_bytes).decode("ascii")
    data_url = f"data:{mime_type};base64,{encoded}"
    return data_url, digest, mime_type


def post_json(
    url: str,
    body: bytes,
    *,
    timeout: float,
    headers: Mapping[str, str] | None = None,
) -> tuple[int, Any]:
    request_headers = {"Content-Type": "application/json"}
    if headers:
        request_headers.update(headers)
    request = urllib.request.Request(
        url,
        data=body,
        headers=request_headers,
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            response_body = response.read()
            status = response.status
    except urllib.error.HTTPError as error:
        response_body = error.read()
        status = error.code
    except urllib.error.URLError as error:
        raise RuntimeError(f"Could not reach {url}: {error.reason}") from error

    try:
        parsed = json.loads(response_body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Server at {url} returned invalid JSON.") from error
    return status, parsed


def call_qwen(args: argparse.Namespace, image_data_url: str) -> dict[str, Any]:
    request = {
        "model": args.model_id,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": args.task},
                    {
                        "type": "image_url",
                        "image_url": {"url": image_data_url},
                    },
                ],
            }
        ],
        "temperature": 0.0,
        "max_tokens": 120,
    }
    status, response = post_json(
        args.qwen_url,
        json.dumps(request, allow_nan=False).encode("utf-8"),
        timeout=args.qwen_timeout,
    )
    if status != 200:
        raise RuntimeError(f"Qwen server returned HTTP {status}: {response}")

    try:
        content = response["choices"][0]["message"]["content"]
        action = json.loads(content)
    except (KeyError, IndexError, TypeError, json.JSONDecodeError) as error:
        raise RuntimeError("Qwen response did not contain action JSON.") from error
    if not isinstance(action, dict):
        raise RuntimeError("Qwen action must be a JSON object.")
    return action


def validate_action(action: Mapping[str, Any], args: argparse.Namespace) -> None:
    expected_keys = {"action", "deltas", "confidence", "observation"}
    if set(action) != expected_keys:
        raise RuntimeError(f"Action keys must be exactly: {sorted(expected_keys)}.")
    if action.get("action") != "joint_deltas":
        raise RuntimeError("Only action='joint_deltas' is accepted.")

    confidence_value = action.get("confidence")
    if isinstance(confidence_value, bool):
        raise RuntimeError("confidence must be numeric, not Boolean.")
    try:
        confidence = float(confidence_value)
    except (TypeError, ValueError) as error:
        raise RuntimeError("confidence must be numeric.") from error
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise RuntimeError("confidence must be finite and in [0, 1].")

    observation = action.get("observation")
    if not isinstance(observation, dict) or set(observation) != {"red_marker_side"}:
        raise RuntimeError("observation must contain only red_marker_side.")
    side = observation["red_marker_side"]
    if side == "unclear" or confidence < args.min_confidence:
        raise NoAction(
            f"side={side!r}, confidence={confidence:.3f}; Ubuntu was not contacted."
        )
    if side not in {"left", "right"}:
        raise RuntimeError("red_marker_side must be left, right, or unclear.")

    deltas = action.get("deltas")
    if not isinstance(deltas, dict) or not deltas:
        raise RuntimeError("deltas must be a non-empty object.")
    moving_deltas: list[float] = []
    total = 0.0
    for joint, raw_delta in deltas.items():
        if joint not in FR3_JOINTS:
            raise RuntimeError(f"Unknown FR3 joint: {joint!r}.")
        if isinstance(raw_delta, bool):
            raise RuntimeError("Joint deltas must be numeric, not Boolean.")
        try:
            delta = float(raw_delta)
        except (TypeError, ValueError) as error:
            raise RuntimeError(f"Delta for {joint} must be numeric.") from error
        if not math.isfinite(delta):
            raise RuntimeError(f"Delta for {joint} must be finite.")
        if abs(delta) > args.max_delta:
            raise RuntimeError(
                f"Delta for {joint} exceeds --max-delta {args.max_delta:.4f}."
            )
        total += abs(delta)
        if abs(delta) > 1e-9:
            moving_deltas.append(delta)

    if not moving_deltas:
        raise NoAction("Qwen proposed zero motion; Ubuntu was not contacted.")
    if len(moving_deltas) > args.max_moving_joints:
        raise RuntimeError("Qwen proposed too many moving joints.")
    if total > args.max_total_delta:
        raise RuntimeError("Qwen proposal exceeds --max-total-delta.")
    if side == "right" and any(delta <= 0.0 for delta in moving_deltas):
        raise RuntimeError("Right observation has a non-positive mapped delta.")
    if side == "left" and any(delta >= 0.0 for delta in moving_deltas):
        raise RuntimeError("Left observation has a non-negative mapped delta.")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        image_data_url, image_sha256, mime_type = load_image(args.image)
        action = call_qwen(args, image_data_url)
        print("Qwen mapped action:")
        print(json.dumps(action, indent=2, sort_keys=True))
        validate_action(action, args)

        frame_id = args.frame_id or f"static-{int(time.time() * 1000)}"
        envelope = create_action_envelope(
            action,
            frame_source="windows-static-file",
            frame_id=frame_id,
            frame_sha256=image_sha256,
        )
        body = canonical_json_bytes(envelope)

        print("\nValidated action envelope:")
        print(json.dumps(envelope, indent=2, sort_keys=True))
        print(f"Image: {args.image} ({mime_type}, sha256={image_sha256[:16]}...)")

        if args.envelope_output:
            args.envelope_output.write_bytes(body + b"\n")
            print(f"Envelope written to: {args.envelope_output}")

        if not args.send_plan:
            print("\nDRY RUN complete. Ubuntu was not contacted. Add --send-plan to request a plan.")
            return 0

        secret = load_shared_secret(args.shared_secret_env)
        signature = make_signature(body, secret)
        status, response = post_json(
            args.ubuntu_plan_url,
            body,
            timeout=args.ubuntu_timeout,
            headers={SIGNATURE_HEADER: signature},
        )
        print(f"\nUbuntu supervisor response (HTTP {status}):")
        print(json.dumps(response, indent=2, sort_keys=True))
        return 0 if status == 200 and response.get("status") == "PLAN_READY" else 1

    except NoAction as error:
        print(f"\nNO_ACTION: {error}")
        return 0
    except Exception as error:
        print(f"Coordinator error: {type(error).__name__}: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
