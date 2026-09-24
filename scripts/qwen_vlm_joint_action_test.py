#!/usr/bin/env python3
"""Ask Qwen-VL for a tiny FR3 action, then validate and plan it through MoveIt.

This script is for early VLM/VLA experiments. The model is allowed to output a
JSON action, but it is never allowed to command the robot directly:

  Qwen output JSON -> local validator -> MoveIt plan -> optional execution

By default this script asks for a primitive action. For later tests, use
--action-space joint-deltas so Qwen can output raw joint deltas. Those deltas are
still bounded by the same safety checks used by vla_joint_command_test.py.
"""

import argparse
import base64
import json
import mimetypes
import os
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any, Dict, Mapping

import rclpy
from moveit_msgs.msg import MoveItErrorCodes

from moveit_joint_goal_test import (
    JOINTS,
    MoveItJointGoalClient,
    add_post_execution_verification_arguments,
    error_name,
    format_joint_table,
    make_plan_request,
    run_post_execution_verification,
    trajectory_summary,
)
from vla_joint_command_test import (
    PRIMITIVES,
    build_target,
    decode_action,
    format_deltas,
    validate_common_args,
    validate_deltas,
    validate_target_limits,
)


DEFAULT_MODEL_ID = "Qwen/Qwen2.5-VL-3B-Instruct"
DEFAULT_API_URL = "http://localhost:8000/v1/chat/completions"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Use Qwen-VL to propose a small FR3 action, then validate it through MoveIt."
    )
    parser.add_argument(
        "--backend",
        choices=["transformers", "openai-compatible"],
        default="transformers",
        help=(
            "Run Qwen locally with Hugging Face transformers, or call a "
            "vLLM/SGLang/OpenAI-compatible chat completions server."
        ),
    )
    parser.add_argument(
        "--model-id",
        default=DEFAULT_MODEL_ID,
        help="Qwen model id. For example Qwen/Qwen2.5-VL-3B-Instruct or Qwen/Qwen2.5-VL-7B-Instruct.",
    )
    parser.add_argument(
        "--image",
        help=(
            "Image path or URL. Local paths are converted to file:// for transformers "
            "and to a base64 data URL for the OpenAI-compatible backend."
        ),
    )
    parser.add_argument(
        "--task",
        default="Choose the smallest safe next robot action from the scene.",
        help="Natural-language task given to the VLM.",
    )
    parser.add_argument(
        "--scene-note",
        default="",
        help="Optional extra text about the scene, camera view, or operator intent.",
    )
    parser.add_argument(
        "--action-space",
        choices=["primitives", "joint-deltas", "both"],
        default="primitives",
        help=(
            "primitives is safest for first tests. joint-deltas allows raw small "
            "FR3 joint deltas in radians. both accepts either schema."
        ),
    )
    parser.add_argument("--max-new-tokens", type=int, default=180)
    parser.add_argument("--temperature", type=float, default=0.0)
    parser.add_argument("--top-p", type=float, default=1.0)
    parser.add_argument(
        "--device",
        choices=["auto", "cuda", "cpu"],
        default="auto",
        help="Only used by the transformers backend.",
    )
    parser.add_argument(
        "--torch-dtype",
        choices=["auto", "float16", "bfloat16", "float32"],
        default="auto",
        help="Only used by the transformers backend.",
    )
    parser.add_argument(
        "--min-pixels",
        type=int,
        default=None,
        help="Optional Qwen image token budget knob for transformers.",
    )
    parser.add_argument(
        "--max-pixels",
        type=int,
        default=None,
        help="Optional Qwen image token budget knob for transformers.",
    )
    parser.add_argument("--api-url", default=DEFAULT_API_URL)
    parser.add_argument("--api-key", default="")
    parser.add_argument("--api-key-env", default="QWEN_API_KEY")
    parser.add_argument("--api-timeout", type=float, default=120.0)
    parser.add_argument("--print-prompt", action="store_true")
    parser.add_argument("--raw-output-file", type=Path)
    parser.add_argument("--json-output-file", type=Path)
    parser.add_argument(
        "--infer-only",
        action="store_true",
        help="Only run Qwen and local JSON validation. Do not contact ROS or MoveIt.",
    )

    parser.add_argument("--min-confidence", type=float, default=0.50)
    parser.add_argument("--max-delta", type=float, default=0.08)
    parser.add_argument("--max-total-delta", type=float, default=0.12)
    parser.add_argument("--max-moving-joints", type=int, default=2)
    parser.add_argument("--joint-limit-margin", type=float, default=0.02)

    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument("--execute-action", default="/execute_trajectory")
    parser.add_argument("--group", default="fr3_arm")
    parser.add_argument("--planning-frame", default="base")
    parser.add_argument("--pipeline", default="")
    parser.add_argument("--planner", default="")
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.05)
    parser.add_argument("--acceleration-scaling", type=float, default=0.05)
    parser.add_argument("--goal-tolerance", type=float, default=0.01)
    parser.add_argument("--workspace-size", type=float, default=2.0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=60.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute the planned motion. Without this flag, MoveIt only plans.",
    )
    add_post_execution_verification_arguments(parser)
    return parser.parse_args()


def validate_inference_args(args: argparse.Namespace) -> None:
    validate_common_args(args)
    if args.max_new_tokens <= 0:
        raise RuntimeError("--max-new-tokens must be positive.")
    if args.temperature < 0.0:
        raise RuntimeError("--temperature must be non-negative.")
    if not (0.0 < args.top_p <= 1.0):
        raise RuntimeError("--top-p must be in (0, 1].")
    if args.api_timeout <= 0.0:
        raise RuntimeError("--api-timeout must be positive.")
    if args.min_pixels is not None and args.min_pixels <= 0:
        raise RuntimeError("--min-pixels must be positive.")
    if args.max_pixels is not None and args.max_pixels <= 0:
        raise RuntimeError("--max-pixels must be positive.")
    if (
        args.min_pixels is not None
        and args.max_pixels is not None
        and args.min_pixels > args.max_pixels
    ):
        raise RuntimeError("--min-pixels must be <= --max-pixels.")


def build_prompt(args: argparse.Namespace) -> str:
    primitive_names = ", ".join(sorted(PRIMITIVES))
    joints = ", ".join(JOINTS)

    rules = [
        "You are controlling a real Franka Research 3 robot through a safety validator.",
        "Return exactly one JSON object and no markdown.",
        "Do not output prose outside the JSON object.",
        "Choose a tiny next action, not a full task trajectory.",
        f"Minimum confidence required by the validator: {args.min_confidence:.2f}.",
        f"Maximum absolute delta per joint: {args.max_delta:.3f} rad.",
        f"Maximum total absolute delta: {args.max_total_delta:.3f} rad.",
        f"Maximum moving joints: {args.max_moving_joints}.",
        f"Valid FR3 joints: {joints}.",
    ]

    schemas = []
    if args.action_space in ("primitives", "both"):
        schemas.append(
            "Primitive schema: "
            '{"action":"primitive","name":"rotate_wrist_right","confidence":0.80,'
            '"reason":"short reason"}. '
            f"Allowed primitive names: {primitive_names}."
        )
    if args.action_space in ("joint-deltas", "both"):
        schemas.append(
            "Joint-delta schema (format only; zero values are neutral placeholders): "
            '{"action":"joint_deltas","deltas":{"fr3_joint7":0.0},'
            '"confidence":0.0,"reason":"short reason"}. '
            "Determine the sign, magnitude, and confidence from the task and image; "
            "do not copy the placeholder values. Use radians and prefer one joint. "
            "Never exceed the maximum delta or maximum total delta stated above."
        )

    prompt_parts = [
        "\n".join(rules),
        "\n".join(schemas),
        f"Operator task: {args.task}",
    ]
    if args.scene_note:
        prompt_parts.append(f"Scene note: {args.scene_note}")
    if not args.image:
        prompt_parts.append("No image was provided, so base the answer only on the text task.")
    return "\n\n".join(prompt_parts)


def run_qwen(args: argparse.Namespace, prompt: str) -> str:
    if args.backend == "transformers":
        return run_transformers_backend(args, prompt)
    return run_openai_compatible_backend(args, prompt)


def run_transformers_backend(args: argparse.Namespace, prompt: str) -> str:
    try:
        import torch
        from qwen_vl_utils import process_vision_info
        from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration
    except ImportError as exc:
        raise RuntimeError(
            "Missing Qwen local dependencies. Install a recent transformers build, "
            "accelerate, qwen-vl-utils, pillow, and torch. Example:\n"
            "  python3 -m pip install -U accelerate qwen-vl-utils pillow torch\n"
            "  python3 -m pip install -U git+https://github.com/huggingface/transformers"
        ) from exc

    processor_kwargs: Dict[str, int] = {}
    if args.min_pixels is not None:
        processor_kwargs["min_pixels"] = args.min_pixels
    if args.max_pixels is not None:
        processor_kwargs["max_pixels"] = args.max_pixels

    processor = AutoProcessor.from_pretrained(args.model_id, **processor_kwargs)
    model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
        args.model_id,
        torch_dtype=resolve_torch_dtype(torch, args.torch_dtype),
        device_map="auto" if args.device == "auto" else None,
    )
    if args.device in ("cuda", "cpu"):
        model = model.to(args.device)

    messages = build_qwen_messages(args, prompt, image_kind="image")
    text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    image_inputs, video_inputs = process_vision_info(messages)
    inputs = processor(
        text=[text],
        images=image_inputs,
        videos=video_inputs,
        padding=True,
        return_tensors="pt",
    )
    inputs = inputs.to(select_input_device(torch, model, args.device))

    generation_kwargs: Dict[str, Any] = {"max_new_tokens": args.max_new_tokens}
    if args.temperature > 0.0:
        generation_kwargs.update(
            do_sample=True,
            temperature=args.temperature,
            top_p=args.top_p,
        )
    else:
        generation_kwargs["do_sample"] = False

    with torch.inference_mode():
        generated_ids = model.generate(**inputs, **generation_kwargs)

    generated_ids_trimmed = [
        output_ids[len(input_ids) :]
        for input_ids, output_ids in zip(inputs.input_ids, generated_ids)
    ]
    output_text = processor.batch_decode(
        generated_ids_trimmed,
        skip_special_tokens=True,
        clean_up_tokenization_spaces=False,
    )
    return output_text[0].strip()


def resolve_torch_dtype(torch_module: Any, dtype_name: str) -> Any:
    if dtype_name == "auto":
        return "auto"
    if dtype_name == "float16":
        return torch_module.float16
    if dtype_name == "bfloat16":
        return torch_module.bfloat16
    if dtype_name == "float32":
        return torch_module.float32
    raise RuntimeError(f"Unsupported torch dtype: {dtype_name}")


def select_input_device(torch_module: Any, model: Any, requested_device: str) -> Any:
    if requested_device in ("cuda", "cpu"):
        return requested_device
    if hasattr(model, "device") and str(model.device) != "meta":
        return model.device
    return "cuda" if torch_module.cuda.is_available() else "cpu"


def run_openai_compatible_backend(args: argparse.Namespace, prompt: str) -> str:
    content = []
    if args.image:
        content.append(
            {
                "type": "image_url",
                "image_url": {"url": image_to_openai_url(args.image)},
            }
        )
    content.append({"type": "text", "text": prompt})

    payload = {
        "model": args.model_id,
        "messages": [{"role": "user", "content": content}],
        "max_tokens": args.max_new_tokens,
        "temperature": args.temperature,
    }
    if args.temperature > 0.0:
        payload["top_p"] = args.top_p

    headers = {"Content-Type": "application/json"}
    api_key = args.api_key or os.environ.get(args.api_key_env, "")
    if api_key:
        headers["Authorization"] = f"Bearer {api_key}"

    request = urllib.request.Request(
        args.api_url,
        data=json.dumps(payload).encode("utf-8"),
        headers=headers,
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=args.api_timeout) as response:
            response_payload = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"Qwen API HTTP {exc.code}: {detail}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Could not contact Qwen API at {args.api_url}: {exc}") from exc

    return extract_openai_text(response_payload)


def extract_openai_text(response_payload: Mapping[str, Any]) -> str:
    choices = response_payload.get("choices")
    if not isinstance(choices, list) or not choices:
        raise RuntimeError("OpenAI-compatible response did not contain choices.")

    message = choices[0].get("message", {})
    content = message.get("content")
    if isinstance(content, str):
        return content.strip()
    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, dict) and item.get("type") == "text":
                parts.append(str(item.get("text", "")))
            elif isinstance(item, str):
                parts.append(item)
        return "\n".join(parts).strip()
    raise RuntimeError("OpenAI-compatible response did not contain text content.")


def build_qwen_messages(
    args: argparse.Namespace,
    prompt: str,
    image_kind: str,
) -> list:
    content = []
    if args.image:
        content.append({"type": image_kind, image_kind: image_to_transformers_url(args.image)})
    content.append({"type": "text", "text": prompt})
    return [{"role": "user", "content": content}]


def image_to_transformers_url(image: str) -> str:
    if is_remote_or_data_url(image) or image.startswith("file://"):
        return image
    path = Path(image).expanduser()
    if path.exists():
        return path.resolve().as_uri()
    return image


def image_to_openai_url(image: str) -> str:
    if image.startswith("data:") or image.startswith("http://") or image.startswith("https://"):
        return image

    if image.startswith("file://"):
        parsed = urllib.parse.urlparse(image)
        path = Path(urllib.parse.unquote(parsed.path))
    else:
        path = Path(image).expanduser()

    if not path.exists():
        return image

    mime_type, _ = mimetypes.guess_type(str(path))
    if mime_type is None:
        mime_type = "image/jpeg"
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime_type};base64,{encoded}"


def is_remote_or_data_url(value: str) -> bool:
    return value.startswith(("http://", "https://", "data:"))


def extract_json_object(text: str) -> Dict[str, Any]:
    decoder = json.JSONDecoder()
    stripped = strip_code_fence(text.strip())

    try:
        parsed = json.loads(stripped)
        if isinstance(parsed, dict):
            return parsed
    except json.JSONDecodeError:
        pass

    for index, char in enumerate(stripped):
        if char != "{":
            continue
        try:
            parsed, _ = decoder.raw_decode(stripped[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            return parsed

    raise RuntimeError(f"Qwen output did not contain a JSON object: {text!r}")


def strip_code_fence(text: str) -> str:
    if not text.startswith("```"):
        return text
    lines = text.splitlines()
    if len(lines) >= 3 and lines[-1].strip() == "```":
        return "\n".join(lines[1:-1]).strip()
    return text


def enforce_action_space(action: Mapping[str, Any], args: argparse.Namespace) -> None:
    action_type = str(action.get("action", "")).strip()
    if not action_type and "primitive" in action:
        action_type = "primitive"

    primitive = action_type == "primitive"
    joint_delta = action_type in ("joint_delta", "joint_deltas")

    if args.action_space == "primitives" and not primitive:
        raise RuntimeError(
            "Qwen output a raw joint command, but --action-space primitives was requested."
        )
    if args.action_space == "joint-deltas" and not joint_delta:
        raise RuntimeError(
            "Qwen output a primitive action, but --action-space joint-deltas was requested."
        )


def write_optional_outputs(
    args: argparse.Namespace,
    raw_text: str,
    action: Mapping[str, Any],
) -> None:
    if args.raw_output_file:
        args.raw_output_file.write_text(raw_text + "\n", encoding="utf-8")
    if args.json_output_file:
        args.json_output_file.write_text(
            json.dumps(action, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )


def main() -> int:
    args = parse_args()

    try:
        validate_inference_args(args)
        prompt = build_prompt(args)
        if args.print_prompt:
            print("Prompt sent to Qwen:")
            print(prompt)
            print()

        raw_text = run_qwen(args, prompt)
        action = extract_json_object(raw_text)
        enforce_action_space(action, args)
        deltas, action_label = decode_action(action, args)
        validate_deltas(deltas, args)
        write_optional_outputs(args, raw_text, action)
    except RuntimeError as exc:
        print(f"Input rejected: {exc}", file=sys.stderr)
        return 2

    print("Qwen raw output:")
    print(raw_text)
    print("\nParsed action JSON:")
    print(json.dumps(action, indent=2, sort_keys=True))
    print(f"\nQwen action accepted by local validator: {action_label}")
    print(format_deltas(deltas))

    if args.infer_only:
        print("\nInfer-only run complete. No ROS or MoveIt command was sent.")
        return 0

    rclpy.init()
    node = MoveItJointGoalClient(args.state_topic, args.plan_service, args.execute_action)

    try:
        current = node.wait_for_joint_state(timeout=5.0)
        target = build_target(current, deltas)
        validate_target_limits(target, args)

        print(format_joint_table(current, target))
        print(
            f"\nMode: {'EXECUTE' if args.execute else 'PLAN ONLY'}\n"
            f"MoveIt group: {args.group}\n"
            f"Backend: {args.backend}\n"
            f"Model: {args.model_id}\n"
            f"Action space: {args.action_space}\n"
            f"Velocity scaling: {args.velocity_scaling:.3f}\n"
            f"Acceleration scaling: {args.acceleration_scaling:.3f}\n"
            f"Goal tolerance: {args.goal_tolerance:.4f} rad\n"
        )

        node.wait_for_plan_service(timeout=args.server_timeout)
        plan_request = make_plan_request(node, current, target, args)
        plan_response = node.plan(plan_request, timeout=args.result_timeout)

        plan_code = plan_response.error_code.val
        print(trajectory_summary(plan_response.trajectory))
        print(f"Planning result: {error_name(plan_code)}")

        if plan_code != MoveItErrorCodes.SUCCESS:
            return 1

        if not args.execute:
            print("Plan-only run complete. Add --execute to move the robot.")
            return 0

        node.wait_for_execute_action(timeout=args.server_timeout)
        execute_result = node.execute(plan_response.trajectory, timeout=args.result_timeout)
        execute_code = execute_result.error_code.val
        print(f"Execution result: {error_name(execute_code)}")
        if execute_code != MoveItErrorCodes.SUCCESS:
            return 1
        return 0 if run_post_execution_verification(node, current, target, args) else 1

    except KeyboardInterrupt:
        print("Interrupted.")
        return 130
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
