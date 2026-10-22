#!/usr/bin/env python3
"""Plan Cosmos targets; optionally enable local approval or one automatic step."""

from __future__ import annotations

import argparse
import ipaddress
import os
from pathlib import Path
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fr3_ai_supervisor import add_motion_arguments, create_approval_server, validate_args
from robot_bridge.planning import CosmosMoveItPlanner, CosmosSupervisor, PlanServer, ExecuteOnceServer
from robot_bridge.protocol import Limits, PLAN_PATH
from robot_bridge.ros_state import LiveStateMonitor


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-host", default="10.10.10.1")
    parser.add_argument("--port", type=int, default=8084)
    parser.add_argument("--allowed-client", action="append")
    parser.add_argument("--enable-local-execution", action="store_true")
    parser.add_argument("--allow-one-auto-execution", action="store_true",
                        help="Allow execute-once to plan/execute/verify one arm step, then disarm")
    parser.add_argument("--verification-mode", choices=("enforce", "report-only"), default="enforce",
                        help="Report-only records accuracy failures; velocity, drift and large errors still stop")
    parser.add_argument("--approval-socket", type=Path, default=Path("/tmp/fr3_cosmos_approval.sock"))
    parser.add_argument("--plan-ttl", type=float, default=30.0)
    parser.add_argument("--start-state-tolerance", type=float, default=0.002)
    parser.add_argument("--max-start-velocity", type=float, default=0.01)
    parser.add_argument("--gripper-state-topic", default="/franka_gripper/joint_states")
    parser.add_argument("--log-file", type=Path, default=Path("log/cosmos_action_plans.jsonl"))
    add_motion_arguments(parser)
    parser.set_defaults(
        state_topic="/franka/joint_states", max_delta=0.1, max_total_delta=0.4,
        max_moving_joints=7, max_request_age=5.0, max_future_skew=1.0,
        replay_retention=300.0, record_retention=300.0,
        verification_tolerance=0.002, joint7_verification_tolerance=0.002,
        non_commanded_drift_limit=0.002,
    )
    return parser.parse_args(argv)


def validate_config(args):
    if args.enable_local_execution and args.allow_one_auto_execution:
        raise ValueError("Choose local approval or one automatic execution, not both modes together")
    args.allowed_client = args.allowed_client or ["10.10.10.2"]
    allowed = validate_args(args)
    address = ipaddress.IPv4Address(args.bind_host)
    if address.is_unspecified or address.is_multicast:
        raise ValueError("Bind a specific control-network IPv4 address")
    defaults = Limits()
    if args.max_delta > defaults.max_joint_delta_rad or args.max_total_delta > defaults.max_total_delta_rad or args.max_moving_joints > 7:
        raise ValueError("Cosmos motion limits cannot exceed 0.1 rad per joint / 0.4 rad total / seven joints")
    if args.start_state_tolerance > defaults.max_basis_drift_rad:
        raise ValueError("Start-state tolerance cannot exceed 0.002 rad")
    if args.velocity_scaling > 0.03 or args.acceleration_scaling > 0.03:
        raise ValueError("Initial velocity and acceleration scaling cannot exceed 0.03")
    if max(args.goal_tolerance, args.verification_tolerance, args.joint7_verification_tolerance, args.non_commanded_drift_limit) > 0.002:
        raise ValueError("Initial planning/verification tolerances cannot exceed 0.002 rad")
    return allowed, Limits(max_joint_delta_rad=args.max_delta, max_total_delta_rad=args.max_total_delta)


def main():
    args = parse_args()
    allowed, limits = validate_config(args)
    local_approval_enabled = args.enable_local_execution
    args.enable_local_execution = local_approval_enabled or args.allow_one_auto_execution
    # Match the robot/state launchers, while respecting an explicitly set domain.
    os.environ.setdefault("ROS_DOMAIN_ID", "99")
    planner = monitor = http = approval = approval_thread = None
    try:
        def snapshot():
            if monitor is None:
                return 503, {"reason": "ROS state monitor is starting"}
            return monitor.snapshot()

        planner = CosmosMoveItPlanner(args, snapshot=snapshot, limits=limits)
        monitor = LiveStateMonitor(joint_topic=args.state_topic, gripper_topic=args.gripper_state_topic)
        core = CosmosSupervisor(planner, snapshot=snapshot, limits=limits,
                                execution_enabled=args.enable_local_execution, plan_ttl_seconds=args.plan_ttl)
        if args.allow_one_auto_execution:
            http = ExecuteOnceServer((args.bind_host, args.port), core=core, allowed_clients=allowed,
                                     audit_log=args.log_file, local_approval_enabled=local_approval_enabled)
        else:
            http = PlanServer((args.bind_host, args.port), core=core, allowed_clients=allowed, audit_log=args.log_file)
        if local_approval_enabled:
            approval = create_approval_server(args.approval_socket, core)
            approval_thread = threading.Thread(target=approval.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True)
            approval_thread.start()
        print(f"Cosmos planning: http://{args.bind_host}:{args.port}{PLAN_PATH}", flush=True)
        print(f"ROS domain: {os.environ['ROS_DOMAIN_ID']} | Local approval enabled: {local_approval_enabled}", flush=True)
        print(f"One automatic execution armed: {args.allow_one_auto_execution}. No gripper commands are supported.", flush=True)
        print(f"Final verification mode: {args.verification_mode}", flush=True)
        if approval is not None:
            print(f"Local approval socket: {args.approval_socket} (mode 0600)", flush=True)
        http.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if approval is not None:
            approval.shutdown()
            approval.server_close()
            approval_thread.join(timeout=2.0)
            args.approval_socket.unlink(missing_ok=True)
        if http is not None:
            http.server_close()
        if monitor is not None:
            monitor.close()
        if planner is not None:
            planner.close()


if __name__ == "__main__":
    main()
