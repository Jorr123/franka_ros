"""IP-restricted dry-run receiver with no execution interface."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time

from .protocol import (
    Limits, MAX_BODY_BYTES, MODE, PATH, ProposalError,
    canonical_json, decode_json, digest, evaluate_proposal,
    validate_shape,
)


class DryRunServer(ThreadingHTTPServer):
    daemon_threads = True
    mode = MODE
    proposal_path = PATH
    handler_class = None

    def __init__(self, address, *, snapshot, allowed_clients: set[str], audit_log: Path, limits: Limits | None = None):
        self.snapshot = snapshot
        self.allowed_clients = allowed_clients
        self.limits = limits or Limits()
        self.audit_log = audit_log
        self.audit_log.parent.mkdir(parents=True, exist_ok=True)
        with self.audit_log.open("a", encoding="utf-8"):
            pass
        self._lock = threading.Lock()
        self._seen = OrderedDict()
        super().__init__(address, self.handler_class or DryRunHandler)

    def health(self):
        status, _ = self.snapshot()
        return {
            "mode": self.mode, "execution_allowed": False, "robot_execution": False,
            "state_available": status == 200, "limits": asdict(self.limits),
        }

    def process_proposal(self, proposal, state):
        return 200, {
            "validation": evaluate_proposal(proposal, state, self.limits, int(time.time() * 1000)),
            "decision": "ACCEPTED_FOR_DRY_RUN", "code": "VALIDATED",
            "reason": "Format, state drift and delta bounds passed; execution remains disabled",
        }

    def discard_plan(self, proposal):
        """Planning subclasses revoke stored approval if receipt logging fails."""

    def reserve_request(self, request_id: str) -> None:
        now = time.monotonic()
        with self._lock:
            while self._seen and next(iter(self._seen.values())) < now:
                self._seen.popitem(last=False)
            if request_id in self._seen:
                raise ProposalError("REPLAYED_REQUEST", "Request UUID was already received; do not retry")
            if len(self._seen) >= 4096:
                raise ProposalError("RECEIVER_BUSY", "Request replay cache is full")
            self._seen[request_id] = now + self.limits.max_request_age_seconds + self.limits.max_future_skew_seconds + 1

    def record(self, proposal, receipt: dict) -> None:
        with self._lock, self.audit_log.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"proposal": proposal, "receipt": receipt}, allow_nan=False) + "\n")


class DryRunHandler(BaseHTTPRequestHandler):
    server: DryRunServer

    def setup(self):
        super().setup()
        self.connection.settimeout(5.0)

    def _allowed(self) -> bool:
        if self.client_address[0] not in self.server.allowed_clients:
            self._send(403, {"mode": self.server.mode, "execution_allowed": False, "reason": "Client IP is not allowed"})
            return False
        return True

    def do_GET(self):
        if not self._allowed():
            return
        if self.path != "/healthz":
            self._send(404, {"mode": self.server.mode, "execution_allowed": False, "reason": "Not found"})
            return
        self._send(200, self.server.health())

    def do_POST(self):
        if not self._allowed():
            return
        if self.path != self.server.proposal_path:
            self._send(404, {"mode": self.server.mode, "execution_allowed": False, "reason": "Not found"})
            return
        body = b""
        proposal = None
        receipt = {
            "schema_version": 1, "mode": self.server.mode, "execution_allowed": False,
            "robot_execution": False, "request_id": None, "proposal_sha256": None,
            "received_at_unix_ms": int(time.time() * 1000), "decision": "REJECTED",
        }
        status = 422
        try:
            lengths = self.headers.get_all("Content-Length", [])
            if len(lengths) != 1 or self.headers.get("Transfer-Encoding"):
                raise ProposalError("INVALID_HTTP", "Exactly one Content-Length and no transfer encoding required")
            try:
                length = int(lengths[0])
            except ValueError as error:
                raise ProposalError("INVALID_HTTP", "Invalid Content-Length") from error
            if not 0 < length <= MAX_BODY_BYTES:
                raise ProposalError("BODY_TOO_LARGE", "Body must be between 1 byte and 64 KiB")
            if self.headers.get_content_type() != "application/json":
                raise ProposalError("INVALID_HTTP", "Content-Type must be application/json")
            body = self.rfile.read(length)
            if len(body) != length:
                raise ProposalError("INVALID_HTTP", "Incomplete request body")
            receipt["proposal_sha256"] = digest(body)
            proposal = decode_json(body)
            # Echo only a valid request ID, even for rejected or malformed messages.
            if isinstance(proposal, dict) and isinstance(proposal.get("request_id"), str):
                receipt["request_id"] = proposal["request_id"][:128]
            validate_shape(proposal, expected_mode=self.server.mode)
            self.server.reserve_request(proposal["request_id"])
            state_status, state = self.server.snapshot()
            if state_status != 200:
                raise ProposalError("STATE_UNAVAILABLE", str(state.get("reason", "Live state unavailable")))
            status, result = self.server.process_proposal(proposal, state)
            receipt.update(result)
        except ProposalError as error:
            receipt.update(code=error.code, reason=str(error))
            status = 409 if error.code == "REPLAYED_REQUEST" else 422
        except (OSError, TimeoutError) as error:
            receipt.update(code="RECEIVER_ERROR", reason=str(error))
            status = 503
        except Exception as error:
            self.server.discard_plan(proposal)
            receipt.update(code="RECEIVER_ERROR", reason=f"Receiver failed: {type(error).__name__}")
            status = 503
        try:
            self.server.record(proposal, receipt)
        except (OSError, ValueError):
            self.server.discard_plan(proposal)
            receipt.update(decision="REJECTED", code="AUDIT_UNAVAILABLE", reason="Cannot save validation receipt")
            status = 503
        self._send(status, receipt)

    def _send(self, status, payload):
        body = canonical_json(payload)
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        try:
            self.wfile.write(body)
        except (OSError, TimeoutError):
            pass

    def log_message(self, message_format, *args):
        pass
