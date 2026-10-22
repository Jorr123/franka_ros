"""Submit validation/planning proposals and read their Ubuntu receipts/results."""

from __future__ import annotations

import math
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

from .protocol import (
    MAX_BODY_BYTES, MODE, PATH, PLAN_MODE, PLAN_PATH, EXECUTE_MODE, EXECUTE_PATH, canonical_json, decode_json,
    digest, validate_shape,
)

DEFAULT_URL = "http://10.10.10.1:8083" + PATH


class NoRedirects(HTTPRedirectHandler):
    def redirect_request(self, request, response, code, message, headers, new_url):
        return None


def submit_proposal(proposal: dict, *, url: str = DEFAULT_URL, timeout: float = 3.0) -> dict:
    return _submit(proposal, url=url, timeout=timeout, path=PATH, mode=MODE, success="ACCEPTED_FOR_DRY_RUN")


def submit_plan(proposal: dict, *, url: str = "http://10.10.10.1:8084" + PLAN_PATH, timeout: float = 75.0) -> dict:
    return _submit(proposal, url=url, timeout=timeout, path=PLAN_PATH, mode=PLAN_MODE, success="PLAN_READY")


def submit_execution(proposal: dict, *, url: str = "http://10.10.10.1:8084" + EXECUTE_PATH, timeout: float = 150.0) -> dict:
    return _submit(proposal, url=url, timeout=timeout, path=EXECUTE_PATH, mode=EXECUTE_MODE, success="VERIFIED_SUCCESS")


def _submit(proposal, *, url, timeout, path, mode, success):
    """One explicitly typed POST; no redirects, proxies or retries."""
    parsed = urlsplit(url)
    if (
        parsed.scheme not in {"http", "https"} or not parsed.hostname or parsed.path != path
        or parsed.username or parsed.password or parsed.query or parsed.fragment
    ):
        raise ValueError(f"Proposal URL must end in {path}, without credentials or query")
    if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("Proposal HTTP timeout must be finite and positive")
    validate_shape(proposal, expected_mode=mode)
    body = canonical_json(proposal)
    if len(body) > MAX_BODY_BYTES:
        raise ValueError("Dry-run proposal exceeds 64 KiB")
    request = Request(url, data=body, method="POST", headers={
        "Accept": "application/json", "Content-Type": "application/json",
    })
    opener = build_opener(ProxyHandler({}), NoRedirects())
    try:
        try:
            response = opener.open(request, timeout=timeout)
        except HTTPError as error:
            response = error
        with response:
            maximum = 2 * 1024 * 1024 if mode in {PLAN_MODE, EXECUTE_MODE} else MAX_BODY_BYTES
            raw = response.read(maximum + 1)
            status = response.code
            content_type = response.headers.get_content_type()
    except (URLError, TimeoutError, OSError) as error:
        raise RuntimeError(f"Proposal delivery failed; receipt unknown; no automatic retry: {error}") from error
    if len(raw) > maximum or content_type != "application/json":
        raise ValueError("Ubuntu returned an invalid dry-run receipt body")
    receipt = decode_json(raw)
    if not isinstance(receipt, dict) or type(receipt.get("schema_version")) is not int or receipt["schema_version"] != 1:
        raise ValueError("Unsupported dry-run receipt")
    if (
        receipt.get("mode") != mode or receipt.get("execution_allowed") is not False
        or (mode != EXECUTE_MODE and receipt.get("robot_execution") is not False)
    ):
        raise ValueError("Ubuntu did not confirm validation-only mode")
    if receipt.get("request_id") != proposal["request_id"] or receipt.get("proposal_sha256") != digest(body):
        raise ValueError("Dry-run receipt does not match this proposal")
    received_at = receipt.get("received_at_unix_ms")
    if type(received_at) is not int or not -1000 <= int(time.time() * 1000) - received_at <= (timeout + 5.0) * 1000:
        raise ValueError("Dry-run receipt timestamp is stale or in the future; check both computers' clocks")
    valid_decision = (status == 200 and receipt.get("decision") == success) or (
        status in {409, 422, 503} and receipt.get("decision") == "REJECTED"
    )
    if mode == EXECUTE_MODE:
        valid_decision = valid_decision or (status in {500, 503} and receipt.get("decision") == "EXECUTION_FAILED")
        warning = receipt.get("decision") == "EXECUTED_WITH_WARNINGS"
        if warning:
            execution = receipt.get("execution")
            if (status != 200 or receipt.get("robot_execution") is not True
                    or receipt.get("execution_attempted") is not True or not isinstance(execution, dict)
                    or execution.get("status") != "EXECUTED_WITH_WARNINGS"
                    or execution.get("verification_mode") != "report-only"
                    or execution.get("moveit_execution_result") != "SUCCESS"
                    or not isinstance(execution.get("verification"), dict)
                    or execution["verification"].get("status") != "FAILED"):
                raise ValueError("Execution warning receipt did not confirm report-only controller completion")
            valid_decision = True
        if receipt.get("decision") == "VERIFIED_SUCCESS" and (
            receipt.get("robot_execution") is not True or receipt.get("execution_attempted") is not True
            or not isinstance(receipt.get("execution"), dict) or receipt["execution"].get("status") != "VERIFIED_SUCCESS"
            or not isinstance(receipt["execution"].get("verification"), dict)
            or receipt["execution"]["verification"].get("status") != "PASSED"
        ):
            raise ValueError("Execution receipt did not confirm measured verification")
    if not valid_decision:
        raise ValueError(f"Unexpected dry-run receipt decision or HTTP status {status}")
    return receipt


def get_plan_status(request_id: str, *, base_url: str = "http://10.10.10.1:8084", timeout: float = 3.0) -> dict:
    import uuid

    if str(uuid.UUID(request_id)) != request_id:
        raise ValueError("Expected a canonical request UUID")
    parsed = urlsplit(base_url)
    if (parsed.scheme not in {"http", "https"} or not parsed.hostname or parsed.path not in {"", "/"}
            or parsed.username or parsed.password or parsed.query or parsed.fragment):
        raise ValueError("Plan status base URL must contain only the HTTP(S) host and port")
    if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("Status timeout must be finite and positive")
    url = base_url.rstrip("/") + "/v1/action-proposals/status/" + request_id
    try:
        with build_opener(ProxyHandler({}), NoRedirects()).open(Request(url, method="GET"), timeout=timeout) as response:
            raw = response.read(MAX_BODY_BYTES + 1)
            if response.headers.get_content_type() != "application/json" or len(raw) > MAX_BODY_BYTES:
                raise ValueError("Invalid plan status response")
    except (URLError, OSError, TimeoutError) as error:
        raise RuntimeError(f"Cannot read plan status: {error}") from error
    result = decode_json(raw)
    if (not isinstance(result, dict) or result.get("request_id") != request_id
            or result.get("mode") != PLAN_MODE or result.get("remote_execution_available") is not False):
        raise ValueError("Plan status does not match this request or endpoint")
    return result
