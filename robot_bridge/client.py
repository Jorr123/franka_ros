"""Send one validation-only proposal; check the matching Ubuntu receipt."""

from __future__ import annotations

import math
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

from .protocol import (
    MAX_BODY_BYTES, MODE, PATH, canonical_json, decode_json,
    digest, validate_shape,
)

DEFAULT_URL = "http://10.10.10.1:8083" + PATH


class NoRedirects(HTTPRedirectHandler):
    def redirect_request(self, request, response, code, message, headers, new_url):
        return None


def submit_proposal(proposal: dict, *, url: str = DEFAULT_URL, timeout: float = 3.0) -> dict:
    """One POST, no redirects/proxies/retries, and no execution endpoint support."""
    parsed = urlsplit(url)
    if (
        parsed.scheme not in {"http", "https"} or not parsed.hostname or parsed.path != PATH
        or parsed.username or parsed.password or parsed.query or parsed.fragment
    ):
        raise ValueError(f"Dry-run URL must end in {PATH}, without credentials or query")
    if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("Dry-run HTTP timeout must be finite and positive")
    validate_shape(proposal)
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
            raw = response.read(MAX_BODY_BYTES + 1)
            status = response.code
            content_type = response.headers.get_content_type()
    except (URLError, TimeoutError, OSError) as error:
        raise RuntimeError(f"Dry-run delivery failed; receipt unknown; no automatic retry: {error}") from error
    if len(raw) > MAX_BODY_BYTES or content_type != "application/json":
        raise ValueError("Ubuntu returned an invalid dry-run receipt body")
    receipt = decode_json(raw)
    if not isinstance(receipt, dict) or type(receipt.get("schema_version")) is not int or receipt["schema_version"] != 1:
        raise ValueError("Unsupported dry-run receipt")
    if (
        receipt.get("mode") != MODE or receipt.get("execution_allowed") is not False
        or receipt.get("robot_execution") is not False
    ):
        raise ValueError("Ubuntu did not confirm validation-only mode")
    if receipt.get("request_id") != proposal["request_id"] or receipt.get("proposal_sha256") != digest(body):
        raise ValueError("Dry-run receipt does not match this proposal")
    received_at = receipt.get("received_at_unix_ms")
    if type(received_at) is not int or not -1000 <= int(time.time() * 1000) - received_at <= (timeout + 5.0) * 1000:
        raise ValueError("Dry-run receipt timestamp is stale or in the future; check both computers' clocks")
    valid_decision = (status == 200 and receipt.get("decision") == "ACCEPTED_FOR_DRY_RUN") or (
        status in {409, 422, 503} and receipt.get("decision") == "REJECTED"
    )
    if not valid_decision:
        raise ValueError(f"Unexpected dry-run receipt decision or HTTP status {status}")
    return receipt
