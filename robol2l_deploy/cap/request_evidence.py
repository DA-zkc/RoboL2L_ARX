"""Persist the logical input at model.decide, without credentials or image copies.

This is a host-side input snapshot, not a claim that a provider accepted it or
that it is a network capture. Text is exact; image URLs are hashed and replaced.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
from uuid import uuid4

from .storage import write_json
from .program import compact


def request_digest(messages):
    return hashlib.sha256(json.dumps(messages, ensure_ascii=False, sort_keys=True).encode('utf-8')).hexdigest()


def record_model_call(folder, model, messages, *, phase, attempts, retry):
    request_id = uuid4().hex
    path = Path(folder) / 'model-requests' / (request_id + '.json')
    path.parent.mkdir(parents=True, exist_ok=True)
    archived = deepcopy(messages)
    for message in archived:
        if not isinstance(message.get('content'), list):
            continue
        for block in message['content']:
            if block.get('type') == 'image_url':
                url = block['image_url']['url']
                block['image_url'] = dict(
                    omitted='Image retained in observation archive; URL not recorded here.',
                    url_sha256=hashlib.sha256(url.encode('utf-8')).hexdigest(),
                    url_bytes=len(url.encode('utf-8')),
                    **{k: v for k, v in block['image_url'].items() if k != 'url'})
    # Do not serialize model config, key, headers, environment or arbitrary
    # exception text. Settings here are an explicit non-secret allowlist.
    settings = getattr(model, 'settings', {})
    settings = settings if isinstance(settings, dict) else {}
    if 'reasoning' not in settings and isinstance(getattr(model, 'reasoning', None), str):
        settings = dict(settings, reasoning=model.reasoning)
    record = dict(request_id=request_id, phase=phase, attempts=attempts, retry=retry,
                  started_s=time.time(), status='calling',
                  boundary='logical_messages_at_model_decide_not_network_capture',
                  messages_sha256=request_digest(messages), messages=archived,
                  model={k: settings[k] for k in ('model', 'provider', 'reasoning', 'transport') if k in settings})
    write_json(path, record)
    try:
        value = model.decide(messages)
    except Exception as error:
        record.update(status='error', finished_s=time.time(), error_type=type(error).__name__)
        write_json(path, record)
        raise
    record.update(status='returned', finished_s=time.time(), response=compact(value))
    write_json(path, record)
    return value
