"""Saved task definitions, so a short list of recurring cases is one click away.

Presets are operator text only: a name, the goal, the completion criteria and
which skill library the case expects. Selecting one fills the form; it never
starts anything.
"""
import json
import os
import tempfile
import time
from pathlib import Path
from uuid import uuid4

MAX_PRESETS = 60
MAX_NAME = 120
MAX_TEXT = 4000
FIELDS = ('name', 'goal', 'criteria', 'skills', 'speed', 'record')


class TaskPresets:
    def __init__(self, path):
        self.path = Path(path)

    def read(self):
        if not self.path.is_file():
            return {'presets': []}
        try:
            value = json.loads(self.path.read_text())
        except ValueError:
            return {'presets': []}
        rows = [p for p in value.get('presets', []) if isinstance(p, dict) and p.get('id')]
        return {'presets': rows}

    def public(self):
        rows = sorted(self.read()['presets'], key=lambda p: p.get('used_s') or p.get('saved_s') or 0, reverse=True)
        return {'presets': rows}

    def _write(self, value):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(dir=self.path.parent, prefix='.task-presets-')
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(value, stream, ensure_ascii=False, indent=2)
                stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return self.public()

    def save(self, body):
        """Create or update one preset. A repeated name updates that preset."""
        if set(body) - set(FIELDS) - {'id'}:
            raise ValueError('invalid_task_preset')
        for field in ('name', 'goal', 'criteria'):
            if not isinstance(body.get(field), str) or not body[field].strip():
                raise ValueError('task_preset_name_goal_and_criteria_required')
            if len(body[field]) > (MAX_NAME if field == 'name' else MAX_TEXT):
                raise ValueError('task_preset_text_too_long')
        skills = body.get('skills', '')
        if not isinstance(skills, str) or len(skills) > MAX_NAME:
            raise ValueError('invalid_task_preset')
        record = body.get('record', '')
        if not isinstance(record, str) or len(record) > MAX_NAME:
            raise ValueError('invalid_task_preset')
        speed = body.get('speed', 0.05)
        if not isinstance(speed, (int, float)) or isinstance(speed, bool) or not 0 < speed <= 1:
            raise ValueError('speed_must_be_above_0_and_at_most_1')
        value = self.read()
        rows = value['presets']
        name = body['name'].strip()
        target = None
        if body.get('id'):
            target = next((p for p in rows if p['id'] == body['id']), None)
            if target is None:
                raise ValueError('task_preset_not_found')
        if target is None:
            target = next((p for p in rows if p['name'].casefold() == name.casefold()), None)
        if target is None:
            if len(rows) >= MAX_PRESETS:
                raise ValueError('task_preset_limit_reached')
            target = {'id': uuid4().hex, 'saved_s': time.time()}
            rows.append(target)
        target.update(name=name, goal=body['goal'].strip(), criteria=body['criteria'].strip(),
                      skills=skills.strip(), speed=float(speed), record=record.strip(),
                      updated_s=time.time())
        return self._write(value)

    def touch(self, preset_id):
        """Record that a preset was used, so the list stays ordered by recency."""
        value = self.read()
        row = next((p for p in value['presets'] if p['id'] == preset_id), None)
        if row is None:
            raise ValueError('task_preset_not_found')
        row['used_s'] = time.time()
        return self._write(value)

    def delete(self, body):
        if set(body) - {'id', 'confirmed'} or body.get('confirmed') is not True:
            raise ValueError('explicit_task_preset_delete_confirmation_required')
        value = self.read()
        remaining = [p for p in value['presets'] if p['id'] != body.get('id')]
        if len(remaining) == len(value['presets']):
            raise ValueError('task_preset_not_found')
        value['presets'] = remaining
        return self._write(value)
