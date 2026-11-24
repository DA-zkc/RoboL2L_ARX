"""Small atomic JSON writer for CaP session artifacts."""
import json
import os
from pathlib import Path


def write_json(path,value):
    path=Path(path);temporary=path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.flush();os.fsync(stream.fileno())
    temporary.replace(path)
