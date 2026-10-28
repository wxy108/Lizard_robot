"""Small shared provenance helpers for auditable, non-overwriting runs."""

import hashlib
import json
from pathlib import Path
import subprocess
from datetime import datetime


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def new_run(path=None, prefix="run"):
    destination = Path(path) if path else ROOT/"outputs"/"general_rft"/(prefix+"_"+datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    destination = destination.resolve()
    destination.mkdir(parents=True, exist_ok=False)
    return destination


def git_state():
    def run(*args):
        return subprocess.check_output(["git", *args],cwd=ROOT,text=True).strip()
    try:
        return {"commit":run("rev-parse","HEAD"),"dirty":bool(run("status","--porcelain"))}
    except (OSError, subprocess.CalledProcessError):
        return {"commit":None,"dirty":None}


def write_json(path, payload):
    Path(path).write_text(json.dumps(payload,indent=2,allow_nan=False)+"\n",encoding="utf-8")


def artifact_manifest(folder):
    folder = Path(folder)
    return {path.relative_to(folder).as_posix():{"bytes":path.stat().st_size,"sha256":sha256(path)}
            for path in sorted(folder.rglob("*")) if path.is_file() and path.name != "manifest.json"}
