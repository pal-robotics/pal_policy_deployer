#!/usr/bin/env python3
"""Push local policy models to, or pull them from, the GitLab model registry.

Usage (GITLAB_TOKEN must be set: api scope to push, read_api to pull):
    handle_models.py --push <model> [--robot-type T] [--stable] [--override]
    handle_models.py --push --robot-type T [--stable] [--override]
    handle_models.py --pull <model> [--robot-type T] [--stable]
    handle_models.py --pull --all --robot-type T [--stable]

<model> is a model directory below the package's models/ directory, given as
<robot_type>/<task>/<task>_v<N>_mjlab, as <task>_v<N>_mjlab together with --robot-type, or
as any path to such a directory; a --robot-type given with a path must match it. It maps to
the registry model <stage>/<robot_type>_<task>, version <N>.0.0, where <stage> is staging/ by
default and stable/ with --stable.

--push uploads every file of the model directory. Versions that already exist upstream are
skipped; with --override, files whose checksum differs from upstream are re-uploaded (GitLab
keeps the old copy and serves the newest; nothing is deleted upstream).
Without <model>, every local model of --robot-type is pushed that way.
With --stable, the package's models.yaml manifest is updated: the model's entry is added or
replaced whenever the upstream version matches the local files afterwards (new, overridden or
already identical); an existing stable version with different files leaves it untouched.

--pull downloads the files of one model version into its model directory, verifying their
SHA256; files that already match are skipped, and a local file that differs from upstream
stops the pull before anything is downloaded. --all pulls every upstream version of
--robot-type whose model directory does not exist locally; versions that are not <N>.0.0
are skipped.

Exits non-zero on invalid arguments, a missing token, a missing upstream version, a
differing local file, a checksum mismatch or a registry error.
"""

import argparse
import hashlib
import json
import os
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

import yaml

DEFAULT_REGISTRY_URL = "https://gitlab.pal-robotics.com/api/v4/projects/control%2Fpal_policy_deployer"
MODELS_DIR = Path(__file__).resolve().parents[1] / "models"
# Tasks must not contain '_' so that a registry name <robot_type>_<task> can be split back.
MODEL_DIR_RE = re.compile(r"(?P<task>[^_/]+)_v(?P<number>\d+)_mjlab")


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def resolve_model(model, robot_type, models_dir):
    """Return the model directory relative to models_dir, as <robot_type>/<task>/<task>_v<N>_mjlab.

    :raises SystemExit: if the robot type cannot be determined or the name does not follow
        the convention.
    """
    path = Path(model)
    resolved = path.resolve()
    if models_dir.resolve() in resolved.parents:
        path = resolved.relative_to(models_dir.resolve())
    if len(path.parts) == 1:
        if not robot_type:
            sys.exit(f"Cannot tell the robot type of '{model}'; pass --robot-type (e.g. lower_body)")
        match = MODEL_DIR_RE.fullmatch(path.name)
        if not match:
            sys.exit(f"'{model}' is not named <task>_v<N>_mjlab")
        path = Path(robot_type, match["task"], path.name)
    if len(path.parts) != 3:
        sys.exit(f"'{model}' is not <robot_type>/<task>/<task>_v<N>_mjlab below {models_dir}")
    if robot_type and path.parts[0] != robot_type:
        sys.exit(f"'{model}' is a {path.parts[0]} model, but --robot-type is {robot_type}")
    registry_name_and_version(path)  # validates the naming
    return path


def registry_name_and_version(rel):
    """Map <robot_type>/<task>/<task>_v<N>_mjlab to ("<robot_type>_<task>", "<N>.0.0")."""
    robot_type, task, leaf = rel.parts
    match = MODEL_DIR_RE.fullmatch(leaf)
    if not match or match["task"] != task:
        sys.exit(f"'{rel}' is not <robot_type>/<task>/<task>_v<N>_mjlab")
    return f"{robot_type}_{task}", f"{match['number']}.0.0"


def local_model_dirs(models_dir, robot_type):
    """Return (conventional model dirs, other dirs holding files) below models_dir/robot_type."""
    root = models_dir / robot_type
    models, ignored = [], []
    for d in sorted(p for p in [root, *root.rglob("*")] if p.is_dir()):
        rel = d.relative_to(models_dir)
        if len(rel.parts) == 3 and MODEL_DIR_RE.fullmatch(rel.parts[2]) and \
                MODEL_DIR_RE.fullmatch(rel.parts[2])["task"] == rel.parts[1]:
            models.append(rel)
        elif any(p.is_file() for p in d.iterdir()) and not any(m in rel.parents for m in models):
            ignored.append(rel)
    return models, ignored


class Registry:
    """The few GitLab model registry calls this script needs."""

    def __init__(self, url, token):
        self.url = url
        self.mlflow = f"{url}/ml/mlflow/api/2.0/mlflow"
        # The ml_models endpoints reject PRIVATE-TOKEN; Bearer works for every call used here.
        self.auth = {"Authorization": f"Bearer {token}"}

    def _request(self, method, url, data=None, content_type=None):
        """Return (body bytes, headers); raises urllib.error.HTTPError."""
        headers = dict(self.auth, **({"Content-Type": content_type} if content_type else {}))
        req = urllib.request.Request(url, data=data, method=method, headers=headers)
        with urllib.request.urlopen(req) as reply:
            return reply.read(), reply.headers

    def _json(self, method, url, payload=None):
        data = json.dumps(payload).encode() if payload is not None else None
        body, _ = self._request(method, url, data, "application/json" if data else None)
        return json.loads(body or b"{}")

    def _paginated(self, path, **query):
        items, page = [], "1"
        while page:
            body, headers = self._request(
                "GET", f"{self.url}/{path}?{urllib.parse.urlencode(dict(query, per_page=100, page=page))}")
            items += json.loads(body)
            page = headers.get("X-Next-Page")
        return items

    def version_id(self, model, version):
        """Return the version_id of model/version, or None if it does not exist."""
        query = urllib.parse.urlencode({"name": model, "version": version})
        try:
            reply = self._json("GET", f"{self.mlflow}/model-versions/get?{query}")
        except urllib.error.HTTPError as e:
            if e.code == 404:
                return None
            raise
        # Only exposed in the version page URL: .../-/ml/models/<model_id>/versions/<version_id>
        return reply["model_version"]["source"].rstrip("/").rsplit("/", 1)[1]

    def create_version(self, model, version, description):
        """Create model (if needed) and version; return the new version_id."""
        try:
            self._json("GET", f"{self.mlflow}/registered-models/get?{urllib.parse.urlencode({'name': model})}")
        except urllib.error.HTTPError as e:
            if e.code != 404:
                raise
            self._json("POST", f"{self.mlflow}/registered-models/create", {"name": model})
        reply = self._json("POST", f"{self.mlflow}/model-versions/create", {
            "name": model,
            "source": "pal_policy_deployer",
            "description": description,
            "tags": [{"key": "gitlab.version", "value": version}],
        })
        return reply["model_version"]["source"].rstrip("/").rsplit("/", 1)[1]

    def versions(self, name_prefix):
        """Return [(model, version)] of all upstream versions whose model name starts with name_prefix."""
        # package_name is a substring filter, so compare exactly.
        return [(p["name"], p["version"])
                for p in self._paginated("packages", package_type="ml_model", package_name=name_prefix)
                if p["name"].startswith(name_prefix)]

    def files(self, model, version):
        """Return {file name: sha256} of a model version, or None if it does not exist."""
        packages = [p for p in self._paginated("packages", package_type="ml_model", package_name=model,
                                               package_version=version)
                    if p["name"] == model and p["version"] == version]
        if not packages:
            return None
        files = {}
        # Re-uploads keep the old file; the newest one with a name is what gets served.
        for f in sorted(self._paginated(f"packages/{packages[0]['id']}/package_files"), key=lambda f: f["id"]):
            files[f["file_name"]] = f["file_sha256"]
        return files

    def _file_url(self, version_id, name):
        return f"{self.url}/packages/ml_models/{version_id}/files/{urllib.parse.quote(name)}"

    def upload(self, version_id, path):
        self._request("PUT", self._file_url(version_id, path.name), path.read_bytes())

    def download(self, version_id, name):
        return self._request("GET", self._file_url(version_id, name))[0]


def manifest_entry(rel, model, version, version_id, files):
    """Return the models.yaml entry text of one stable version."""
    lines = [f"{rel.as_posix()}:", f"  model: {model}", f"  version: {version}",
             f"  version_id: {version_id}", "  files:"]
    lines += [f"    {name}: {digest}" for name, digest in sorted(files.items())]
    return "\n".join(lines) + "\n"


def update_manifest(path, rel, model, version, version_id, files):
    """Add or replace the entry of rel in the models.yaml manifest at path.

    The file is rewritten from its entries, sorted by directory; the leading comment block is
    kept, comments inside entries are not. The file is created if it does not exist.
    """
    text = path.read_text() if path.exists() else ""
    header = []
    for line in text.splitlines(keepends=True):
        if line.strip() and not line.startswith("#"):
            break
        header.append(line)
    header = "".join(header).rstrip("\n")

    entries = yaml.safe_load(text) or {}
    entries[rel.as_posix()] = {"model": model, "version": version, "version_id": version_id, "files": files}
    body = "\n".join(manifest_entry(Path(d), e.get("model", ""), e.get("version", ""), e["version_id"], e["files"])
                     for d, e in sorted(entries.items()))
    path.write_text((header + "\n\n" if header else "") + body)
    print(f"Updated {path.name}: {rel}")


def push(registry, models_dir, rel, stage, override):
    """Push one model directory; return True if anything was uploaded.

    For stable/, the models.yaml next to models_dir is updated whenever the upstream version
    matches the local files afterwards (new, overridden, or already identical); if it exists
    upstream with different files and override is False, the manifest is left alone.
    """
    name, version = registry_name_and_version(rel)
    model = stage + name
    local = {p.name: p for p in sorted((models_dir / rel).iterdir()) if p.is_file()}
    if not local:
        sys.exit(f"{models_dir / rel} has no files to push")
    digests = {n: sha256(p.read_bytes()) for n, p in local.items()}
    stable = stage == "stable/"

    def record():
        if stable:
            update_manifest(models_dir.parent / "models.yaml", rel, model, version, int(version_id), digests)

    version_id = registry.version_id(model, version)
    if version_id is None:
        version_id = registry.create_version(model, version, rel.name)
        to_upload = list(local)
    elif not override and not stable:
        print(f"Skipping {rel}: {model} {version} already exists upstream (use --override to update it)")
        return False
    else:
        upstream = registry.files(model, version) or {}
        to_upload = [n for n in local if upstream.get(n) != digests[n]]
        if not override:
            print(f"Skipping {rel}: {model} {version} already exists upstream (use --override to update it)")
            if to_upload or set(upstream) != set(local):
                print(f"Warning: {rel} differs from {model} {version}; not updating models.yaml")
            else:
                record()
            return False
        for n in sorted(set(upstream) - set(local)):
            print(f"Warning: {model} {version} has {n}, which is not in {rel}; it stays upstream")
        if not to_upload:
            print(f"{model} {version} is up to date")
            record()
            return False

    for n in to_upload:
        print(f"Uploading {rel / n}")
        registry.upload(version_id, local[n])
    print(f"Pushed {model} {version} (version_id {version_id})")
    record()
    return True


def pull(registry, models_dir, rel, stage):
    """Pull one model version into models_dir/rel."""
    name, version = registry_name_and_version(rel)
    model = stage + name
    version_id = registry.version_id(model, version)
    upstream = registry.files(model, version) if version_id else None
    if upstream is None:
        sys.exit(f"{model} {version} not found upstream")

    dst = models_dir / rel
    missing = {}
    for n, digest in upstream.items():
        path = dst / n
        if not path.exists():
            missing[n] = digest
        elif sha256(path.read_bytes()) != digest:
            sys.exit(f"{path} differs from {model} {version}; not overwriting it")
    if not missing:
        print(f"{rel} is up to date with {model} {version}")
        return

    dst.mkdir(parents=True, exist_ok=True)
    for n, digest in sorted(missing.items()):
        print(f"Downloading {rel / n}")
        data = registry.download(version_id, n)
        if sha256(data) != digest:
            sys.exit(f"SHA256 mismatch for {n} of {model} {version}; not saved")
        (dst / n).write_bytes(data)
    print(f"Pulled {model} {version} into {dst}")


def pull_all(registry, models_dir, robot_type, stage):
    """Pull every upstream version of robot_type whose model directory is missing locally."""
    prefix = f"{stage}{robot_type}_"
    for model, version in sorted(registry.versions(prefix)):
        task = model[len(prefix):]
        number = re.fullmatch(r"(\d+)\.0\.0", version)
        # A '_' left in the task means another robot type sharing this prefix (full_body vs full_body_4dof).
        if "_" in task:
            continue
        if not number:
            print(f"Skipping {model} {version}: only <N>.0.0 versions map to a model directory")
            continue
        rel = Path(robot_type, task, f"{task}_v{number[1]}_mjlab")
        if (models_dir / rel).exists():
            print(f"Skipping {rel}: exists locally")
            continue
        pull(registry, models_dir, rel, stage)


def main(argv=None, models_dir=MODELS_DIR):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--push", action="store_true", help="upload local models to the registry")
    action.add_argument("--pull", action="store_true", help="download models from the registry")
    parser.add_argument("model", nargs="?", help="<task>_v<N>_mjlab or <robot_type>/<task>/<task>_v<N>_mjlab")
    parser.add_argument("--robot-type", help="e.g. lower_body, full_body_4dof")
    parser.add_argument("--stable", action="store_true", help="use stable/ models instead of staging/")
    parser.add_argument("--all", action="store_true", help="with --pull: all upstream models missing locally")
    parser.add_argument("--override", action="store_true", help="with --push: re-upload files that differ upstream")
    parser.add_argument("--registry-url", default=DEFAULT_REGISTRY_URL, help="GitLab project API URL")
    args = parser.parse_args(argv)

    if args.all and not args.pull:
        parser.error("--all only works with --pull")
    if args.override and not args.push:
        parser.error("--override only works with --push")
    if args.pull and bool(args.model) == args.all:
        parser.error("--pull needs either a model or --all")
    if args.all and not args.robot_type:
        parser.error("--all needs --robot-type")
    if args.push and not (args.model or args.robot_type):
        parser.error("--push needs a model or --robot-type")
    token = os.environ.get("GITLAB_TOKEN")
    if not token:
        sys.exit("GITLAB_TOKEN is not set (api scope to push, read_api to pull)")

    registry = Registry(args.registry_url, token)
    stage = "stable/" if args.stable else "staging/"
    try:
        if args.pull and args.all:
            pull_all(registry, models_dir, args.robot_type, stage)
        elif args.pull:
            pull(registry, models_dir, resolve_model(args.model, args.robot_type, models_dir), stage)
        elif args.model:
            push(registry, models_dir, resolve_model(args.model, args.robot_type, models_dir), stage, args.override)
        else:
            rels, ignored = local_model_dirs(models_dir, args.robot_type)
            for rel in ignored:
                print(f"Ignoring {rel}: not <robot_type>/<task>/<task>_v<N>_mjlab")
            if not rels:
                sys.exit(f"No local models found in {models_dir / args.robot_type}")
            pushed = sum(push(registry, models_dir, rel, stage, args.override) for rel in rels)
            print(f"Pushed {pushed} of {len(rels)} {args.robot_type} models to {stage}")
    except urllib.error.HTTPError as e:
        sys.exit(f"Registry error {e.code} for {e.url}: {e.read().decode(errors='replace')}")


if __name__ == "__main__":
    main()
