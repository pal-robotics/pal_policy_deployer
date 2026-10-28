"""Tests scripts/handle_models.py against a fake GitLab model registry served on localhost."""

import contextlib
import hashlib
import importlib.util
import io
import json
import os
import threading
import unittest
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from tempfile import TemporaryDirectory

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "handle_models.py"
spec = importlib.util.spec_from_file_location("handle_models", SCRIPT)
hm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hm)

PROJECT = "/api/v4/projects/1"
MLFLOW = PROJECT + "/ml/mlflow/api/2.0/mlflow"


def sha(data):
    return hashlib.sha256(data).hexdigest()


MANIFEST_HEADER = "# Stable policy models.\n# Only stable/ versions belong here.\n"
# Sorts before lower_body/..., so it must stay first.
OTHER_ENTRY = ("full_body_4dof/loco/loco_v700_mjlab:\n  model: stable/full_body_4dof_loco\n"
               "  version: 700.0.0\n  version_id: 25\n  files:\n    a.onnx: " + "a" * 64 + "\n")


class FakeGitLab(BaseHTTPRequestHandler):
    """In-memory registry. Every list is served one item per page to exercise pagination."""

    models = set()
    versions = {}   # (model, version) -> {"vid", "pid", "files": [(file_id, name, bytes)]}
    requests = []   # (method, path, query, body, auth)
    corrupt_downloads = False

    @classmethod
    def reset(cls):
        cls.models, cls.versions, cls.requests, cls.corrupt_downloads = set(), {}, [], False

    @classmethod
    def add_version(cls, model, version, files):
        """Seed an upstream version with {name: bytes}."""
        cls.models.add(model)
        n = len(cls.versions) + 1
        cls.versions[(model, version)] = {
            "vid": 100 + n, "pid": 200 + n,
            "files": [(i, name, data) for i, (name, data) in enumerate(files.items())]}

    def _handle(self, method):
        url = urllib.parse.urlsplit(self.path)
        path, query = url.path, dict(urllib.parse.parse_qsl(url.query))
        body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
        FakeGitLab.requests.append((method, path, query, body, self.headers.get("Authorization")))

        if path == MLFLOW + "/registered-models/get":
            return self._json(200 if query["name"] in self.models else 404, {})
        if path == MLFLOW + "/registered-models/create":
            FakeGitLab.models.add(json.loads(body)["name"])
            return self._json(200, {})
        if path == MLFLOW + "/model-versions/get":
            v = self.versions.get((query["name"], query["version"]))
            if not v:
                return self._json(404, {"error_code": "RESOURCE_DOES_NOT_EXIST"})
            return self._json(200, {"model_version": {
                "source": f"https://gitlab.example.com/g/p/-/ml/models/9/versions/{v['vid']}"}})
        if path == MLFLOW + "/model-versions/create":
            req = json.loads(body)
            tags = {t["key"]: t["value"] for t in req["tags"]}
            FakeGitLab.add_version(req["name"], tags["gitlab.version"], {})
            v = self.versions[(req["name"], tags["gitlab.version"])]
            return self._json(200, {"model_version": {
                "source": f"https://gitlab.example.com/g/p/-/ml/models/9/versions/{v['vid']}"}})
        if path == PROJECT + "/packages":
            # Like GitLab: package_name is a substring match, package_version exact.
            items = [{"id": v["pid"], "name": m, "version": ver} for (m, ver), v in self.versions.items()
                     if query.get("package_name", "") in m
                     and query.get("package_version", ver) == ver]
            return self._page(items, query)
        for (m, ver), v in self.versions.items():
            if path == f"{PROJECT}/packages/{v['pid']}/package_files":
                return self._page([{"id": i, "file_name": n, "file_sha256": sha(d)} for i, n, d in v["files"]], query)
            prefix = f"{PROJECT}/packages/ml_models/{v['vid']}/files/"
            if path.startswith(prefix):
                name = urllib.parse.unquote(path[len(prefix):])
                if method == "PUT":
                    v["files"].append((len(v["files"]), name, body))
                    return self._json(201, {"message": "201 Created"})
                latest = [d for _, n, d in v["files"] if n == name]
                if not latest:
                    return self._json(404, {})
                return self._raw(200, b"corrupt" if self.corrupt_downloads else latest[-1])
        return self._json(500, {"unexpected": path})

    def _page(self, items, query):
        page = int(query.get("page", 1))
        headers = {"X-Next-Page": str(page + 1) if page < len(items) else ""}
        return self._json(200, items[page - 1:page], headers)

    def _json(self, code, payload, headers=None):
        self._raw(code, json.dumps(payload).encode(), headers)

    def _raw(self, code, data, headers=None):
        self.send_response(code)
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self._handle("GET")

    def do_POST(self):
        self._handle("POST")

    def do_PUT(self):
        self._handle("PUT")

    def log_message(self, *args):
        pass


class TestResolveModel(unittest.TestCase):

    def test_bare_name_with_robot_type(self):
        self.assertEqual(hm.resolve_model("loco_v534_mjlab", "lower_body", Path("/m")),
                         Path("lower_body/loco/loco_v534_mjlab"))

    def test_full_path(self):
        self.assertEqual(hm.resolve_model("lower_body/loco/loco_v534_mjlab", None, Path("/m")),
                         Path("lower_body/loco/loco_v534_mjlab"))

    def test_path_through_models_dir(self):
        with TemporaryDirectory() as tmp:
            models = Path(tmp) / "models"
            (models / "lower_body/loco/loco_v534_mjlab").mkdir(parents=True)
            self.assertEqual(hm.resolve_model(str(models / "lower_body/loco/loco_v534_mjlab"), None, models),
                             Path("lower_body/loco/loco_v534_mjlab"))

    def test_bare_name_needs_robot_type(self):
        with self.assertRaisesRegex(SystemExit, "--robot-type"):
            hm.resolve_model("loco_v534_mjlab", None, Path("/m"))

    def test_unconventional_name(self):
        with self.assertRaisesRegex(SystemExit, "_v<N>_mjlab"):
            hm.resolve_model("reaching_tiagopro.onnx", "tiagopro", Path("/m"))

    def test_registry_names(self):
        self.assertEqual(hm.registry_name_and_version(Path("full_body_4dof/loco/loco_v700_mjlab")),
                         ("full_body_4dof_loco", "700.0.0"))


class RegistryTestCase(unittest.TestCase):

    def setUp(self):
        FakeGitLab.reset()
        self.server = HTTPServer(("127.0.0.1", 0), FakeGitLab)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.server.server_port}{PROJECT}"
        self.tmp = TemporaryDirectory()
        self.models = Path(self.tmp.name) / "models"
        self.manifest = Path(self.tmp.name) / "models.yaml"
        os.environ["GITLAB_TOKEN"] = "tok"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.tmp.cleanup()

    def run_cli(self, *args):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            hm.main([*args, "--registry-url", self.url], models_dir=self.models)
        return out.getvalue()

    def local_model(self, rel, files):
        d = self.models / rel
        d.mkdir(parents=True, exist_ok=True)
        for name, data in files.items():
            (d / name).write_bytes(data)
        return d

    def writes(self):
        return [r for r in FakeGitLab.requests if r[0] in ("POST", "PUT")]

    def uploads(self):
        return sorted(r[1].rsplit("/", 1)[1] for r in FakeGitLab.requests if r[0] == "PUT")


class TestPush(RegistryTestCase):

    def test_new_model_goes_to_staging_by_default(self):
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"onnx", "env.yaml": b"e"})

        self.run_cli("--push", "loco_v534_mjlab", "--robot-type", "lower_body")

        self.assertIn("staging/lower_body_loco", FakeGitLab.models)
        self.assertIn(("staging/lower_body_loco", "534.0.0"), FakeGitLab.versions)
        self.assertEqual(self.uploads(), ["env.yaml", "p.onnx"])
        self.assertTrue(all(r[4] == "Bearer tok" for r in FakeGitLab.requests))

    def test_existing_version_is_skipped(self):
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"new"})
        FakeGitLab.add_version("staging/lower_body_loco", "534.0.0", {"p.onnx": b"old"})

        out = self.run_cli("--push", "lower_body/loco/loco_v534_mjlab")

        self.assertIn("already exists", out)
        self.assertEqual(self.writes(), [])

    def test_override_uploads_only_changed_files(self):
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"new", "env.yaml": b"same"})
        FakeGitLab.add_version("staging/lower_body_loco", "534.0.0", {"p.onnx": b"old", "env.yaml": b"same"})

        self.run_cli("--push", "lower_body/loco/loco_v534_mjlab", "--override")

        self.assertEqual(self.uploads(), ["p.onnx"])

    def test_override_with_identical_files_changes_nothing(self):
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"same"})
        FakeGitLab.add_version("staging/lower_body_loco", "534.0.0", {"p.onnx": b"same"})

        out = self.run_cli("--push", "lower_body/loco/loco_v534_mjlab", "--override")

        self.assertIn("up to date", out)
        self.assertEqual(self.writes(), [])

    def test_stable_push_adds_manifest_entry_keeping_header_and_order(self):
        self.manifest.write_text(MANIFEST_HEADER + "\n" + OTHER_ENTRY)
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"onnx"})

        self.run_cli("--push", "lower_body/loco/loco_v534_mjlab", "--stable")

        vid = FakeGitLab.versions[("stable/lower_body_loco", "534.0.0")]["vid"]
        self.assertEqual(self.manifest.read_text(), MANIFEST_HEADER + "\n" + OTHER_ENTRY + "\n" + (
            "lower_body/loco/loco_v534_mjlab:\n  model: stable/lower_body_loco\n"
            f"  version: 534.0.0\n  version_id: {vid}\n  files:\n    p.onnx: {sha(b'onnx')}\n"))

    def test_stable_push_creates_missing_manifest(self):
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"onnx"})

        self.run_cli("--push", "lower_body/loco/loco_v534_mjlab", "--stable")

        self.assertIn("lower_body/loco/loco_v534_mjlab:", self.manifest.read_text())

    def test_stable_override_replaces_manifest_checksums(self):
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"new"})
        FakeGitLab.add_version("stable/lower_body_loco", "534.0.0", {"p.onnx": b"old"})
        vid = FakeGitLab.versions[("stable/lower_body_loco", "534.0.0")]["vid"]
        self.manifest.write_text(
            "lower_body/loco/loco_v534_mjlab:\n  model: stable/lower_body_loco\n"
            f"  version: 534.0.0\n  version_id: {vid}\n  files:\n    p.onnx: {sha(b'old')}\n")

        self.run_cli("--push", "lower_body/loco/loco_v534_mjlab", "--stable", "--override")

        text = self.manifest.read_text()
        self.assertIn(f"p.onnx: {sha(b'new')}", text)
        self.assertNotIn(sha(b"old"), text)

    def test_existing_identical_stable_version_is_added_to_manifest(self):
        self.manifest.write_text(MANIFEST_HEADER)
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"same"})
        FakeGitLab.add_version("stable/lower_body_loco", "534.0.0", {"p.onnx": b"same"})

        self.run_cli("--push", "lower_body/loco/loco_v534_mjlab", "--stable")

        self.assertEqual(self.writes(), [])
        self.assertIn(f"p.onnx: {sha(b'same')}", self.manifest.read_text())

    def test_existing_differing_stable_version_leaves_manifest_alone(self):
        self.manifest.write_text(MANIFEST_HEADER)
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"mine"})
        FakeGitLab.add_version("stable/lower_body_loco", "534.0.0", {"p.onnx": b"theirs"})

        out = self.run_cli("--push", "lower_body/loco/loco_v534_mjlab", "--stable")

        self.assertIn("not updating models.yaml", out)
        self.assertEqual(self.manifest.read_text(), MANIFEST_HEADER)

    def test_staging_push_and_pulls_never_touch_manifest(self):
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"onnx"})
        FakeGitLab.add_version("stable/lower_body_loco", "619.0.0", {"s.onnx": b"s"})

        self.run_cli("--push", "lower_body/loco/loco_v534_mjlab")
        self.run_cli("--pull", "lower_body/loco/loco_v619_mjlab", "--stable")
        self.run_cli("--pull", "--all", "--robot-type", "lower_body", "--stable")

        self.assertFalse(self.manifest.exists())

    def test_robot_type_alone_pushes_all_missing_local_models(self):
        self.local_model("lower_body/loco/loco_v315_mjlab", {"a.onnx": b"a"})
        self.local_model("lower_body/imitation/imitation_v472_mjlab", {"b.onnx": b"b"})
        self.local_model("full_body_4dof/loco/loco_v700_mjlab", {"c.onnx": b"c"})
        self.local_model("lower_body/misc", {"x.onnx": b"x"})
        FakeGitLab.add_version("staging/lower_body_loco", "315.0.0", {"a.onnx": b"a"})

        out = self.run_cli("--push", "--robot-type", "lower_body")

        self.assertEqual(self.uploads(), ["b.onnx"])
        self.assertIn("Ignoring", out)
        self.assertIn("lower_body/misc", out)


class TestPull(RegistryTestCase):

    def test_pulls_model_into_models_dir(self):
        FakeGitLab.add_version("staging/lower_body_loco", "534.0.0", {"p.onnx": b"onnx", "env.yaml": b"e"})

        self.run_cli("--pull", "loco_v534_mjlab", "--robot-type", "lower_body")

        d = self.models / "lower_body/loco/loco_v534_mjlab"
        self.assertEqual((d / "p.onnx").read_bytes(), b"onnx")
        self.assertEqual((d / "env.yaml").read_bytes(), b"e")

    def test_pull_from_stable(self):
        FakeGitLab.add_version("stable/lower_body_loco", "534.0.0", {"p.onnx": b"stable"})
        FakeGitLab.add_version("staging/lower_body_loco", "534.0.0", {"p.onnx": b"staging"})

        self.run_cli("--pull", "lower_body/loco/loco_v534_mjlab", "--stable")

        self.assertEqual((self.models / "lower_body/loco/loco_v534_mjlab/p.onnx").read_bytes(), b"stable")

    def test_matching_local_files_are_not_downloaded_again(self):
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"onnx"})
        FakeGitLab.add_version("staging/lower_body_loco", "534.0.0", {"p.onnx": b"onnx", "env.yaml": b"e"})

        self.run_cli("--pull", "lower_body/loco/loco_v534_mjlab")

        downloads = [r[1] for r in FakeGitLab.requests if "/packages/ml_models/" in r[1]]
        self.assertEqual([d.rsplit("/", 1)[1] for d in downloads], ["env.yaml"])

    def test_differing_local_file_is_not_overwritten(self):
        self.local_model("lower_body/loco/loco_v534_mjlab", {"p.onnx": b"mine"})
        FakeGitLab.add_version("staging/lower_body_loco", "534.0.0", {"p.onnx": b"theirs", "env.yaml": b"e"})

        with self.assertRaisesRegex(SystemExit, "differs"):
            self.run_cli("--pull", "lower_body/loco/loco_v534_mjlab")

        d = self.models / "lower_body/loco/loco_v534_mjlab"
        self.assertEqual((d / "p.onnx").read_bytes(), b"mine")
        self.assertFalse((d / "env.yaml").exists())

    def test_missing_upstream_version(self):
        with self.assertRaisesRegex(SystemExit, "not found"):
            self.run_cli("--pull", "lower_body/loco/loco_v534_mjlab")

    def test_corrupt_download_is_rejected_and_removed(self):
        FakeGitLab.add_version("staging/lower_body_loco", "534.0.0", {"p.onnx": b"onnx"})
        FakeGitLab.corrupt_downloads = True

        with self.assertRaisesRegex(SystemExit, "SHA256"):
            self.run_cli("--pull", "lower_body/loco/loco_v534_mjlab")

        self.assertFalse((self.models / "lower_body/loco/loco_v534_mjlab/p.onnx").exists())

    def test_override_reupload_serves_latest_file(self):
        # GitLab keeps every uploaded file; the newest one with a name wins.
        FakeGitLab.add_version("staging/lower_body_loco", "534.0.0", {"p.onnx": b"old"})
        FakeGitLab.versions[("staging/lower_body_loco", "534.0.0")]["files"].append((1, "p.onnx", b"new"))

        self.run_cli("--pull", "lower_body/loco/loco_v534_mjlab")

        self.assertEqual((self.models / "lower_body/loco/loco_v534_mjlab/p.onnx").read_bytes(), b"new")

    def test_all_pulls_only_missing_models_of_that_robot_type(self):
        FakeGitLab.add_version("staging/lower_body_loco", "315.0.0", {"a.onnx": b"a"})
        FakeGitLab.add_version("staging/lower_body_loco", "534.0.0", {"b.onnx": b"b"})
        FakeGitLab.add_version("staging/lower_body_imitation", "472.0.0", {"c.onnx": b"c"})
        FakeGitLab.add_version("stable/lower_body_loco", "619.0.0", {"s.onnx": b"s"})
        FakeGitLab.add_version("staging/full_body_4dof_loco", "700.0.0", {"d.onnx": b"d"})
        self.local_model("lower_body/loco/loco_v315_mjlab", {"a.onnx": b"a"})

        self.run_cli("--pull", "--all", "--robot-type", "lower_body")

        self.assertTrue((self.models / "lower_body/loco/loco_v534_mjlab/b.onnx").exists())
        self.assertTrue((self.models / "lower_body/imitation/imitation_v472_mjlab/c.onnx").exists())
        self.assertFalse((self.models / "lower_body/loco/loco_v619_mjlab").exists())
        self.assertFalse((self.models / "full_body_4dof").exists())
        downloads = [r[1].rsplit("/", 1)[1] for r in FakeGitLab.requests if "/packages/ml_models/" in r[1]]
        self.assertNotIn("a.onnx", downloads)

    def test_all_with_prefix_of_another_robot_type(self):
        # 'full_body' must not pick up 'full_body_4dof' models.
        FakeGitLab.add_version("staging/full_body_4dof_loco", "700.0.0", {"d.onnx": b"d"})

        self.run_cli("--pull", "--all", "--robot-type", "full_body")

        self.assertFalse(self.models.exists() and any(self.models.rglob("*.onnx")))


class TestArguments(RegistryTestCase):

    def check_error(self, *args):
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            self.run_cli(*args)
        self.assertEqual(FakeGitLab.requests, [])

    def test_push_or_pull_is_required(self):
        self.check_error("loco_v534_mjlab", "--robot-type", "lower_body")

    def test_pull_needs_model_or_all(self):
        self.check_error("--pull", "--robot-type", "lower_body")

    def test_all_excludes_model(self):
        self.check_error("--pull", "loco_v534_mjlab", "--all", "--robot-type", "lower_body")

    def test_robot_type_must_match_full_path(self):
        with self.assertRaisesRegex(SystemExit, "is a lower_body model, but --robot-type is full_body_4dof"):
            self.run_cli("--pull", "lower_body/loco/loco_v534_mjlab", "--robot-type", "full_body_4dof")
        self.assertEqual(FakeGitLab.requests, [])

    def test_all_needs_robot_type(self):
        self.check_error("--pull", "--all")

    def test_push_needs_model_or_robot_type(self):
        self.check_error("--push")

    def test_override_only_with_push(self):
        self.check_error("--pull", "lower_body/loco/loco_v534_mjlab", "--override")

    def test_all_only_with_pull(self):
        self.check_error("--push", "--all", "--robot-type", "lower_body")

    def test_missing_token(self):
        del os.environ["GITLAB_TOKEN"]
        self.check_error("--pull", "lower_body/loco/loco_v534_mjlab")


if __name__ == "__main__":
    unittest.main()
