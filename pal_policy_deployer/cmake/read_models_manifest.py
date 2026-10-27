"""Print the files of a models.yaml manifest as ``<version_id>|<relpath>|<sha256>`` lines.

Used by pal_policy_models_from_yaml() in fetch_models.cmake, since CMake cannot parse YAML.
Exits non-zero with a message naming the entry if a required key is missing.
"""

import sys

import yaml

with open(sys.argv[1]) as f:
    manifest = yaml.safe_load(f) or {}

for directory, entry in manifest.items():
    try:
        for name, sha256 in entry["files"].items():
            print(f"{entry['version_id']}|{directory}/{name}|{sha256}")
    except (KeyError, TypeError, AttributeError) as e:
        sys.exit(f"{sys.argv[1]}: entry '{directory}' needs 'version_id' and 'files' ({e!r})")
