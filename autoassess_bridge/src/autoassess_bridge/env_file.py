"""Load CDF credentials from a local .env file without logging their values."""

from pathlib import Path


COGNITE_KEYS = frozenset({
    "COGNITE_PROJECT",
    "COGNITE_CLUSTER",
    "COGNITE_TENANT_ID",
    "COGNITE_CLIENT_ID",
    "COGNITE_CLIENT_SECRET",
})


def load_credentials(path, environ):
    """Set COGNITE_* variables in the bridge process from KEY=VALUE lines."""
    for line_number, raw_line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].lstrip()
        if "=" not in line:
            raise ValueError("Invalid credentials file line {}".format(line_number))
        key, value = line.split("=", 1)
        key = key.strip()
        if key not in COGNITE_KEYS:
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1]
        environ[key] = value
