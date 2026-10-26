# Offline C4 ONNX Runtime bundle

Official PyPI binary wheels for Linux AArch64, CPython 3.12. Runtime 1.22.1 plus
coloredlogs, humanfriendly, flatbuffers, packaging, protobuf, sympy and mpmath.
NumPy is supplied by openpilot's existing environment. Exact versions, filenames,
SHA256 and sizes are in manifest.json. Original wheel metadata/licenses remain
inside each archive and are retained when installed. No package is imported into
the primary UI process from this bundle; only the low-priority detector worker
adds its isolated runtime directory to sys.path.

Startup prepares this from local files under /data/extra/lead-brake-runtime.
First boot extracts/validates once and atomically activates a versioned directory.
Subsequent boots check the small marker and critical file sizes and skip setup.
A bundle change, missing/corrupt critical file, or import-failure marker causes
repair on the next boot. No network, pip, API key or manual bash is required.
The primary Python environment and uv.lock are not changed. Wheel hashes are
verified before preparation; native execution on C4 still needs device validation.
