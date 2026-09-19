#!/usr/bin/env python3
"""
One-shot build of !RTLSDR's RTLSDR binary via build.riscos.online.

Single C file -- no chunking needed (unlike riscos-rdpclient's adaptive
buildapp.py, which exists to split ~71 compiles across the service's
per-request wall-clock cap). This project doesn't need that yet.

Usage:
    python3 build_one_shot.py

Requires the 'websocket-client' package. If not installed here, run with
another interpreter that has it, e.g. riscos-rdpclient's venv:
    ~/Development/riscos-rdpclient/rdpclient/build/.venv/bin/python3 \\
        build_one_shot.py

Reads plan.json for the setup/compile/link commands, zips
../app/!RTLSDR/{c,h} plus a generated .robuild.yaml, submits it to the
build service, and saves the returned RTLSDR binary to ./RTLSDR.
"""
import sys
import os
import json
import base64
import zipfile
import io
import shutil
import subprocess
import tempfile

try:
    import websocket
except ImportError:
    sys.exit("Missing dependency. Run:  python3 -m pip install websocket-client\n"
              "(or use riscos-rdpclient's venv, which already has it -- see "
              "the module docstring)")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PLAN = os.path.join(HERE, "plan.json")
APP_C = os.path.join(ROOT, "app", "!RTLSDR", "c")
APP_H = os.path.join(ROOT, "app", "!RTLSDR", "h")
VIEW_C = os.path.join(ROOT, "app", "!RTLSDRView", "c")
VIEW_H = os.path.join(ROOT, "app", "!RTLSDRView", "h")
# Reuse riscos-rdpclient's prebuilt 32-bit DeskLib (built once via that
# project's builddesklib.py -- compiles all ~516 DeskLib source objects
# with -apcs 3/32bit, since the official prebuilt DeskLib doesn't match
# this toolchain's 32-bit APCS variant) rather than rebuilding DeskLib
# from scratch for this project. See docs/PLAN.md, Phase 2 section.
DESKLIB_HEADERS = os.path.join(
    os.path.expanduser("~"), "Development", "desklib-module-src",
    "!DeskLib", "include")
DESKLIB32_LIB = os.path.join(
    os.path.expanduser("~"), "Development", "riscos-rdpclient",
    "rdpclient", "build", "DeskLib32")
SERVER = "wss://build.riscos.online/ws"
ARCH = "aarch32"
TIMEOUT = 480
ARTIFACT_NAMES = ["RTLSDR", "RTLSDRView"]
# The build service collects artifacts by zipping a DIRECTORY (confirmed
# from riscos-rdpclient/build/buildapp.py: write_yaml(script, "out")) --
# not a single file by name. plan.json's finals step copies the linked
# binary into ./out/ for exactly this reason.
ARTIFACT_DIR = "out"


def load_plan():
    p = json.load(open(PLAN))
    return p["setup"], p["compiles"], p["finals"]


def _riscos_leaf(fn):
    """'RTLSDR.c' -> 'RTLSDR' -- RISC OS leafnames carry no extension; the
    c/ or h/ directory is what tells Norcroft what kind of source it is."""
    return os.path.splitext(fn)[0]


def build_zip():
    setup, compiles, finals = load_plan()
    script = list(setup)
    for c in compiles:
        script.append(c["cmd"])
    script.extend(finals)

    yaml_lines = ["jobs:", "  build:", "    script:"]
    for s in script:
        yaml_lines.append('      - "%s"' % s.replace('"', '\\"'))
    yaml_lines.append("    artifacts:")
    yaml_lines.append("      - path: %s" % ARTIFACT_DIR)
    yaml_text = "\n".join(yaml_lines) + "\n"

    # Stage into a real directory tree and shell out to `zip`, matching
    # riscos-rdpclient/build/buildapp.py's zip_work(): the build service's
    # extractor needs the directory-entry-ordering that Info-ZIP's `zip`
    # produces -- Python's zipfile (file entries only, no dir entries)
    # tripped it ("File exists") in that project, so don't repeat that here.
    stage = tempfile.mkdtemp(prefix="rtlsdr-build-")
    try:
        os.makedirs(os.path.join(stage, "c"))
        os.makedirs(os.path.join(stage, "h"))
        os.makedirs(os.path.join(stage, "desklib", "o"))
        with open(os.path.join(stage, ".robuild.yaml"), "w") as f:
            f.write(yaml_text)
        for fn in os.listdir(APP_C):
            shutil.copyfile(os.path.join(APP_C, fn),
                             os.path.join(stage, "c", _riscos_leaf(fn)))
        for fn in os.listdir(APP_H):
            shutil.copyfile(os.path.join(APP_H, fn),
                             os.path.join(stage, "h", _riscos_leaf(fn)))
        if os.path.isdir(VIEW_C):
            for fn in os.listdir(VIEW_C):
                shutil.copyfile(os.path.join(VIEW_C, fn),
                                 os.path.join(stage, "c", _riscos_leaf(fn)))
        if os.path.isdir(VIEW_H):
            for fn in os.listdir(VIEW_H):
                shutil.copyfile(os.path.join(VIEW_H, fn),
                                 os.path.join(stage, "h", _riscos_leaf(fn)))

        # DeskLib headers are referenced by DeskLib:Xyz.h -- a full
        # filing-system-prefixed pathname resolved via the DeskLib$Path
        # system variable (set in plan.json's setup), NOT the -I,C:
        # search path that strips extensions -- so these keep their
        # literal dotted names, matching riscos-rdpclient's proven-
        # working desklib/ layout for this same build service.
        for fn in os.listdir(DESKLIB_HEADERS):
            if fn.endswith(".h"):
                shutil.copyfile(os.path.join(DESKLIB_HEADERS, fn),
                                 os.path.join(stage, "desklib", fn))
        shutil.copyfile(DESKLIB32_LIB,
                         os.path.join(stage, "desklib", "o", "DeskLib"))

        fd, zip_path = tempfile.mkstemp(suffix=".zip")
        os.close(fd)
        os.remove(zip_path)
        subprocess.run(["zip", "-rq", zip_path, "."], cwd=stage, check=True)
        data = open(zip_path, "rb").read()
        os.remove(zip_path)
        return data
    finally:
        shutil.rmtree(stage, ignore_errors=True)


def run_build(src_bytes):
    """Protocol mirrors riscos-rdpclient/build/buildapp.py's run_build()."""
    src_b64 = base64.b64encode(src_bytes).decode("ascii")
    ws = websocket.create_connection(
        SERVER, timeout=TIMEOUT + 240,
        header=["Origin: https://build.riscos.online"])
    options = [("timeout", TIMEOUT), ("ansitext", False), ("arch", ARCH)]
    oi = 0
    state = "connecting"
    res = dict(rc=None, throwbacks=0, artifact=None, art_ft=0,
               completed=False, errors=[], service_error=None)

    def send(a, p):
        ws.send(json.dumps([a, p]))

    def pump():
        nonlocal oi, state
        if oi < len(options):
            n, v = options[oi]
            oi += 1
            send("option", [n, v])
        else:
            send("source", src_b64)
            state = "source_sent"

    try:
        while True:
            raw = ws.recv()
            if raw is None or raw == "":
                break
            if isinstance(raw, bytes):
                raw = raw.decode("utf-8", "replace")
            try:
                arr = json.loads(raw)
            except ValueError:
                continue
            if not isinstance(arr, list) or not arr:
                continue
            typ = arr[0]
            data = arr[1] if len(arr) > 1 else None

            if typ == "welcome":
                state = "options"
                pump()
            elif typ == "response":
                if state == "options":
                    pump()
                elif state == "source_sent":
                    send("build", "")
                    state = "build_sent"
                elif state == "build_sent":
                    state = "compiling"
            elif typ == "error":
                if state == "options":
                    pump()
                else:
                    print("  server error:", data)
                    break
            elif typ == "message":
                ds = data if isinstance(data, str) else str(data)
                print("  [message]", ds)
                if "Build failure" in ds or "Errno" in ds or "Traceback" in ds:
                    res["service_error"] = ds.strip()
            elif typ == "output":
                txt = data if isinstance(data, str) else str(data)
                sys.stdout.write(txt)
                sys.stdout.flush()
            elif typ == "throwback":
                res["throwbacks"] += 1
                try:
                    s = "%s %s:%s  %s" % (
                        data.get("severity_name") or "",
                        data.get("filename") or "",
                        data.get("lineno"), data.get("message") or "")
                except AttributeError:
                    s = json.dumps(data)
                print("  [throwback]", s)
                if any(k in s for k in ("Serious", "Error", "error")):
                    res["errors"].append(s.strip())
            elif typ == "clipboard":
                ft = data.get("filetype", 0) if isinstance(data, dict) else 0
                b64 = data.get("data", "") if isinstance(data, dict) else ""
                res["artifact"] = base64.b64decode(b64)
                res["art_ft"] = ft
            elif typ == "rc":
                res["rc"] = data if isinstance(data, int) else 1
            elif typ == "complete":
                res["completed"] = True
                break
    finally:
        try:
            ws.close()
        except Exception:
            pass
    return res


def save_artifact(art_bytes):
    """The service may return the artifact as a raw file or a zip wrapping
    it (possibly with a RISC OS ',xxx' filetype suffix on the member name).
    Handle both. Now two binaries (RTLSDR, RTLSDRView) can come back in
    the same zip -- extract whichever of ARTIFACT_NAMES are present."""
    try:
        with zipfile.ZipFile(io.BytesIO(art_bytes)) as z:
            names = [n for n in z.namelist() if not n.endswith("/")]
            print("artifact zip members:", names)
            found = []
            for n in names:
                leaf = os.path.basename(n.rstrip("/")).split(",")[0]
                if leaf in ARTIFACT_NAMES:
                    data = z.read(n)
                    outp = os.path.join(HERE, leaf)
                    open(outp, "wb").write(data)
                    print("Wrote %s (%d bytes)" % (outp, len(data)))
                    found.append(leaf)
            missing = [a for a in ARTIFACT_NAMES if a not in found]
            if missing:
                print("Not found in artifact zip: %s" % missing)
            return len(found) > 0
    except zipfile.BadZipFile:
        # Raw single-file response -- only meaningful when there's just
        # one artifact name to expect.
        outp = os.path.join(HERE, ARTIFACT_NAMES[0])
        open(outp, "wb").write(art_bytes)
        print("Wrote %s (%d bytes, raw)" % (outp, len(art_bytes)))
        return True


def main():
    print("Building %s via %s" % (", ".join(ARTIFACT_NAMES), SERVER))
    src = build_zip()
    print("Source zip: %d bytes" % len(src))
    res = run_build(src)
    print()
    print("rc=%s throwbacks=%d errors=%d completed=%s" % (
        res["rc"], res["throwbacks"], len(res["errors"]), res["completed"]))
    if res["service_error"]:
        print("SERVICE ERROR:", res["service_error"])
    for e in res["errors"]:
        print("ERROR:", e)

    if res["artifact"]:
        ok = save_artifact(res["artifact"])
        return 0 if ok else 1

    print("No artifact returned -- build likely failed. See output above.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
