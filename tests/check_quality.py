#!/usr/bin/env python3
"""Reject diagnostics beyond the reviewed legacy baseline; do not suppress them."""

import argparse
from collections import Counter
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def location(file, line):
    path = (ROOT / file).resolve()
    try:
        relative = str(path.relative_to(ROOT))
    except ValueError:
        relative = str(path)
    context = path.read_text().splitlines()[int(line) - 1].strip()
    return {"file": relative, "source": context}


def compiler_diagnostics(log):
    diagnostics = []
    pattern = re.compile(r"^(.+?):(\d+):\d+: warning: (.+) \[(-W[^\]]+)\]$")
    for line in log.splitlines():
        if "warning:" not in line:
            continue
        match = pattern.match(line)
        if not match:
            raise ValueError("Unrecognized compiler warning: " + line)
        file, number, message, option = match.groups()
        diagnostics.append(dict(location(file, number), id=option, message=message))
    return diagnostics


def static_diagnostics(xml):
    document = ET.fromstring(xml)
    if document.tag != "results" or document.find("errors") is None:
        raise ValueError("Cppcheck did not return a results document")
    diagnostics = []
    seen = set()
    for error in document.findall("./errors/error"):
        at = error.find("location")
        if at is None:
            raise ValueError("Cppcheck diagnostic has no location: " + str(error.attrib))
        key = (at.get("file"), at.get("line"), error.get("id"), error.get("msg"))
        if key in seen:
            continue
        seen.add(key)
        diagnostics.append(dict(location(at.get("file"), at.get("line")),
                                id=error.get("id"), message=error.get("msg")))
    return diagnostics


def unexpected(actual, allowed):
    encode = lambda item: json.dumps(item, sort_keys=True)
    extra = Counter(map(encode, actual)) - Counter(map(encode, allowed))
    return [json.loads(item) for item in extra.elements()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("warnings", "static"))
    parser.add_argument("--compiler", default="gcc")
    parser.add_argument("--cppcheck", default="cppcheck")
    parser.add_argument("--base", default="HEAD")
    parser.add_argument("--file", action="append", help="Override static-analysis inputs")
    args = parser.parse_args()
    subprocess.run(["git", "diff", "--check", args.base], cwd=ROOT, check=True)
    subprocess.run(["git", "diff", "--cached", "--check"], cwd=ROOT, check=True)
    baseline = json.loads((ROOT / "tests/diagnostics.json").read_text())
    output = ROOT / "build/quality"
    output.mkdir(parents=True, exist_ok=True)
    failures = []
    if args.mode == "warnings":
        version = subprocess.check_output([args.compiler, "--version"], text=True)
        family = "clang" if "clang" in version.lower() else "gcc"
        for target in ("release", "debug", "test-unit"):
            result = subprocess.run(["make", target, "CC=" + args.compiler], cwd=ROOT,
                                    capture_output=True, text=True, timeout=120)
            log = result.stdout + result.stderr
            (output / (family + "-" + target + ".log")).write_text(log)
            if result.returncode:
                print(log)
                return result.returncode
            found = compiler_diagnostics(log)
            failures.extend(unexpected(found, baseline[family]))
            print(f"{family} {target}: {len(found)} diagnostics; "
                  f"{len(unexpected(found, baseline[family]))} new", flush=True)
    else:
        files = args.file or ["rtl_wmbus.c", "tests/test_core.c"]
        command = [args.cppcheck, "--enable=warning,performance,portability", "--std=c99",
                   "--platform=unix64", "--language=c", "--library=posix", "-Iinclude",
                   "--xml", "--xml-version=2"] + files
        result = subprocess.run(command, cwd=ROOT, capture_output=True, timeout=120)
        (output / "cppcheck.xml").write_bytes(result.stderr)
        if result.returncode:
            print((result.stdout + result.stderr).decode(errors="replace"))
            return result.returncode
        found = static_diagnostics(result.stderr)
        failures.extend(unexpected(found, baseline["cppcheck"]))
        print(f"Cppcheck: {len(found)} diagnostics; {len(failures)} new", flush=True)
    if failures:
        print(json.dumps(failures, indent=2))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
