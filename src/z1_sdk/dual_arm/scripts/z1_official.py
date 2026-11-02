#!/usr/bin/env python3
"""Configure independent upstream controllers; no robot control algorithm here."""
import argparse
import fcntl
import ipaddress
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml

SOURCE_ROOT = Path(r"@Z1_SOURCE_ROOT@")
BUILD_BIN = Path(r"@Z1_BUILD_DIR@")


def locations():
    script = Path(__file__).resolve()
    # Installed prefix, native build, and direct source invocation.
    prefix = script.parent.parent
    installed = prefix / "share/z1_sdk"
    if installed.is_dir():
        return installed / "config/arms.yaml", installed / "upstream_config"
    source = SOURCE_ROOT if SOURCE_ROOT.is_dir() else script.parents[2]
    return source / "dual_arm/config/arms.yaml", source / "controller/config"


def load_config(path):
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if (not isinstance(data, dict) or not isinstance(data.get("arms"), dict)
            or set(data["arms"]) != {"left", "right"}):
        raise ValueError("configuration must define exactly arms.left and arms.right")
    used = set()
    for name, arm in data["arms"].items():
        if not isinstance(arm, dict):
            raise ValueError(f"arms.{name} must be a mapping")
        ipaddress.IPv4Address(arm["mcu_ip"])
        for key in ("mcu_local_port", "controller_port", "sdk_port"):
            value = arm[key]
            if type(value) is not int or not 1 <= value <= 65535:
                raise ValueError(f"{name}.{key}: expected UDP port 1..65535")
            if value in used:
                raise ValueError(f"UDP local port {value} is used more than once")
            used.add(value)
        if type(arm["has_gripper"]) is not bool:
            raise ValueError(f"{name}.has_gripper must be a YAML boolean")
        load = arm["load_kg"]
        if isinstance(load, bool) or not isinstance(load, (int, float)) or not math.isfinite(load) or load < 0:
            raise ValueError(f"{name}.load_kg must be finite and nonnegative")
    return data["arms"]


def binary(name, explicit=None):
    if explicit:
        result = Path(explicit).resolve()
        if not result.is_file():
            raise ValueError(f"executable not found: {result}")
        return result
    script_dir = Path(__file__).resolve().parent
    if (script_dir.parent / "share/z1_sdk").is_dir():
        result = script_dir / name
        if not result.is_file():
            raise ValueError(f"installed executable not found: {result}")
        return result
    for candidate in (script_dir / name, BUILD_BIN / name):
        if candidate.is_file():
            return candidate.resolve()
    found = shutil.which(name)
    if found:
        return Path(found).resolve()
    raise ValueError(f"cannot find {name}; use a built/installed z1_official.py or specify its binary")


def prepare_runtime(root, name, arm, template):
    directory = Path(root).resolve() / name
    config = directory / "config"
    run = directory / "run"
    config.mkdir(parents=True, exist_ok=True)
    run.mkdir(exist_ok=True)
    # The upstream library reads ../config/config.xml from its working directory.
    tree = ET.parse(Path(template) / "config.xml")
    tree.getroot().find("IP").text = arm["mcu_ip"]
    tree.getroot().find("Port").text = str(arm["mcu_local_port"])
    tree.getroot().find("collision/load").text = str(arm["load_kg"])
    tree.write(config / "config.xml", encoding="utf-8", xml_declaration=True)
    states = config / "savedArmStates.csv"
    if not states.exists():
        shutil.copyfile(Path(template) / "savedArmStates.csv", states)
    return run


def stop_children(children):
    # Only processes created by this launcher are signalled.
    for child in children:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)  # upstream main handles SIGINT
    deadline = time.monotonic() + 4
    for child in children:
        try:
            child.wait(timeout=max(0.01, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            child.terminate()
    for child in children:
        try:
            child.wait(timeout=1)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()


def controllers(names, arms, root, template, executable):
    children, locks = [], []
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    old_signals = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for name in names:
            directory = Path(root).resolve() / name
            directory.mkdir(parents=True, exist_ok=True)
            lock = (directory / "controller.lock").open("a")
            locks.append(lock)
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise ValueError(f"{name} controller already uses {directory}") from error
            arm = arms[name]
            cwd = prepare_runtime(root, name, arm, template)
            env = dict(os.environ, Z1_CONTROLLER_PORT=str(arm["controller_port"]),
                       Z1_SDK_PORT=str(arm["sdk_port"]))
            print(f"{name}: MCU {arm['mcu_ip']}, local {arm['mcu_local_port']}, "
                  f"controller {arm['controller_port']}, SDK {arm['sdk_port']}", flush=True)
            children.append(subprocess.Popen([str(executable)], cwd=cwd, env=env,
                                             start_new_session=True))
        exited = set()
        while not stopping:
            for name, child in zip(names, children):
                if child.poll() is not None and name not in exited:
                    exited.add(name)
                    print(f"{name}: official controller exited with status {child.returncode}; "
                          "other arm is not stopped", file=sys.stderr, flush=True)
            if len(exited) == len(children):
                return 1
            time.sleep(0.1)
        return 1 if exited else 0
    finally:
        stop_children(children)
        for lock in locks:
            lock.close()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)


def main(argv=None):
    config, template = locations()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("controllers", "client", "check"))
    parser.add_argument("--arm", choices=("left", "right", "both"), default="both")
    parser.add_argument("--config", type=Path, default=config)
    parser.add_argument("--runtime-dir", type=Path,
                        default=Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "z1_sdk")
    parser.add_argument("--controller-bin")
    parser.add_argument("--client-bin")
    parser.add_argument("--template-dir", type=Path, default=template)
    # The command after -- is forwarded unchanged to the SDK client.
    args_list = list(sys.argv[1:] if argv is None else argv)
    command = []
    if "--" in args_list:
        split = args_list.index("--")
        command, args_list = args_list[split + 1:], args_list[:split]
    args = parser.parse_args(args_list)
    arms = load_config(args.config)
    if args.mode == "check":
        for name, arm in arms.items():
            print(name, arm)
        return 0
    if args.mode == "controllers":
        if command:
            parser.error("commands after -- apply only to client")
        return controllers(list(arms) if args.arm == "both" else [args.arm], arms,
                           args.runtime_dir, args.template_dir,
                           binary("z1_ctrl", args.controller_bin))
    if args.arm == "both":
        parser.error("client requires --arm left or --arm right (one owner per arm)")
    arm = arms[args.arm]
    client_directory = args.runtime_dir.resolve() / args.arm
    client_directory.mkdir(parents=True, exist_ok=True)
    client_lock = (client_directory / "client.lock").open("a")
    try:
        fcntl.flock(client_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        raise ValueError(f"{args.arm} SDK client is already running") from error
    os.set_inheritable(client_lock.fileno(), True)  # release when the exec'd client exits
    cmd = [str(binary("z1_sdk_client", args.client_bin)),
           "--controller-port", str(arm["controller_port"]),
           "--sdk-port", str(arm["sdk_port"]),
           "--has-gripper", str(arm["has_gripper"]).lower(),
           "--load-kg", str(arm["load_kg"])]
    if command:
        cmd += ["--"] + command
    os.execv(cmd[0], cmd)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, ET.ParseError, yaml.YAMLError, RuntimeError) as error:
        print(f"Z1 launcher: {error}", file=sys.stderr)
        sys.exit(1)
