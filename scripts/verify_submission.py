#!/usr/bin/env python3
"""Check that the repository is safe to submit and clone on another PC."""

from __future__ import annotations

from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
GITHUB_HARD_FILE_LIMIT = 100 * 1024 * 1024
GITHUB_WARNING_FILE_SIZE = 50 * 1024 * 1024

REQUIRED_FILES = (
    "README.md",
    "sim/assets/project1/simulation_integration_v3.usd",
    "sim/assets/project1/robot2_sample/m0609_isaac_sim.urdf",
    "sim/assets/project1/robot2_sample/m0609_description.yaml",
    "sim/assets/project1/Collected_collected_net/collected_net.usd",
    "sim/assets/project1/Collected_collected_net/Collected_net/World4.usd",
    "sim/assets/project1/Collected_collected_net/Collected_net/SubUSDs/net_collected.usd",
    "sim/standalone/netclean_standalone.py",
    "scripts/run_standalone.sh",
    "ros2_ws/src/nc_bringup/launch/pc_a.launch.py",
    "ros2_ws/src/nc_bringup/launch/pc_b.launch.py",
    "models/netclean_yolo11n/weights/best.pt",
)

REQUIRED_PACKAGES = (
    "nc_interfaces",
    "nc_control",
    "nc_robot",
    "nc_vision",
    "nc_vision_debug",
    "nc_bringup",
)

FORBIDDEN_DIRECTORIES = {"build", "install", "log"}
FORBIDDEN_ARCHIVE_SUFFIXES = (".zip", ".7z", ".rar", ".tar", ".tar.gz", ".tgz")
TEXT_SUFFIXES = {".py", ".sh", ".yaml", ".yml", ".xml", ".urdf", ".json"}
HOME_PATH_PATTERN = re.compile(r"(?:^|[^A-Za-z0-9_])/(?:home|Users)/")
WINDOWS_HOME_PATTERN = re.compile(r"[A-Za-z]:[\\/](?:Users|Documents and Settings)[\\/]", re.IGNORECASE)


def git_output(*args: str) -> bytes:
    return subprocess.run(
        ["git", "-C", str(ROOT), *args],
        check=True,
        stdout=subprocess.PIPE,
    ).stdout


def git_tracked_paths() -> list[PurePosixPath]:
    values = git_output("ls-files", "-z").split(b"\0")
    return [PurePosixPath(value.decode("utf-8", "surrogateescape")) for value in values if value]


def submission_paths() -> tuple[list[PurePosixPath], bool]:
    """Return Git-tracked paths, or all files when run from an exported ZIP."""
    in_git_repository = subprocess.run(
        ["git", "-C", str(ROOT), "rev-parse", "--is-inside-work-tree"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0
    if in_git_repository:
        return git_tracked_paths(), True

    paths = [
        PurePosixPath(path.relative_to(ROOT).as_posix())
        for path in ROOT.rglob("*")
        if path.is_file() and ".git" not in path.relative_to(ROOT).parts
    ]
    return sorted(paths, key=lambda item: item.as_posix()), False


def main() -> int:
    errors: list[str] = []
    warnings: list[str] = []

    try:
        tracked, in_git_repository = submission_paths()
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: Git tracked-file 목록을 읽지 못했습니다: {exc}", file=sys.stderr)
        return 2

    tracked_set = {path.as_posix() for path in tracked}
    tracked_bytes = 0

    for required in REQUIRED_FILES:
        if required not in tracked_set:
            errors.append(f"필수 파일이 Git에 포함되지 않음: {required}")
        elif not (ROOT / required).is_file():
            errors.append(f"필수 파일이 작업 트리에 없음: {required}")

    for package in REQUIRED_PACKAGES:
        package_xml = f"ros2_ws/src/{package}/package.xml"
        if package_xml not in tracked_set:
            errors.append(f"ROS 2 패키지가 Git에 포함되지 않음: {package}")

    for path in tracked:
        path_text = path.as_posix()
        if FORBIDDEN_DIRECTORIES.intersection(path.parts):
            errors.append(f"생성 폴더가 Git에 포함됨: {path_text}")
        if path_text.lower().endswith(FORBIDDEN_ARCHIVE_SUFFIXES):
            errors.append(f"압축/백업 파일이 Git에 포함됨: {path_text}")

        disk_path = ROOT / path_text
        if not disk_path.is_file():
            continue
        size = disk_path.stat().st_size
        tracked_bytes += size
        if size > GITHUB_HARD_FILE_LIMIT:
            errors.append(f"GitHub 100MiB 제한 초과: {path_text} ({size / 1048576:.1f}MiB)")
        elif size > GITHUB_WARNING_FILE_SIZE:
            warnings.append(f"GitHub 대용량 경고 대상: {path_text} ({size / 1048576:.1f}MiB)")

        if disk_path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        try:
            text = disk_path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        if HOME_PATH_PATTERN.search(text) or WINDOWS_HOME_PATTERN.search(text):
            errors.append(f"개인 PC 절대경로가 남아 있음: {path_text}")

    urdf_path = ROOT / "sim/assets/project1/robot2_sample/m0609_isaac_sim.urdf"
    if urdf_path.is_file():
        try:
            root = ET.parse(urdf_path).getroot()
        except (ET.ParseError, OSError) as exc:
            errors.append(f"URDF XML 파싱 실패: {exc}")
        else:
            meshes = root.findall(".//mesh")
            if not meshes:
                errors.append("URDF에 mesh 항목이 없습니다")
            for mesh in meshes:
                filename = mesh.get("filename", "").strip()
                if not filename:
                    errors.append("URDF mesh filename이 비어 있습니다")
                    continue
                if filename.startswith(("file://", "package://")) or Path(filename).is_absolute():
                    errors.append(f"URDF mesh가 비이식 경로를 사용함: {filename}")
                    continue
                resolved = (urdf_path.parent / filename).resolve()
                try:
                    relative = resolved.relative_to(ROOT).as_posix()
                except ValueError:
                    errors.append(f"URDF mesh가 저장소 밖을 가리킴: {filename}")
                    continue
                if not resolved.is_file():
                    errors.append(f"URDF mesh 파일 누락: {relative}")
                elif relative not in tracked_set:
                    errors.append(f"URDF mesh가 Git에 포함되지 않음: {relative}")

    if in_git_repository:
        dirty = subprocess.run(
            ["git", "-C", str(ROOT), "status", "--porcelain"],
            check=True,
            stdout=subprocess.PIPE,
            text=True,
        ).stdout.strip()
        if dirty:
            warnings.append("커밋되지 않은 변경사항이 있습니다. 제출 ZIP은 커밋 후 생성하세요.")

    source = "Git 추적 파일" if in_git_repository else "압축 해제 파일"
    print(f"{source}: {len(tracked)}개, 검사 크기: {tracked_bytes / 1048576:.1f}MiB")
    for warning in warnings:
        print(f"WARNING: {warning}")
    for error in errors:
        print(f"ERROR: {error}", file=sys.stderr)

    if errors:
        print(f"제출 점검 실패: 오류 {len(errors)}개", file=sys.stderr)
        return 1
    print("제출 점검 통과")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
