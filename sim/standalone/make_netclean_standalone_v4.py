#!/usr/bin/env python3
"""Create netclean_standalone_v4.py from the current main standalone.

Changes are intentionally limited to:
1. Default USD -> simulation_integration_v4.usd
2. Runtime suction FixedJoint frames -> common SuctionTCP world anchor

The source file is never modified.
"""

from __future__ import annotations

import argparse
import py_compile
import re
from pathlib import Path


STANDALONE_DIR = Path(__file__).resolve().parent
DEFAULT_SOURCE = STANDALONE_DIR / "netclean_standalone.py"
DEFAULT_OUTPUT = STANDALONE_DIR / "netclean_standalone_v4.py"
V4_USD_PATH = STANDALONE_DIR.parent / "assets" / "project1" / "simulation_integration_v4.usd"


OLD_SUCTION_BLOCK = """\
            body0_pos, body0_q = _get_stage_world_pose(self._stage, ROBOT2_SUCTION_BODY_PRIM_PATH)
            body1_pos, body1_q = _get_stage_world_pose(self._stage, object_body_path)
            rotation0 = _quaternion_to_rotation_matrix_wxyz(body0_q)
            local_pos0 = rotation0.T @ (body1_pos - body0_pos)
            local_rot0 = _normalize_quaternion_wxyz(_quaternion_multiply_wxyz(_quaternion_inverse_wxyz(body0_q), body1_q), "runtime suction local rotation")
            joint = UsdPhysics.FixedJoint.Define(self._stage, RUNTIME_SUCTION_JOINT_PATH)
            joint.CreateBody0Rel().SetTargets([Sdf.Path(ROBOT2_SUCTION_BODY_PRIM_PATH)])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(object_body_path)])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in local_pos0]))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(local_rot0[0]), Gf.Vec3f(float(local_rot0[1]), float(local_rot0[2]), float(local_rot0[3]))))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
            joint.CreateJointEnabledAttr().Set(True)
"""


NEW_SUCTION_BLOCK = """\
            # Runtime FixedJoint의 두 local frame을 실제 흡착 접촉점인
            # SuctionTCP의 동일한 World pose에서 계산한다. 물체 Prim 원점을
            # anchor로 사용하지 않으므로 Joint 생성 순간의 snap/회전을 방지한다.
            body0_pos, body0_q = _get_stage_world_pose(
                self._stage,
                ROBOT2_SUCTION_BODY_PRIM_PATH,
            )
            body1_pos, body1_q = _get_stage_world_pose(
                self._stage,
                object_body_path,
            )
            anchor_pos, anchor_q = _get_stage_world_pose(
                self._stage,
                ROBOT2_TCP_PRIM_PATH,
            )

            rotation0 = _quaternion_to_rotation_matrix_wxyz(body0_q)
            rotation1 = _quaternion_to_rotation_matrix_wxyz(body1_q)
            local_pos0 = rotation0.T @ (anchor_pos - body0_pos)
            local_pos1 = rotation1.T @ (anchor_pos - body1_pos)
            local_rot0 = _normalize_quaternion_wxyz(
                _quaternion_multiply_wxyz(
                    _quaternion_inverse_wxyz(body0_q),
                    anchor_q,
                ),
                "runtime suction local rotation0",
            )
            local_rot1 = _normalize_quaternion_wxyz(
                _quaternion_multiply_wxyz(
                    _quaternion_inverse_wxyz(body1_q),
                    anchor_q,
                ),
                "runtime suction local rotation1",
            )

            joint = UsdPhysics.FixedJoint.Define(
                self._stage,
                RUNTIME_SUCTION_JOINT_PATH,
            )
            joint.CreateBody0Rel().SetTargets([
                Sdf.Path(ROBOT2_SUCTION_BODY_PRIM_PATH)
            ])
            joint.CreateBody1Rel().SetTargets([
                Sdf.Path(object_body_path)
            ])
            joint.CreateLocalPos0Attr().Set(
                Gf.Vec3f(*[float(v) for v in local_pos0])
            )
            joint.CreateLocalRot0Attr().Set(
                Gf.Quatf(
                    float(local_rot0[0]),
                    Gf.Vec3f(
                        float(local_rot0[1]),
                        float(local_rot0[2]),
                        float(local_rot0[3]),
                    ),
                )
            )
            joint.CreateLocalPos1Attr().Set(
                Gf.Vec3f(*[float(v) for v in local_pos1])
            )
            joint.CreateLocalRot1Attr().Set(
                Gf.Quatf(
                    float(local_rot1[0]),
                    Gf.Vec3f(
                        float(local_rot1[1]),
                        float(local_rot1[2]),
                        float(local_rot1[3]),
                    ),
                )
            )
            joint.CreateJointEnabledAttr().Set(True)
"""


def replace_usd_path(text: str) -> str:
    pattern = re.compile(r'^USD_PATH\s*=.*$', re.MULTILINE)
    replacement = 'USD_PATH = str(ASSET_ROOT / "simulation_integration_v4.usd")'
    updated, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise RuntimeError("USD_PATH 선언을 정확히 하나 찾지 못했습니다.")
    return updated


def replace_suction_joint(text: str) -> str:
    if "runtime suction local rotation1" in text and "ROBOT2_TCP_PRIM_PATH" in text:
        print("SuctionTCP 공통 anchor 코드는 이미 들어 있습니다.")
        return text

    if OLD_SUCTION_BLOCK not in text:
        raise RuntimeError(
            "RuntimeSuctionController.attach_class()의 기존 Joint 블록을 찾지 못했습니다. "
            "현재 v2 파일의 attach_class()가 대화에서 확인한 버전과 다릅니다."
        )
    return text.replace(OLD_SUCTION_BLOCK, NEW_SUCTION_BLOCK, 1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"원본 v2 파일이 없습니다: {source}")
    if source == output:
        raise ValueError("원본과 출력 경로가 같습니다.")

    text = source.read_text(encoding="utf-8")
    text = replace_usd_path(text)
    text = replace_suction_joint(text)

    output.write_text(text, encoding="utf-8")
    py_compile.compile(str(output), doraise=True)

    print(f"생성 완료: {output}")
    print(f"기본 USD: {V4_USD_PATH}")
    print("문법 검사: 성공")
    print("원본 standalone 파일은 변경하지 않았습니다.")


if __name__ == "__main__":
    main()
