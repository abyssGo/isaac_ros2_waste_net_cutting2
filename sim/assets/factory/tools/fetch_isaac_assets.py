"""Download Isaac Sim 5.1 warehouse props (+ materials/textures) for offline use.

The props live on NVIDIA's public asset bucket.  This copies each listed USD and
every file it depends on (sub-layers, references, payloads, asset-valued
attributes, MDL imports and MDL textures) into ``../isaac/`` while keeping the
bucket's relative folder layout, so the relative paths inside the assets stay
valid and the factory world opens without internet.

Run with the Isaac Sim kit python (needs ``pxr``)::

    EXT=$(ls -d $ISAAC_SIM_ROOT/extscache/omni.usd.libs-*)
    PYTHONPATH=$EXT LD_LIBRARY_PATH=$EXT/bin $ISAAC_SIM_ROOT/kit/python/bin/python3 -P fetch_isaac_assets.py
"""
import posixpath
import re
import sys
import urllib.request
from pathlib import Path

from pxr import Sdf

BASE_URL = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/"
DEST = Path(__file__).resolve().parents[1] / "isaac"

PROPS = "Environments/Simple_Warehouse/Props/"
ASSETS = [PROPS + name + ".usd" for name in (
    "SM_WallA_6M", "SM_WallB_6M", "SM_PillarA_9M", "SM_CeilingA_6X6", "SM_LampCeilingA_04",
    "SM_RackFrame_03", "SM_RackShelf_01", "SM_RackPile_03", "SM_RackPile_06", "SM_Rackshield_02",
    "SM_PaletteA_01", "SM_CratePlastic_A_01", "SM_CratePlastic_B_01", "SM_CratePlastic_E_01",
    "SM_BarelPlastic_A_01", "SM_BarelPlastic_B_01", "SM_BarelPlastic_C_01", "SM_BucketPlastic_B",
    "SM_CardBoxA_01", "SM_CardBoxB_01", "SM_PushcartA_02", "SM_FuseBox_01", "SM_FireExtinguisher_02",
    "SM_EmergencyBoardFull_01", "SM_WallWire_01", "SM_FloorDecal_StripeFull_4m",
    "SM_FloorDecal_Keepclear", "S_TrafficCone", "S_WetFloorSign",
    "SM_BottlePlasticA_01", "SM_BottlePlasticB_01", "SM_BottlePlasticC_01",
)] + ["Props/Forklift/forklift.usd"]

MDL_STRING = re.compile(r'"([^"\n]+\.(?:png|jpg|jpeg|exr|hdr|dds|tga|mdl))"', re.IGNORECASE)
MDL_RELATIVE_MODULE = re.compile(r"^\s*(?:import|using)\s+\.::([A-Za-z0-9_]+)", re.MULTILINE)  # e.g. using .::OmniUe4Base import *;


def fetch(rel):
    """Download ``rel`` (bucket-relative) once; return local path or None if missing."""
    local = DEST / rel
    if local.exists():
        return local
    try:
        with urllib.request.urlopen(BASE_URL + rel, timeout=120) as response:
            data = response.read()
    except Exception as exc:  # 404 for optional/unused dependencies
        print(f"  missing {rel}: {exc}")
        return None
    local.parent.mkdir(parents=True, exist_ok=True)
    local.write_bytes(data)
    print(f"  {len(data) / 1e6:7.2f} MB  {rel}")
    return local


def usd_dependencies(local):
    layer = Sdf.Layer.FindOrOpen(str(local))
    deps = set(layer.GetCompositionAssetDependencies())  # sub-layers, references, payloads

    def collect(value):
        if isinstance(value, Sdf.AssetPath):
            if value.path:
                deps.add(value.path)
        elif hasattr(value, "__iter__") and not isinstance(value, str):
            for item in value:
                collect(item)

    def visit(path):
        if not path.IsPropertyPath():
            return
        spec = layer.GetAttributeAtPath(path)
        if spec is None or spec.typeName not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
            return
        if spec.HasDefaultValue():
            collect(spec.default)
        for t in layer.ListTimeSamplesForPath(path):
            collect(layer.QueryTimeSample(path, t))

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    return deps


def mdl_dependencies(local):
    text = local.read_text(errors="ignore")
    deps = set(MDL_STRING.findall(text))
    deps.update(name + ".mdl" for name in MDL_RELATIVE_MODULE.findall(text))
    return deps


def resolve(parent_rel, dep):
    dep = dep.replace("\\", "/")
    if "://" in dep or dep.startswith("/"):
        return None  # absolute/remote outside the bucket layout: ignored
    return posixpath.normpath(posixpath.join(posixpath.dirname(parent_rel), dep))


def main():
    queue = list(ASSETS)
    seen = set()
    while queue:
        rel = queue.pop()
        if rel in seen:
            continue
        seen.add(rel)
        local = fetch(rel)
        if local is None:
            continue
        suffix = local.suffix.lower()
        if suffix in (".usd", ".usda", ".usdc"):
            deps = usd_dependencies(local)
        elif suffix == ".mdl":
            deps = mdl_dependencies(local)
        else:
            continue
        for dep in deps:
            target = resolve(rel, dep)
            if target and target not in seen:
                queue.append(target)
    total = sum(p.stat().st_size for p in DEST.rglob("*") if p.is_file())
    print(f"done: {len(seen)} paths visited, {total / 1e6:.1f} MB in {DEST}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
