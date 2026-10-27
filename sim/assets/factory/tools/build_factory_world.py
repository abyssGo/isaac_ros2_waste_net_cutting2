"""Build the waste-net factory decoration layers and check them against the vision cameras.

Outputs (paths relative to ``sim/assets``)::

    factory/factory_outside_fov.usda       step 1: decoration the cameras can never see
    factory/factory_station_backdrop.usda  step 2: uniform screen behind the net (inside the camera view)
    factory/factory_enclosure.usda         step 2: walls/roof/props that the screen hides from the cameras
    project1/simulation_integration_v4_factory_step1.usda = step 1 + v4
    project1/simulation_integration_v4_factory.usda       = step 1 + step 2 + v4 (final)

The original ``simulation_integration_v4.usd`` is only sub-layered, never edited.
Every decoration prim is visual only (collision, rigid bodies and joints disabled).

After building, every decoration prim is sampled and tested against both camera
frusta (Camera1/2 at (-0.1, -/+2.0, 1.2) looking -X, 94.5 deg FOV) and against the
scene's DistantLight shadow direction. The result is written to
``factory/visibility_check.txt``; the script exits non-zero if anything other than
the backdrop panels can reach a camera image.

Run with the Isaac Sim kit python (needs ``pxr``)::

    EXT=$(ls -d $ISAAC_SIM_ROOT/extscache/omni.usd.libs-*)
    PYTHONPATH=$EXT LD_LIBRARY_PATH=$EXT/bin $ISAAC_SIM_ROOT/kit/python/bin/python3 -P build_factory_world.py [--backdrop-gray 0.35]
"""
import argparse
import math
import random
import sys
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

ASSETS = Path(__file__).resolve().parents[2]
FACTORY = ASSETS / "factory"
PROJECT = ASSETS / "project1"
PROPS = "./isaac/Environments/Simple_Warehouse/Props/"
NET_ASSET = "../project1/Collected_collected_net/Collected_net/SubUSDs/net_collected.usd"

# ---- scene facts (from simulation_integration_v4.usd) -------------------------
CAMERAS = {"camera1": Gf.Vec3d(-0.1, -2.0, 1.2), "camera2": Gf.Vec3d(-0.1, 2.0, 1.2)}
TAN_HALF_FOV = (3.896 / 2.0) / 1.8          # horizontal = vertical (640 x 640)
FOV_MARGIN = 1.05                           # test against a 5 % wider frustum
LIGHT_DIR = Gf.Vec3d(-1.0, 0.0, -1.0).GetNormalized()   # /Environment/defaultLight travels -X, 45 deg down
WALL_HEIGHT = 6.1                           # SM_WallA_6M (3.1) + SM_WallB_6M (3.0)

# ---- station backdrop (step 2) ---------------------------------------------------
# One continuous screen behind both stations. It sits far enough back (x = -2.6) that the
# shadows of the rail, carriages and objects (light at 45 deg, z <= 1.8 m) land on the floor
# in front of it exactly as in v4; at x = -1.6 the top-rail shadow crossed the image.
PANEL_X = -2.6                              # front face; the net hangs at x = -0.7
PANEL_Y = 5.0                               # screen covers |y| <= 5.0
PANEL_TOP = 4.3
PANEL_THICK = 0.05


# =============================================================================
# authoring helpers
# =============================================================================
def new_layer(path, comment):
    layer = Sdf.Layer.CreateNew(str(path)) if not path.exists() else Sdf.Layer.FindOrOpen(str(path))
    layer.Clear()
    layer.comment = comment
    stage = Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(world)
    return layer, stage


def xform(stage, path, t=(0, 0, 0), rz=0.0, s=None, rx=0.0, ry=0.0):
    prim = UsdGeom.Xform.Define(stage, path)
    prim.AddTranslateOp().Set(Gf.Vec3d(*t))
    if rz or rx or ry:
        prim.AddRotateXYZOp().Set(Gf.Vec3f(rx, ry, rz))
    if s is not None:
        prim.AddScaleOp().Set(Gf.Vec3f(*s) if isinstance(s, (tuple, list)) else Gf.Vec3f(s, s, s))
    return prim.GetPrim()


def ref(stage, path, asset, t=(0, 0, 0), rz=0.0, s=None, rx=0.0, ry=0.0, prim_path=None):
    prim = xform(stage, path, t, rz, s, rx, ry)
    if prim_path:
        prim.GetReferences().AddReference(asset, Sdf.Path(prim_path))
    else:
        prim.GetReferences().AddReference(asset)
    return prim


def preview_material(stage, path, color=(0.5, 0.5, 0.5), roughness=0.8, metallic=0.0, texture=None):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    if texture:
        reader = UsdShade.Shader.Define(stage, path + "/st")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex = UsdShade.Shader.Define(stage, path + "/Texture")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture)
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    else:
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def box(stage, path, center, size, material):
    prim = xform(stage, path, center, s=size)
    cube = UsdGeom.Cube.Define(stage, path + "/Box")
    cube.CreateSizeAttr(1.0)
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    return prim


def sign(stage, path, center, width, height, texture, facing="+X", post_height=None):
    """Textured board. ``facing`` = direction its printed face looks (+X or -X)."""
    rz = 0.0 if facing == "+X" else 180.0
    root = xform(stage, path, center, rz=rz)
    mesh = UsdGeom.Mesh.Define(stage, path + "/Face")
    w, h = width / 2.0, height / 2.0
    mesh.CreatePointsAttr([(0.012, -w, -h), (0.012, w, -h), (0.012, w, h), (0.012, -w, h)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([(1, 0, 0)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateExtentAttr([(0.012, -w, -h), (0.012, w, h)])
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray([(0, 0), (1, 0), (1, 1), (0, 1)]))
    name = Path(texture).stem
    material = preview_material(stage, f"/World/FactoryLooks/{name}", roughness=0.55, texture=texture)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    backing = preview_material(stage, "/World/FactoryLooks/SignBacking", (0.08, 0.08, 0.09), 0.5, 0.6)
    box(stage, path + "/Backing", (0.0, 0.0, 0.0), (0.02, width + 0.04, height + 0.04), backing)
    if post_height:
        box(stage, path + "/Post", (-0.03, 0.0, -(center[2] - h) / 2.0 - h), (0.05, 0.05, center[2] - h), backing)
        box(stage, path + "/Foot", (-0.03, 0.0, -center[2] + 0.015), (0.4, 0.4, 0.03), backing)
    return root


def finalize(stage, roots, no_shadow_roots=()):
    """Make every decoration prim visual only; drop shadows for overhead parts."""
    for root in roots:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            if prim.IsA(UsdPhysics.Joint):
                prim.SetActive(False)
    for root in no_shadow_roots:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if prim.IsA(UsdGeom.Gprim):
                UsdGeom.PrimvarsAPI(prim).CreatePrimvar(
                    "doNotCastShadows", Sdf.ValueTypeNames.Bool, UsdGeom.Tokens.constant).Set(True)


# =============================================================================
# step 1 -- outside every camera view (robot side x > 0, side walls, back wall)
# =============================================================================
def build_outside_fov():
    layer, stage = new_layer(FACTORY / "factory_outside_fov.usda",
                             "NetClean factory decoration, step 1: only places no vision camera can see.")
    R = "/World/Factory"
    xform(stage, R)

    # back wall behind the robots (inner face x = 12.3), two rows = 6.1 m
    for i, yc in enumerate((-9.0, -3.0, 3.0, 9.0)):
        ref(stage, f"{R}/BackWall/Low_{i}", PROPS + "SM_WallA_6M.usd", (12.3, yc, 0.0), rz=180.0, s=(1, 1, 1))
        ref(stage, f"{R}/BackWall/High_{i}", PROPS + "SM_WallB_6M.usd", (12.3, yc, 3.1), rz=180.0)
    # side walls (inner faces |y| = 12.3), x from 0.5 to 12.5 (further -X their shadows reach the far floor corner)
    for i, xc in enumerate((9.5, 3.5)):
        for side, (y, rz) in {"Left": (-12.5, 90.0), "Right": (12.5, -90.0)}.items():
            ref(stage, f"{R}/SideWalls/{side}Low_{i}", PROPS + "SM_WallA_6M.usd", (xc, y, 0.0), rz=rz)
            ref(stage, f"{R}/SideWalls/{side}High_{i}", PROPS + "SM_WallB_6M.usd",
                (xc, y + (0.2 if y < 0 else -0.2), 3.1), rz=rz)
    # pillars (9 m asset scaled to the 6.1 m wall)
    pz = WALL_HEIGHT / 9.0
    for i, y in enumerate((-12.0, -6.0, 0.0, 6.0, 12.0)):
        ref(stage, f"{R}/Pillars/Back_{i}", PROPS + "SM_PillarA_9M.usd", (12.2, y, 0.0), rz=180.0, s=(1, 1, pz))
    for i, x in enumerate((6.5, 0.5)):
        ref(stage, f"{R}/Pillars/Left_{i}", PROPS + "SM_PillarA_9M.usd", (x, -12.2, 0.0), rz=90.0, s=(1, 1, pz))
        ref(stage, f"{R}/Pillars/Right_{i}", PROPS + "SM_PillarA_9M.usd", (x, 12.2, 0.0), rz=-90.0, s=(1, 1, pz))

    # storage racks along the back wall (run along Y)
    for i, yc in enumerate((-8.5, -4.0, 4.0, 8.5)):
        rack = f"{R}/Racks/Rack_{i}"
        xform(stage, rack, (11.0, yc, 0.0), rz=90.0)
        for j, lx in enumerate((-2.0, 2.0)):
            ref(stage, f"{rack}/Frame_{j}", PROPS + "SM_RackFrame_03.usd", (lx, 0.0, 0.0))
        for j, z in enumerate((0.35, 1.5, 2.65)):
            ref(stage, f"{rack}/Shelf_{j}", PROPS + "SM_RackShelf_01.usd", (0.0, 0.0, z))
        ref(stage, f"{rack}/Load_0", PROPS + "SM_RackPile_06.usd", (0.0, 0.1, 0.38))
        ref(stage, f"{rack}/Load_1", PROPS + "SM_RackPile_06.usd", (0.0, 0.1, 1.53), rz=180.0)
        ref(stage, f"{rack}/Load_2", PROPS + "SM_RackPile_03.usd", (-1.0, 0.0, 2.68))
        ref(stage, f"{rack}/Load_3", PROPS + "SM_RackPile_03.usd", (1.0, 0.0, 2.68))

    # sorted output (PET / CAN / BUOY) pallets near the exit side
    crate = {"pet": "SM_CratePlastic_A_01", "can": "SM_CratePlastic_E_01", "buoy": "SM_CratePlastic_B_01"}
    for i, (name, y) in enumerate((("pet", 3.0), ("can", 5.0), ("buoy", 7.0))):
        root = f"{R}/SortedOutput/{name.capitalize()}"
        ref(stage, f"{root}/Pallet", PROPS + "SM_PaletteA_01.usd", (4.0, y, 0.0))
        for k, (dx, dy) in enumerate(((-0.31, -0.25), (0.31, -0.25), (-0.31, 0.25), (0.31, 0.25))):
            ref(stage, f"{root}/Crate_{k}", PROPS + crate[name] + ".usd", (4.0 + dx, y + dy, 0.21))
            if k < 2:
                ref(stage, f"{root}/CrateTop_{k}", PROPS + crate[name] + ".usd", (4.0 + dx, y + dy, 0.66))
        sign(stage, f"{root}/Sign", (3.25, y, 1.65), 0.9, 0.45, f"./textures/sign_bin_{name}.png", "+X", post_height=True)

    # barrels, cart, forklift, safety
    barrels = ["SM_BarelPlastic_A_01", "SM_BarelPlastic_B_01", "SM_BarelPlastic_C_01"]
    for i, (x, y) in enumerate(((6.0, -9.6), (6.0, -8.9), (6.0, -8.2), (6.7, -9.6), (6.7, -8.9), (6.7, -8.2))):
        ref(stage, f"{R}/Barrels/Barrel_{i}", PROPS + barrels[i % 3] + ".usd", (x, y, 0.0), rz=37.0 * i)
    ref(stage, f"{R}/Barrels/Bucket", PROPS + "SM_BucketPlastic_B.usd", (5.4, -9.8, 0.0))
    ref(stage, f"{R}/Cart/Pushcart", PROPS + "SM_PushcartA_02.usd", (2.4, -6.2, 0.0))
    ref(stage, f"{R}/Cart/Crate_0", PROPS + "SM_CratePlastic_B_01.usd", (2.4, -6.5, 0.38))
    ref(stage, f"{R}/Cart/Crate_1", PROPS + "SM_CratePlastic_B_01.usd", (2.4, -5.9, 0.38))
    ref(stage, f"{R}/Forklift", "./isaac/Props/Forklift/forklift.usd", (7.6, -1.2, 0.0), rz=90.0)
    ref(stage, f"{R}/WallGear/FuseBox", PROPS + "SM_FuseBox_01.usd", (12.3, 1.0, 0.0), rz=180.0)
    ref(stage, f"{R}/WallGear/EmergencyBoard", PROPS + "SM_EmergencyBoardFull_01.usd", (12.3, -0.6, 1.5), rz=180.0)
    ref(stage, f"{R}/WallGear/FireExtinguisher", PROPS + "SM_FireExtinguisher_02.usd", (12.3, 2.0, 1.3), rz=180.0)
    ref(stage, f"{R}/WallGear/Cables", PROPS + "SM_WallWire_01.usd", (12.3, -1.6, 0.0), rz=180.0)
    ref(stage, f"{R}/FloorMarks/KeepClear", PROPS + "SM_FloorDecal_Keepclear.usd", (11.3, 1.2, 0.003))
    for i, y in enumerate((-10.0, -6.0, -2.0, 2.0)):
        for j, x in enumerate((1.4, 2.9)):
            ref(stage, f"{R}/FloorMarks/Walkway_{j}_{i}", PROPS + "SM_FloorDecal_StripeFull_4m.usd", (x, y, 0.003), rz=90.0)
    for i, (x, y) in enumerate(((2.15, -10.3), (2.15, 6.3), (2.15, -2.6))):
        ref(stage, f"{R}/Safety/Cone_{i}", PROPS + "S_TrafficCone.usd", (x, y, 0.0))
    ref(stage, f"{R}/Safety/WetFloor", PROPS + "S_WetFloorSign.usd", (2.2, 0.4, 0.0), rz=90.0)
    sign(stage, f"{R}/Signs/RobotCell_0", (1.3, -4.6, 1.55), 1.0, 0.5, "./textures/sign_robot_cell.png", "+X", post_height=True)
    sign(stage, f"{R}/Signs/RobotCell_1", (1.3, 4.6, 1.55), 1.0, 0.5, "./textures/sign_robot_cell.png", "+X", post_height=True)
    sign(stage, f"{R}/Signs/LineBack", (12.28, 0.0, 4.6), 4.8, 1.2, "./textures/sign_line.png", "-X")

    # the 6.1 m back wall would put the robot-side props in its shadow; lighting there is cosmetic
    finalize(stage, [R], no_shadow_roots=[f"{R}/BackWall", f"{R}/Pillars"])
    layer.Save()
    return layer


# =============================================================================
# step 2a -- uniform screen behind the net (the only thing added to the camera view)
# =============================================================================
def build_backdrop(gray):
    layer, stage = new_layer(FACTORY / "factory_station_backdrop.usda",
                             "NetClean station backdrop, step 2: matte screen behind the net at both stations.")
    R = "/World/StationBackdrop"
    xform(stage, R)
    panel = preview_material(stage, f"{R}/Looks/Screen", (gray, gray, gray), roughness=0.95)
    frame = preview_material(stage, f"{R}/Looks/Aluminium", (0.62, 0.63, 0.65), roughness=0.35, metallic=0.9)
    cx = PANEL_X - PANEL_THICK / 2.0
    box(stage, f"{R}/Panel", (cx, 0.0, PANEL_TOP / 2.0), (PANEL_THICK, 2 * PANEL_Y, PANEL_TOP), panel)
    for name, y in (("Left", -PANEL_Y - 0.05), ("Right", PANEL_Y + 0.05)):
        box(stage, f"{R}/Frame/Post_{name}", (cx - 0.03, y, (PANEL_TOP + 0.05) / 2.0), (0.1, 0.1, PANEL_TOP + 0.05), frame)
        box(stage, f"{R}/Frame/Foot_{name}", (cx - 0.2, y, 0.02), (0.6, 0.14, 0.04), frame)
    box(stage, f"{R}/Frame/TopRail", (cx - 0.03, 0.0, PANEL_TOP + 0.05), (0.1, 2 * PANEL_Y + 0.2, 0.1), frame)
    finalize(stage, [R])
    layer.Save()
    return layer


# =============================================================================
# step 2b -- enclosure the backdrop hides from the cameras (far wall, roof, net storage)
# =============================================================================
def build_enclosure():
    layer, stage = new_layer(FACTORY / "factory_enclosure.usda",
                             "NetClean factory enclosure, step 2: hidden from the cameras by the station backdrop.")
    R = "/World/FactoryEnclosure"
    xform(stage, R)
    # the plain white wall of v4 is replaced by the factory wall
    stage.OverridePrim("/World/wall_plane").SetActive(False)

    for i, yc in enumerate((-9.0, -3.0, 3.0, 9.0)):
        ref(stage, f"{R}/FarWall/Low_{i}", PROPS + "SM_WallA_6M.usd", (-11.7, yc, 0.0))
        ref(stage, f"{R}/FarWall/High_{i}", PROPS + "SM_WallB_6M.usd", (-11.5, yc, 3.1))
    for i, xc in enumerate((-2.5, -8.5)):
        for side, (y, rz) in {"Left": (-12.5, 90.0), "Right": (12.5, -90.0)}.items():
            ref(stage, f"{R}/SideWalls/{side}Low_{i}", PROPS + "SM_WallA_6M.usd", (xc, y, 0.0), rz=rz)
            ref(stage, f"{R}/SideWalls/{side}High_{i}", PROPS + "SM_WallB_6M.usd", (xc, y + (0.2 if y < 0 else -0.2), 3.1), rz=rz)
    pz = WALL_HEIGHT / 9.0
    for i, x in enumerate((-5.5,)):
        ref(stage, f"{R}/Pillars/Left_{i}", PROPS + "SM_PillarA_9M.usd", (x, -12.2, 0.0), rz=90.0, s=(1, 1, pz))
        ref(stage, f"{R}/Pillars/Right_{i}", PROPS + "SM_PillarA_9M.usd", (x, 12.2, 0.0), rz=-90.0, s=(1, 1, pz))
    for i, y in enumerate((-12.0, -6.0, 0.0, 6.0, 12.0)):
        ref(stage, f"{R}/Pillars/Far_{i}", PROPS + "SM_PillarA_9M.usd", (-11.4, y, 0.0), s=(1, 1, pz))

    # roof + pendant lamps: no shadows, so the existing DistantLight lighting is unchanged
    for i, x in enumerate((-8.5, -2.5, 3.5, 9.5)):
        for j, y in enumerate((-12.0, -6.0, 0.0, 6.0, 12.0)):
            ref(stage, f"{R}/Roof/Tile_{i}_{j}", PROPS + "SM_CeilingA_6X6.usd", (x, y, WALL_HEIGHT), rx=180.0)
    for i, x in enumerate((-6.0, 0.6, 6.5)):
        for j, y in enumerate((-8.0, -4.0, 0.0, 4.0, 8.0)):
            # origin 1.6 m above the roof: the cable passes through it and the lamp body hangs at ~4.5 m
            ref(stage, f"{R}/Lamps/Lamp_{i}_{j}", PROPS + "SM_LampCeilingA_04.usd", (x, y, WALL_HEIGHT + 1.6))

    # racks along the far wall
    for i, yc in enumerate((-8.5, -4.0, 4.0, 8.5)):
        rack = f"{R}/Racks/Rack_{i}"
        xform(stage, rack, (-10.5, yc, 0.0), rz=90.0)
        for j, lx in enumerate((-2.0, 2.0)):
            ref(stage, f"{rack}/Frame_{j}", PROPS + "SM_RackFrame_03.usd", (lx, 0.0, 0.0))
        for j, z in enumerate((0.35, 1.5, 2.65)):
            ref(stage, f"{rack}/Shelf_{j}", PROPS + "SM_RackShelf_01.usd", (0.0, 0.0, z))
        ref(stage, f"{rack}/Load_0", PROPS + "SM_RackPile_06.usd", (0.0, -0.1, 0.38))
        ref(stage, f"{rack}/Load_1", PROPS + "SM_RackPile_06.usd", (0.0, -0.1, 1.53), rz=180.0)

    # collected-net storage: the team's net mesh, laid flat and stacked on pallets
    rng = random.Random(7)
    for p, (x, y) in enumerate(((-3.6, -9.6), (-3.6, -8.2), (-5.2, -9.6), (-5.2, -8.2), (-3.6, -6.8))):
        root = f"{R}/NetStorage/Pallet_{p}"
        ref(stage, f"{root}/Pallet", PROPS + "SM_PaletteA_01.usd", (x, y, 0.0))
        for k in range(3 + p % 3):
            net = ref(stage, f"{root}/Net_{k}", NET_ASSET, (x, y, 0.24 + 0.05 * k),
                      rz=rng.uniform(-25.0, 25.0), ry=90.0, s=(1, 0.95, 0.85))
            for child in ("carriage_TL", "carriage_TR", "carriage_BL", "carriage_BR"):
                stage.OverridePrim(net.GetPath().AppendPath(child)).SetActive(False)
    sign(stage, f"{R}/NetStorage/Sign", (-2.7, -8.2, 1.6), 1.0, 0.5, "./textures/sign_net_storage.png", "+X", post_height=True)
    sign(stage, f"{R}/Signs/LineFar", (-11.48, 0.0, 4.6), 4.8, 1.2, "./textures/sign_line.png", "+X")

    # presentation cameras (not used by any ROS graph)
    for name, eye, target in (("Overview", (10.5, -9.5, 4.0), (-1.5, 0.8, 1.2)),
                              ("RobotSide", (-1.2, -11.0, 4.4), (7.0, 1.5, 0.8)),
                              ("Line", (6.5, 0.0, 2.6), (-1.6, 0.0, 1.2))):
        cam = UsdGeom.Camera.Define(stage, f"/World/FactoryCams/{name}")
        cam.CreateFocalLengthAttr(18.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
        cam.AddTransformOp().Set(view.GetInverse())

    finalize(stage, [R], no_shadow_roots=[f"{R}/Roof", f"{R}/Lamps"])
    layer.Save()
    return layer


# =============================================================================
# world wrappers (stage metadata copied from v4: root-layer metadata is not inherited)
# =============================================================================
def world(path, sublayers, perspective):
    v4 = Sdf.Layer.FindOrOpen(str(PROJECT / "simulation_integration_v4.usd"))
    layer = Sdf.Layer.CreateNew(str(path)) if not path.exists() else Sdf.Layer.FindOrOpen(str(path))
    layer.Clear()
    layer.comment = "NetClean factory world: decoration layers over the unchanged simulation_integration_v4.usd"
    for key in ("upAxis", "metersPerUnit", "timeCodesPerSecond", "startTimeCode", "endTimeCode", "defaultPrim"):
        if v4.pseudoRoot.HasInfo(key):
            layer.pseudoRoot.SetInfo(key, v4.pseudoRoot.GetInfo(key))
    data = dict(v4.customLayerData)
    data.pop("omni_layer", None)
    if perspective:
        settings = dict(data.get("cameraSettings", {}))
        settings["Perspective"] = {"position": Gf.Vec3d(*perspective[0]), "target": Gf.Vec3d(*perspective[1])}
        data["cameraSettings"] = settings
    layer.customLayerData = data
    layer.subLayerPaths[:] = sublayers
    layer.Save()


# =============================================================================
# camera / shadow check
# =============================================================================
def in_view(point, camera, backdrop):
    d = camera[0] - point[0]
    if d <= 1e-6 or point[2] < -1e-6:
        return False
    k = TAN_HALF_FOV * FOV_MARGIN
    if abs(point[1] - camera[1]) > k * d or abs(point[2] - camera[2]) > k * d:
        return False
    if backdrop and point[0] < PANEL_X - 1e-6:
        t = (camera[0] - PANEL_X) / d
        yi = camera[1] + t * (point[1] - camera[1])
        zi = camera[2] + t * (point[2] - camera[2])
        if abs(yi) <= PANEL_Y and 0.0 <= zi <= PANEL_TOP:
            return False   # hidden behind the screen
    return True


def box_points(box_range, n=9):
    lo, hi = box_range.GetMin(), box_range.GetMax()
    for i in range(n):
        for j in range(n):
            for k in range(n):
                yield Gf.Vec3d(lo[0] + (hi[0] - lo[0]) * i / (n - 1),
                               lo[1] + (hi[1] - lo[1]) * j / (n - 1),
                               lo[2] + (hi[2] - lo[2]) * k / (n - 1))


def shadow_path(point, backdrop, steps=30):
    """Points along the light ray from ``point`` until it hits the floor or the screen."""
    end_s = point[2] / -LIGHT_DIR[2]
    if backdrop and point[0] > PANEL_X:
        s_panel = (point[0] - PANEL_X) / -LIGHT_DIR[0]
        hit = point + LIGHT_DIR * s_panel
        if s_panel < end_s and abs(hit[1]) <= PANEL_Y and 0.0 <= hit[2] <= PANEL_TOP:
            end_s = s_panel - 1e-3
    for i in range(1, steps + 1):
        yield point + LIGHT_DIR * (end_s * i / steps)


def casts_shadow(prim):
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Gprim):
            pv = UsdGeom.PrimvarsAPI(p).GetPrimvar("doNotCastShadows")
            if not (pv and pv.Get()):
                return True
    return False


def check(world_path, roots, backdrop, allowed):
    stage = Usd.Stage.Open(str(world_path))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    lines, failures = [], 0
    for root in roots:
        for prim in stage.GetPrimAtPath(root).GetChildren():
            leaves = [prim] if not prim.GetChildren() or prim.GetTypeName() == "Xform" and prim.HasAuthoredReferences() else prim.GetChildren()
            for leaf in leaves:
                rng = cache.ComputeWorldBound(leaf).ComputeAlignedRange()
                if rng.IsEmpty():
                    continue
                name = str(leaf.GetPath())
                seen = [c for c, pos in CAMERAS.items() if any(in_view(p, pos, backdrop) for p in box_points(rng))]
                shadow = []
                if casts_shadow(leaf):
                    shadow = [c for c, pos in CAMERAS.items()
                              if any(in_view(q, pos, backdrop) for p in box_points(rng, 5) for q in shadow_path(p, backdrop))]
                by_design = any(name == a or name.startswith(a + "/") for a in allowed)
                ok = by_design or (not seen and not shadow)
                failures += 0 if ok else 1
                tag = "ALLOWED (in view by design)" if by_design else ("OK" if ok else "FAIL")
                lines.append(f"{tag:28s} {name}  view={seen or '-'}  shadow_in_view={shadow or '-'}")
    return lines, failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backdrop-gray", type=float, default=0.35, help="linear albedo of the screen (0-1)")
    args = parser.parse_args()

    build_outside_fov()
    build_backdrop(args.backdrop_gray)
    build_enclosure()
    v4 = "./simulation_integration_v4.usd"
    keep_view = None  # step 1 keeps v4's viewport so before/after screenshots match
    world(PROJECT / "simulation_integration_v4_factory_step1.usda", ["../factory/factory_outside_fov.usda", v4], keep_view)
    world(PROJECT / "simulation_integration_v4_factory.usda",
          ["../factory/factory_station_backdrop.usda", "../factory/factory_enclosure.usda",
           "../factory/factory_outside_fov.usda", v4],
          ((10.5, -9.5, 4.0), (-1.5, 0.8, 1.2)))

    report = ["# factory decoration vs. vision cameras (generated by tools/build_factory_world.py)",
              f"# cameras {', '.join(f'{k}={tuple(v)}' for k, v in CAMERAS.items())}, FOV margin x{FOV_MARGIN},"
              f" light dir {tuple(round(c, 3) for c in LIGHT_DIR)}", ""]
    report.append("## step 1 world (no backdrop): every prim must be outside both views and cast no shadow into them")
    lines, fail1 = check(PROJECT / "simulation_integration_v4_factory_step1.usda", ["/World/Factory"], False, set())
    report += lines
    report.append("")
    report.append("## final world (backdrop): only the screen panel may be in view")
    allowed = {"/World/StationBackdrop/Panel"}
    lines, fail2 = check(PROJECT / "simulation_integration_v4_factory.usda",
                         ["/World/Factory", "/World/FactoryEnclosure", "/World/StationBackdrop"], True, allowed)
    report += lines
    report.append("")
    report.append(f"RESULT step1 failures={fail1}  final failures={fail2}")
    (FACTORY / "visibility_check.txt").write_text("\n".join(report) + "\n")
    print("\n".join(l for l in report if not l.startswith("OK")))
    return 1 if fail1 or fail2 else 0


if __name__ == "__main__":
    sys.exit(main())
