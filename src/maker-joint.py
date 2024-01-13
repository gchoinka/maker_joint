import argparse
import math
import os
import re
import shutil
from typing import Tuple, List, Optional

from solid2 import cube, linear_extrude, polygon, cylinder, hull, scale, intersection, union, P3, scad_inline, surface, \
    openscad_functions, polyhedron, import_stl
from solid2.core.object_base import OpenSCADObject
from solid2_utils.utils import save_to_stl_scad, StlTask
from solid2.extensions.bosl2.threading import buttress_threaded_rod
from solid2.extensions.bosl2.screw_drive import torx_mask2d

unprintable_thickness = 0.01
preview_fix = 0.05
nut_slop = 0.4


# TODO nut turn helper
# TODO make stls for 6mm,8mm,15mm and 20mm

def middle_bolt(scaffold_rod_r: float, middle_rod_r: float) -> List[Tuple[Tuple[OpenSCADObject, P3], str]]:
    nut_l = 12
    bolt_l = nut_l * 2 + scaffold_rod_r * 2 * 4 + 3 * 3 + 4 * 2
    washer_h = 4
    washer_r = middle_rod_r * 1.8  #
    bolt = buttress_threaded_rod(d=middle_rod_r * 2, l=bolt_l, pitch=2, _fa=1, _fs=0.5, internal=False).up(bolt_l / 2)

    flat_face_box = cube([0.75 * middle_rod_r * 2, middle_rod_r * 2, bolt_l], center=True).up(bolt_l / 2)
    middle_rod = intersection()(bolt, flat_face_box)

    washer = cylinder(r=washer_r, h=washer_h, center=True).up(washer_h / 2)
    middle_rod_hole = intersection()(
        cylinder(r=middle_rod_r * 0.99, h=washer_h + preview_fix * 3, center=True).up(washer_h / 2),
        scale(1.08)(flat_face_box))
    washer -= middle_rod_hole.down(preview_fix)

    inside_thread = buttress_threaded_rod(d=middle_rod_r * 2, l=nut_l + preview_fix * 3, pitch=2, _fa=1, _fs=0.5,
                                          internal=True,
                                          _slop=nut_slop).rotate([180, 0, 0]).up(nut_l / 2 - preview_fix)
    top_chamfer_h = 3
    torx_mask_base = torx_mask2d(size=55, _fa=1, _fs=1)
    torx_nut = scale([2, 2, 1])(
        linear_extrude(height=nut_l - top_chamfer_h, center=False, scale=1)(torx_mask_base))
    torx_nut += scale([2, 2, 1])(
        linear_extrude(height=top_chamfer_h, center=False, scale=0.9)(torx_mask_base)).up(
        nut_l - top_chamfer_h + top_chamfer_h / 2 * 0)
    torx_nut += cylinder(r1=washer_r, r2=middle_rod_r, h=5).up(2)
    torx_nut += cylinder(r=washer_r, h=2)

    nut = torx_nut - inside_thread

    return [((middle_rod, (0, 0, 0)), "middle_rod"),
            ((nut, (40, 40, 0)), "nut01"),
            ((washer, (40, -40, 0)), "washer"),
            ]


def create_rotation_stop_washer(radius1: float, radius2: float, h: float = 3, num_triangles: int = 360):
    coordinates = []
    faces = []
    freq = 16
    amp_at0 = 0.03
    amp_at1 = 0.03
    offset = h
    r = radius1, radius2
    prev_idx_top_p0 = None
    prev_idx_top_p1 = None
    prev_idx_bottom_p0 = None
    prev_idx_bottom_p1 = None
    for i in range(num_triangles):
        theta = 2 * math.pi * i / num_triangles

        top_p0 = r[0] * math.cos(theta), r[0] * math.sin(theta), math.sin(theta * freq) * r[0] * amp_at0 + offset
        top_p1 = r[1] * math.cos(theta), r[1] * math.sin(theta), math.sin(theta * freq) * r[1] * amp_at1 + offset

        idx_top_p0 = len(coordinates) + 0
        idx_top_p1 = len(coordinates) + 1
        idx_bottom_p0 = len(coordinates) + 2
        idx_bottom_p1 = len(coordinates) + 3

        coordinates.extend((top_p0, top_p1, (*top_p0[0:2], 0), (*top_p1[0:2], 0)))
        if prev_idx_top_p0 is not None and prev_idx_top_p1 is not None:
            faces.append([idx_top_p1, prev_idx_top_p1, prev_idx_top_p0, idx_top_p0])
            faces.append([prev_idx_bottom_p0, prev_idx_bottom_p1, idx_bottom_p1, idx_bottom_p0])
            faces.append([prev_idx_bottom_p0, idx_bottom_p0, idx_top_p0, prev_idx_top_p0])
            faces.append([prev_idx_top_p1, idx_top_p1, idx_bottom_p1, prev_idx_bottom_p1])

        prev_idx_top_p0 = idx_top_p0
        prev_idx_top_p1 = idx_top_p1
        prev_idx_bottom_p0 = idx_bottom_p0
        prev_idx_bottom_p1 = idx_bottom_p1

    idx_top_p0 = 0
    idx_top_p1 = 1
    idx_bottom_p0 = 2
    idx_bottom_p1 = 3
    faces.append([idx_top_p1, prev_idx_top_p1, prev_idx_top_p0, idx_top_p0])
    faces.append([prev_idx_bottom_p0, prev_idx_bottom_p1, idx_bottom_p1, idx_bottom_p0])
    faces.append([prev_idx_bottom_p0, idx_bottom_p0, idx_top_p0, prev_idx_top_p0])
    faces.append([prev_idx_top_p1, idx_top_p1, idx_bottom_p1, prev_idx_bottom_p1])

    return coordinates, faces


def make_joint_half(scaffold_rod_r: float, middle_rod_r: float) -> List[Tuple[Tuple[OpenSCADObject, P3], str]]:
    rotate_part_h = 1.5
    compression_gap_h = scaffold_rod_r / 2
    top_washer_h = scaffold_rod_r + rotate_part_h - compression_gap_h / 2
    rotation_washer_h = top_washer_h
    top_washer_z = top_washer_h + compression_gap_h
    joint_r = (3.25 * scaffold_rod_r)
    # joint_r = (bold_d/2 + pipe_r*2+rotate_part_h+rotate_part_h)
    dummy_rod_l = joint_r * 2

    top_washer_reduction = 15 / 15
    rotation_washer = cylinder(r1=joint_r, r2=joint_r, h=rotation_washer_h, center=False, _fn=180)
    top_washer = cylinder(r1=joint_r, r2=joint_r * top_washer_reduction, h=top_washer_h, center=False, _fn=180).up(
        scaffold_rod_r + rotate_part_h + compression_gap_h / 2)
    compression_gap_washer = cylinder(r1=120 / 2, r2=120 / 2, h=compression_gap_h, center=True, _fn=180).up(
        rotate_part_h + scaffold_rod_r)

    dummy_rod_pos1 = [middle_rod_r + scaffold_rod_r + 0.5, 0, scaffold_rod_r + rotate_part_h]
    dummy_rod = cylinder(r=scaffold_rod_r + 0.2, h=dummy_rod_l + preview_fix * 2, center=True, _fn=60).rotate(
        [90, 0, 0]).translate(dummy_rod_pos1)
    pipe_1 = cylinder(r=scaffold_rod_r + rotate_part_h + 2, h=dummy_rod_l, center=True).rotate([90, 0, 0]).translate(
        dummy_rod_pos1)
    dummy_rod_pos2 = [-(middle_rod_r + scaffold_rod_r + 0.5), 0, scaffold_rod_r + rotate_part_h]
    dummy_rod += cylinder(r=scaffold_rod_r + 0.2, h=dummy_rod_l + preview_fix * 2, center=True, _fn=60).rotate(
        [90, 0, 0]).translate(dummy_rod_pos2)
    pipe_2 = cylinder(r=scaffold_rod_r + rotate_part_h + 2, h=dummy_rod_l, center=True).rotate([90, 0, 0]).translate(
        dummy_rod_pos2)

    rod_distance = math.fabs(dummy_rod_pos1[0]) + math.fabs(dummy_rod_pos2[0])

    middle_hole = cylinder(r=middle_rod_r, h=120, center=True, _fn=180)
    middle_hole_slopy = cylinder(r1=middle_rod_r + 0.8, r2=middle_rod_r + 0.1, h=top_washer_h + preview_fix * 2,
                                 center=False, _fn=180).up(top_washer_z - preview_fix)

    bottom_cutoff = cylinder(r=60, h=20).rotate([180, 0, 0])
    top_cutoff = cylinder(r=60, h=20).up(rotation_washer_h + compression_gap_h + rotation_washer_h)
    bottom_part = (rotation_washer - dummy_rod) - middle_hole
    top_bottom_threshold = bottom_cutoff + top_cutoff
    top_part = (top_washer + (
            pipe_1 + pipe_2) - compression_gap_washer - top_bottom_threshold - middle_hole) - dummy_rod - middle_hole_slopy

    start_r = middle_rod_r
    stop_r = joint_r

    rotation_stop_washer = polyhedron(
        *create_rotation_stop_washer(start_r, stop_r, h=rod_distance / 2 - scaffold_rod_r - rotate_part_h))
    joint_half = bottom_part + top_part + rotation_stop_washer.rotate([180, 0, 0])

    inside_thread = buttress_threaded_rod(d=middle_rod_r * 2, l=30 + preview_fix * 3, pitch=2, _fa=1, _fs=0.5,
                                          internal=True,
                                          _slop=nut_slop).rotate([180, 0, 0]).up(rotate_part_h / 2 - preview_fix)
    joint_half_end = bottom_part + top_part - inside_thread
    return [((joint_half, (-40, 80, 0)), "joint_half"),
            ((joint_half_end, (80, 80, 0)), "joint_half_end")
            ]


def parse_args():
    parser = argparse.ArgumentParser(
        prog='ProgramName',
        description='What the program does',
        epilog='Text at the bottom of help')
    parser.add_argument('--skip-stl', action='store_true')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--include_filter')
    return parser.parse_args()


def main(output_scad_basename, output_stl_basename, include_filter: Optional[str], verbose: bool):
    scaffold_rod_r = 8 / 2
    middle_rod_r = 10 / 2
    stl_tasks: List[StlTask] = [
        *middle_bolt(scaffold_rod_r, middle_rod_r),
        *make_joint_half(scaffold_rod_r, middle_rod_r)
    ]
    if include_filter is not None:
        stl_tasks = [t for t in stl_tasks if re.search(include_filter, t[1])]
    save_to_stl_scad(output_scad_basename, output_stl_basename, stl_tasks, verbose)


if __name__ == "__main__":
    args = parse_args()
    build_path: str = os.path.dirname(os.path.realpath(__file__))
    output_path: str = os.path.abspath(os.path.join(build_path, '..', 'build')) + os.path.sep

    if not os.path.exists(output_path):
        os.makedirs(output_path)

    stl_output_path: str | None = output_path
    if shutil.which("openscad") is None or args.skip_stl:
        stl_output_path = None
    main(output_scad_basename=output_path, output_stl_basename=stl_output_path, include_filter=args.include_filter,
         verbose=args.verbose)
