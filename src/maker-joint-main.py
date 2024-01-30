import argparse
from dataclasses import dataclass
import math
import operator
import os
import shutil
from pathlib import Path
from typing import Tuple, List, Iterable, Generator

from solid2 import cube, linear_extrude, cylinder, scale, intersection, P3, polyhedron
from solid2.core.object_base import OpenSCADObject
from solid2_utils.utils import save_to_file, RenderTask, modify_render_task
from solid2.extensions.bosl2.threading import buttress_threaded_rod
from solid2.extensions.bosl2.screw_drive import torx_mask2d

unprintable_thickness = 0.01
preview_fix = 0.05
nut_slop = 0.4
thread_pitch = 2


def todo_remove_constant(v: any):
    return v


def middle_bolt(scaffold_rod_r: float, middle_rod_r: float) -> List[RenderTask]:
    nut_l = 12
    bolt_l = nut_l * 2 + scaffold_rod_r * 2 * 4 + 3 * 3 + 4 * 2
    washer_h = 4
    washer_r = middle_rod_r * 1.8  #
    bolt = buttress_threaded_rod(d=middle_rod_r * 2, l=bolt_l, pitch=thread_pitch, _fa=1, _fs=0.5, internal=False).up(
        bolt_l / 2)

    flat_face_box = cube([0.69 * middle_rod_r * 2, middle_rod_r * 2, bolt_l], center=True).up(bolt_l / 2)
    middle_rod = intersection()(bolt, flat_face_box)

    washer = cylinder(r=washer_r, h=washer_h, center=True).up(washer_h / 2)
    middle_rod_hole = intersection()(
        cylinder(r=middle_rod_r * 0.99, h=washer_h + preview_fix * 3, center=True).up(washer_h / 2),
        scale(1.09)(flat_face_box))
    washer -= middle_rod_hole.down(preview_fix)

    inside_thread = buttress_threaded_rod(d=middle_rod_r * 2, l=nut_l + preview_fix * 3, pitch=thread_pitch, _fa=1,
                                          _fs=0.5,
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

    return [RenderTask(middle_rod, (0, 0, 0), Path("middle_rod")),
            RenderTask(nut, (20, 20, 0), Path("nut")),
            RenderTask(washer, (40, -40, 0), Path("washer")),
            ]


def create_rotation_stop_washer(r1_hole: float, r2_outside: float, h: float = 3, num_triangles: int = 360):
    coordinates = []
    faces = []
    freq = 16
    amp_at0 = 0.005
    amp_at1 = 0.02
    offset = h
    r = r1_hole, r2_outside
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


def make_joint_half(scaffold_rod_r: float, middle_rod_r: float) -> List[RenderTask]:
    rotate_part_h = 1.5
    compression_gap_h = scaffold_rod_r / 2
    top_washer_h = scaffold_rod_r + rotate_part_h - compression_gap_h / 2
    body_h = top_washer_h
    top_washer_z = top_washer_h + compression_gap_h
    joint_r = (3.25 * scaffold_rod_r)
    dummy_rod_l = joint_r * 2

    top_washer_reduction = 15 / 15
    bottom_main = cube([joint_r * 2, joint_r * 2, body_h], center=True).up(body_h / 2)
    top_washer = cylinder(r1=joint_r, r2=joint_r * top_washer_reduction, h=top_washer_h, center=False, _fn=180).up(
        scaffold_rod_r + rotate_part_h + compression_gap_h / 2)
    compression_gap_washer = cylinder(r1=120 / 2, r2=120 / 2, h=compression_gap_h, center=True, _fn=180).up(
        rotate_part_h + scaffold_rod_r)  # TODO: r1 and r2 and this is not a washer

    dummy_rod_pos1 = [middle_rod_r + scaffold_rod_r + 0.5, 0, scaffold_rod_r + rotate_part_h]
    dummy_rod = cylinder(r=scaffold_rod_r + 0.2, h=dummy_rod_l + preview_fix * 2, center=True, _fn=60).rotate(
        [90, 0, 0]).translate(dummy_rod_pos1)
    pipe_1 = cylinder(r=scaffold_rod_r + rotate_part_h + 2, h=dummy_rod_l, center=True).rotate([90, 0, 0]).translate(
        dummy_rod_pos1)
    dummy_rod_pos2 = [-(middle_rod_r + scaffold_rod_r + 0.5), 0, scaffold_rod_r + rotate_part_h]
    dummy_rod += cylinder(r=scaffold_rod_r + 0.2, h=dummy_rod_l + preview_fix * 2, center=True, _fn=60).rotate(
        [90, 0, 0]).translate(dummy_rod_pos2)
    pipe_2 = scale([1, 1, 1])(
        cylinder(r=scaffold_rod_r + rotate_part_h + 2, h=dummy_rod_l, center=True).rotate([90, 0, 0])).translate(
        dummy_rod_pos2)

    rod_distance = math.fabs(dummy_rod_pos1[0]) + math.fabs(dummy_rod_pos2[0])

    middle_hole = cylinder(r=middle_rod_r, h=top_washer_z * 2 + body_h, center=True, _fn=180).up(
        top_washer_z - body_h / 2)  # + cube([middle_rod_r,middle_rod_r,top_washer_z * 2 + rotation_washer_h +todo_remove_constant(4)],center=True).up(top_washer_z - rotation_washer_h / 2).rotate([0,0,45]).translate([0, middle_rod_r-middle_rod_r/2.5,0])

    bottom_part = (bottom_main + (pipe_1 + pipe_2) - compression_gap_washer) - dummy_rod
    # top_part = (top_washer + (pipe_1 + pipe_2) - compression_gap_washer) - dummy_rod

    start_r = middle_rod_r
    stop_r = joint_r

    rotation_stop_washer_h = rod_distance / 2 - scaffold_rod_r - rotate_part_h
    rotation_stop_washer = polyhedron(
        *create_rotation_stop_washer(start_r, stop_r + 20, h=rotation_stop_washer_h))

    rotation_stop_washer = intersection()(rotation_stop_washer, cube(
        [stop_r * 2, stop_r * 2, rotation_stop_washer_h + todo_remove_constant(5)],
        center=True))
    joint_half = bottom_part + rotation_stop_washer.rotate([180, 0, 0]) - middle_hole

    inside_thread_h = body_h + rotation_stop_washer_h
    inside_thread = buttress_threaded_rod(d=middle_rod_r * 2, l=inside_thread_h, pitch=thread_pitch, _fa=1, _fs=0.5,
                                          internal=True,
                                          _slop=nut_slop - 0.2).rotate([180, 0, 0]).up(
        inside_thread_h / 2. - rotation_stop_washer_h)

    middle_hole = cylinder(r1=middle_rod_r, r2=middle_rod_r * 2, h=3, center=True, _fn=180).up(
        5)  # + cube([middle_rod_r,middle_rod_r,top_washer_z * 2 + rotation_washer_h +todo_remove_constant(4)],center=True).up(top_washer_z - rotation_washer_h / 2).rotate([0,0,45]).translate([0, -middle_rod_r+middle_rod_r/1.5,0])

    rotation_stop_washer = polyhedron(
        *create_rotation_stop_washer(middle_rod_r - 2, stop_r + 20,
                                     h=rod_distance / 2 - scaffold_rod_r - rotate_part_h))

    rotation_stop_washer = intersection()(rotation_stop_washer, cube(
        [stop_r * 2, stop_r * 2, rod_distance / 2 - scaffold_rod_r - rotate_part_h + todo_remove_constant(10)],
        center=True))
    joint_half_end = bottom_part + rotation_stop_washer.rotate(
        [180, 0, 0]) - inside_thread - middle_hole.rotate([180, 0, 0])
    return [RenderTask(joint_half, (-40, 80, 0), Path("joint_half")),
            RenderTask(joint_half_end, (-80, 80, 0), Path("joint_half_end")),
            ]


@dataclass
class MakerJointSet:
    scaffold_rod_r: float
    middle_rod_r: float


def maker_joint_sets() -> List[RenderTask]:
    render_task: List[RenderTask] = list()
    sets: List[MakerJointSet] = [MakerJointSet(sr, mr) for sr, mr in (
        (3 / 2, 6 / 2), (6 / 2, 10 / 2), (8 / 2, 10 / 2), (8 / 2, 10 / 2), (8 / 2, 10 / 2), (8 / 2, 10 / 2))]
    offset_pos = (0., 0., 0.)
    offset_diff = (100., 100., 100.)
    for s in sets:
        sub_render_task: List[RenderTask] = [
            *middle_bolt(s.scaffold_rod_r, s.middle_rod_r),
            *make_joint_half(s.scaffold_rod_r, s.middle_rod_r)
        ]
        render_task += list(
            modify_render_task(sub_render_task, offset_pos, name_suffix=f"_{s.scaffold_rod_r * 2:.0f}mm"))
        offset_pos = tuple(map(operator.add, offset_pos, offset_diff))
    return render_task


def parse_args():
    parser = argparse.ArgumentParser(
        prog='MakerJointMakeScript',
        description='Creates the 3MFs and SCAD files for building the maker_joint')
    parser.add_argument('--skip_rendering', action='store_true')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--openscad_bin')
    parser.add_argument('--include_filter_regex')
    return parser.parse_args()


def main():
    args = parse_args()
    build_path: str = os.path.dirname(os.path.realpath(__file__))
    output_path: str = os.path.abspath(os.path.join(build_path, '..', 'build')) + os.path.sep

    if not os.path.exists(output_path):
        os.makedirs(output_path)

    openscad_bin: Path | None = Path(args.openscad_bin) if args.openscad_bin is not None else shutil.which("openscad")
    if openscad_bin is None:
        print("Didn't found openscad in PATH environment variable, skipping rendering 3mf/stl/png!")

    if args.skip_rendering:
        openscad_bin = None

    render_tasks = maker_joint_sets()
    save_to_file(Path(output_path), openscad_bin, render_tasks, Path("all"),
                 include_filter_regex=args.include_filter_regex,
                 verbose=args.verbose)


if __name__ == "__main__":
    main()
