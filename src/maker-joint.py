import argparse
import itertools
import math
import os
import shutil
import sys
from itertools import pairwise
from typing import Tuple, List, Optional

from solid2 import cube, linear_extrude, polygon, cylinder, hull, scale, intersection, union, P3, scad_inline
from solid2.core.object_base import OpenSCADObject
from solid2_utils.utils import save_to_stl_scad, StlTask
from solid2.extensions.bosl2.threading import buttress_threaded_rod
from solid2.extensions.bosl2.screw_drive import torx_mask2d, torx_mask

unprintable_thickness = 0.01
preview_fix = 0.05
pipe_r = 8 / 2
bold_d = 10
pitch = 2


# TODO rotation stop washer needs to handle small rods and big rods
# TODO nut turn helper
# TODO double rotation stop washer
# TODO handling of 4 rods
# TODO make stls for 6mm,8mm,15mm and 20mm
# TODO make washer_r fixed


def middle_bolt() -> List[Tuple[Tuple[OpenSCADObject, P3], str]]:
    nut_l = 15
    bolt_l = nut_l * 2 + pipe_r * 4 + 3 * 3 + 4 * 2
    washer_h = 4
    washer_r = bold_d / 2 * 1.8 #
    bolt = buttress_threaded_rod(d=bold_d, l=bolt_l, pitch=2, _fa=1, _fs=0.5, internal=False).up(bolt_l / 2)
    # bolt -= cylinder(r=3.5 / 2, h=bolt_l + 2 * preview_fix, center=True, _fn=30).up(bolt_l / 2)

    flat_face_box = cube([0.75 * bold_d, bold_d, bolt_l], center=True).up(bolt_l / 2)
    floated_bolt = intersection()(bolt, flat_face_box)

    floated_bolt_washer = cylinder(r=washer_r, h=washer_h, center=True).up(washer_h / 2)
    floated_bolt_washer_inside = intersection()(
        cylinder(r=bold_d / 2 * 0.99, h=washer_h + preview_fix * 3, center=True).up(washer_h / 2),
        scale(1.08)(flat_face_box))
    floated_bolt_washer = floated_bolt_washer - floated_bolt_washer_inside.down(preview_fix)

    inside_thread = buttress_threaded_rod(d=bold_d, l=nut_l + preview_fix * 3, pitch=2, _fa=1, _fs=0.5, internal=True,
                                          _slop=0.4).rotate([180, 0, 0]).up(nut_l / 2 - preview_fix)
    top_chamfer_h = 3
    torx_mask_base = torx_mask2d(size=55, _fa=1, _fs=1)
    torx_nut = scale([2, 2, 1])(
        linear_extrude(height=nut_l - top_chamfer_h, center=False, scale=1)(torx_mask_base))
    torx_nut += scale([2, 2, 1])(
        linear_extrude(height=top_chamfer_h, center=False, scale=0.9)(torx_mask_base)).up(
        nut_l - top_chamfer_h + top_chamfer_h / 2 * 0)
    # torx_nut += scale([2, 2, 1])(
    #     linear_extrude(height=top_chamfer_h / 2, center=False, scale=1)(torx_mask_base)).up(
    #     nut_l - top_chamfer_h + top_chamfer_h / 2 * 1)
    torx_nut += cylinder(r1=washer_r, r2=bold_d / 2, h=5).up(2)
    torx_nut += cylinder(r=washer_r, h=2)

    nut = torx_nut - inside_thread

    return [((floated_bolt, (0, 0, 0)), "middle_bolt"),
            ((nut, (40, 40, 0)), "nut01"),
            ((floated_bolt_washer, (40, 80, 0)), "floated_bolt_washer"),
            ]


# TODO replace return type with dataclass
def create_circle_coordinates(radius1: float, radius2: float, num_triangles: int) -> List[
    Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], float]]:
    coordinates = []
    for i in range(num_triangles):
        theta1 = 2 * math.pi * i / num_triangles
        theta2 = 2 * math.pi * (i + 1) / num_triangles

        x1 = radius2 * math.cos(theta1)
        y1 = radius2 * math.sin(theta1)

        x2 = radius2 * math.cos(theta2)
        y2 = radius2 * math.sin(theta2)

        x3 = radius1 * math.cos(theta2)
        y3 = radius1 * math.sin(theta2)

        x4 = radius1 * math.cos(theta1)
        y4 = radius1 * math.sin(theta1)

        coordinates.append(((x1, y1), (x2, y2), (x3, y3), (x4, y4), (theta1 + theta2) / 2))  # Center of the circle
    return coordinates


def seq(start, end, step):
    assert (step != 0)
    sample_count = int(abs(end - start) / step)
    return itertools.islice(itertools.count(start, step), sample_count)


def make_joint_half() -> List[Tuple[Tuple[OpenSCADObject, P3], str]]:
    rotate_part_h = 1.5
    compression_gap_h = pipe_r / 2
    top_washer_h = pipe_r + rotate_part_h - compression_gap_h / 2
    rotation_washer_h = top_washer_h
    top_washer_z = top_washer_h + compression_gap_h
    joint_r = (3.25 * pipe_r)
    # joint_r = (bold_d/2 + pipe_r*2+rotate_part_h+rotate_part_h)
    dummy_rod_l = joint_r * 2

    top_washer_reduction = 15 / 15
    rotation_washer = cylinder(r1=joint_r, r2=joint_r, h=rotation_washer_h, center=False, _fn=180)
    top_washer = cylinder(r1=joint_r, r2=joint_r * top_washer_reduction, h=top_washer_h, center=False, _fn=180).up(
        pipe_r + rotate_part_h + compression_gap_h / 2)
    compression_gap_washer = cylinder(r1=120 / 2, r2=120 / 2, h=compression_gap_h, center=True, _fn=180).up(
        rotate_part_h + pipe_r)

    dummy_rod_pos1 = [bold_d / 2 + pipe_r + 0.5, 0, pipe_r + rotate_part_h]
    dummy_rod = cylinder(r=pipe_r + 0.2, h=dummy_rod_l + preview_fix * 2, center=True, _fn=60).rotate(
        [90, 0, 0]).translate(dummy_rod_pos1)
    pipe_1 = cylinder(r=pipe_r + rotate_part_h + 2, h=dummy_rod_l, center=True).rotate([90, 0, 0]).translate(
        dummy_rod_pos1)
    dummy_rod_pos2 = [-(bold_d / 2 + pipe_r + 0.5), 0, pipe_r + rotate_part_h]
    dummy_rod += cylinder(r=pipe_r + 0.2, h=dummy_rod_l + preview_fix * 2, center=True, _fn=60).rotate(
        [90, 0, 0]).translate(dummy_rod_pos2)
    pipe_2 = cylinder(r=pipe_r + rotate_part_h + 2, h=dummy_rod_l, center=True).rotate([90, 0, 0]).translate(
        dummy_rod_pos2)

    middle_hole = cylinder(r=bold_d / 2, h=120, center=True, _fn=180)
    middle_hole_slopy = cylinder(r1=bold_d / 2 + 0.8, r2=bold_d / 2 + 0.1, h=top_washer_h + preview_fix * 2,
                                 center=False, _fn=180).up(top_washer_z - preview_fix)
    # middle_hole_slopy = middle_hole

    bottom_cutoff = cylinder(r=60, h=20).rotate([180, 0, 0])
    top_cutoff = cylinder(r=60, h=20).up(rotation_washer_h + compression_gap_h + rotation_washer_h)
    bottom_part = (rotation_washer - dummy_rod) - middle_hole
    top_bottom_threshold = bottom_cutoff + top_cutoff
    top_part = (top_washer + (
            pipe_1 + pipe_2) - compression_gap_washer - top_bottom_threshold - middle_hole) - dummy_rod - middle_hole_slopy

    rotation_stop_washer = cube(1)  # cylinder(r=joint_r, h=rotate_part_h/1.66)
    # num_triangles = 360  # You can adjust this value to change the number of triangles
    #
    # freq = 16
    # start_amp = 0.05
    # end_amp = 0.4
    # offset = 2
    # start_r = bold_d / 2
    # stop_r = joint_r
    # steps = 20
    # step_r = (stop_r - start_r) / steps
    # amp_step = (end_amp - start_amp) / steps
    # r_pairs = list(pairwise(seq(start_r, joint_r + step_r, step_r)))
    # for k in range(len(r_pairs)):
    #     r1, r2 = r_pairs[k]
    #     amp = start_amp + amp_step * k
    #     circle_coordinates = create_circle_coordinates(r1, r2, num_triangles)
    #     for i in range(len(circle_coordinates)):
    #         segment = linear_extrude(math.sin(circle_coordinates[i][4] * freq) * amp + offset)(
    #             polygon(circle_coordinates[i][0:4])).up(0)
    #         rotation_stop_washer += segment

    return [((bottom_part + top_part + rotation_stop_washer.rotate([180, 0, 0]), (-40, 80, 0)),
             "joint_half"),
            (((rotation_stop_washer) - middle_hole, (-40, -40, 0)), "rotation_stop_washer"),
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


def main(output_scad_basename, output_stl_basename, include_filter: Optional[str],  verbose:bool):
    output: List[StlTask] = [
        *middle_bolt(),
        *make_joint_half()
        # (middle_end_nut(), "middle_end_nut")
    ]
    save_to_stl_scad(output_scad_basename, output_stl_basename, output, verbose)


if __name__ == "__main__":
    args = parse_args()
    build_path: str = os.path.dirname(os.path.realpath(__file__))
    output_path: str = os.path.abspath(os.path.join(build_path, '..', 'build')) + os.path.sep

    if not os.path.exists(output_path):
        os.makedirs(output_path)

    stl_output_path: str | None = output_path
    if shutil.which("openscad") is None or args.skip_stl:
        stl_output_path = None
    main(output_scad_basename=output_path, output_stl_basename=stl_output_path, include_filter=args.include_filter, verbose=args.verbose)
