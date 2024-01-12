import re
import sys
from typing import Iterable, Tuple, List, Optional
from solid2 import cube, P3, scad_inline
from solid2.core.object_base import OpenSCADObject
from pathlib import Path
import subprocess

StlTask = Tuple[Tuple[OpenSCADObject, P3], str]


def _render_to_stl_file(root: OpenSCADObject, stl_filename: str, openscad_bin: str | None,
                        verbose: bool = False) -> str:
    png_file = Path(stl_filename).with_suffix(".png")
    if verbose and openscad_bin is not None:
        scad_filename = Path(stl_filename).with_suffix(".scad")
        root.save_as_scad(scad_filename.absolute().as_posix())
        openscad_cli_args = [openscad_bin, "-o", stl_filename, "-o", png_file, scad_filename]

        try:
            subprocess.check_output(openscad_cli_args)
        except subprocess.CalledProcessError as ex:
            print(ex, file=sys.stderr)
        return Path(stl_filename).absolute().as_posix()
    else:
        return root.save_as_stl(stl_filename)


def _stl_task_function(render_to_stl_file_args: Tuple[OpenSCADObject, str, str,  bool]) -> str:
    (obj, filename, openscad_bin, verbose) = render_to_stl_file_args
    return _render_to_stl_file(obj, filename, openscad_bin, verbose)


def save_to_stl_scad(output_scad_basename: str, openscad_bin: str | None, output: Iterable[StlTask],
                     all_filename: str = "all",
                     verbose: bool = False) -> None:
    if verbose:
        from multiprocessing.dummy import Pool
    else:
        from multiprocessing import Pool

    stl_task: List[Tuple[OpenSCADObject, str, str, bool]] = []
    all_obj: Optional[OpenSCADObject] = None
    for obj_and_dim, obj_name in output:
        obj, position = obj_and_dim
        scad_filename = output_scad_basename + obj_name + ".scad"
        stl_filename = output_scad_basename + obj_name + ".stl"
        if verbose:
            obj += scad_inline(f'echo("Writing {obj_name}");\n')
        obj.save_as_scad(scad_filename)
        stl_task.append((obj, stl_filename, openscad_bin, verbose))
        if all_obj is None:
            all_obj = obj.translate(position)
        else:
            all_obj += obj.translate(position)

    scad_filename = f"{output_scad_basename}{all_filename}.scad"
    # stl_filename = f"{output_scad_basename}{all_filename}.stl"
    if all_obj is not None and len(stl_task) > 1:
        all_obj.save_as_scad(scad_filename)
        # stl_task.append((all_obj, stl_filename, verbose))
    if openscad_bin is not None:
        with Pool() as pool:
            pool.map(_stl_task_function, stl_task)
