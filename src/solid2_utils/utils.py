import re
from typing import Iterable, Tuple, List, Optional
from solid2 import cube, P3, scad_inline
from solid2.core.object_base import OpenSCADObject
from pathlib import Path
import subprocess

StlTask = Tuple[Tuple[OpenSCADObject, P3], str]


def _render_to_stl_file(root: OpenSCADObject, filename: str, verbose: bool = False) -> str:
    if verbose:
        scad_file = Path(filename).with_suffix(".scad")
        root.save_as_scad(scad_file.absolute().as_posix())
        args = ["openscad", "-o", filename, scad_file]

        try:
            subprocess.check_output(args)
        except subprocess.CalledProcessError as ex:
            print(ex)
        return Path(filename).absolute().as_posix()
    else:
        return root.save_as_stl(filename)


def _stl_task_function(stl_task: Tuple[OpenSCADObject, str, bool]) -> str:
    (obj, filename, verbose) = stl_task
    return _render_to_stl_file(obj, filename, verbose)


def save_to_stl_scad(output_scad_basename: str, output_stl_basename: str | None, output: Iterable[StlTask],
                     verbose: bool = False) -> None:
    if verbose:
        from multiprocessing.dummy import Pool
    else:
        from multiprocessing import Pool

    stl_task: List[Tuple[OpenSCADObject, str, bool]] = []
    all_obj: Optional[OpenSCADObject] = None
    for obj_and_dim, obj_name in output:
        obj, position = obj_and_dim
        scad_filename = output_scad_basename + obj_name + ".scad"
        stl_filename = output_scad_basename + obj_name + ".stl"
        if verbose:
            obj += scad_inline(f"""echo("Writing {obj_name}");\n""")
        obj.save_as_scad(scad_filename)
        stl_task.append((obj, stl_filename, verbose))
        if all_obj is None:
            all_obj = obj.translate(position)
        else:
            all_obj += obj.translate(position)

    scad_filename = f"{output_scad_basename}all.scad"
    stl_filename = f"{output_scad_basename}all.stl"
    if all_obj is not None and len(stl_task) > 1:
        all_obj.save_as_scad(scad_filename)
        stl_task.append((all_obj, stl_filename, verbose))
    if output_stl_basename is not None:
        with Pool() as pool:
            pool.map(_stl_task_function, stl_task)
