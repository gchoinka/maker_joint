import re
import sys
from itertools import chain, product
from typing import Iterable, Tuple, List, Optional
from solid2 import cube, P3, scad_inline
from solid2.core.object_base import OpenSCADObject
from pathlib import Path
import subprocess

RenderTask = Tuple[Tuple[OpenSCADObject, P3], str]


def _render_to_file(root: OpenSCADObject, filename: str, openscad_bin: str | None,
                    verbose: bool = False) -> str:
    scad_filename = Path(filename).with_suffix(".scad").absolute().as_posix()
    out_filenames = tuple(chain.from_iterable(
        product(("-o",), (Path(filename).with_suffix(ext).absolute().as_posix() for ext in (".3mf", ".png")))))
    root.save_as_scad(scad_filename)
    openscad_cli_args = [openscad_bin, *out_filenames, "--colorscheme", "BeforeDawn", scad_filename]

    try:
        subprocess.check_output(openscad_cli_args)
    except subprocess.CalledProcessError as ex:
        print(ex, file=sys.stderr)
    return Path(filename).absolute().as_posix()


def _render_task_function(render_to_file_args: Tuple[OpenSCADObject, str, str, bool]) -> str:
    (obj, filename, openscad_bin, verbose) = render_to_file_args
    return _render_to_file(obj, filename, openscad_bin, verbose)


def save_to_file(output_scad_basename: str, openscad_bin: str | None, output: Iterable[RenderTask],
                 all_filename: str = "all",
                 verbose: bool = False) -> None:
    if verbose:
        from multiprocessing.dummy import Pool
    else:
        from multiprocessing import Pool

    render_task: List[Tuple[OpenSCADObject, str, str, bool]] = []
    all_obj: Optional[OpenSCADObject] = None
    for obj_and_dim, obj_name in output:
        obj, position = obj_and_dim
        filename = output_scad_basename + obj_name
        if verbose:
            obj += scad_inline(f'echo("Writing {obj_name}");\n')
        render_task.append((obj, filename, openscad_bin, verbose))
        if all_obj is None:
            all_obj = obj.translate(position)
        else:
            all_obj += obj.translate(position)

    scad_filename = f"{output_scad_basename}{all_filename}.scad"
    if all_obj is not None and len(render_task) > 1:
        all_obj.save_as_scad(scad_filename)
    if openscad_bin is not None:
        with Pool() as pool:
            pool.map(_render_task_function, render_task)
