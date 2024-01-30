from dataclasses import dataclass
import re
import sys
from itertools import chain, product
from typing import Iterable, List, Optional, Generator
from solid2 import P3, scad_inline
from solid2.core.object_base import OpenSCADObject
from pathlib import Path
import subprocess


@dataclass
class RenderTask:
    scad_object: OpenSCADObject
    position: P3
    filename: Path


@dataclass
class _RenderTaskArgs:
    scad_object: OpenSCADObject
    position: P3
    filename: Path
    openscad_bin: Path | None
    verbose: bool


def make_RenderTaskArgs(task: RenderTask, openscad_bin: Path | None, verbose: bool) -> _RenderTaskArgs:
    return _RenderTaskArgs(task.scad_object, task.position, task.filename, openscad_bin, verbose)


def modify_render_task(render_tasks: Iterable[RenderTask], offset: P3 = (0.0, 0.0, 0.0), name_suffix: str = "") -> \
Generator[
    RenderTask, None, None]:
    for task in render_tasks:
        pos = task.position
        new_pos = (pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2])
        new_name = Path(f"{task.filename}{name_suffix}")
        yield RenderTask(task.scad_object, new_pos, Path(new_name))


def _render_to_file(task: _RenderTaskArgs) -> Path:
    scad_filename = task.filename.with_suffix(".scad").absolute().as_posix()
    task.scad_object.save_as_scad(scad_filename)
    if task.openscad_bin is not None:
        out_filenames = tuple(chain.from_iterable(
            product(("-o",), (task.filename.with_suffix(ext).absolute().as_posix() for ext in (".3mf", ".png")))))
        openscad_cli_args = [task.openscad_bin, *out_filenames, "--colorscheme", "BeforeDawn", scad_filename]

        try:
            subprocess.check_output(openscad_cli_args)
        except subprocess.CalledProcessError as ex:
            print(ex, file=sys.stderr)
    return task.filename.absolute()


def save_to_file(output_scad_basename: Path, openscad_bin: Path | None, render_tasks: Iterable[RenderTask],
                 all_filename: Path = Path("all"), include_filter_regex: re.Pattern[str] | None = None,
                 verbose: bool = False) -> None:
    if verbose:
        from multiprocessing.dummy import Pool
    else:
        from multiprocessing import Pool

    render_tasks_args: List[_RenderTaskArgs] = [make_RenderTaskArgs(t, openscad_bin, verbose) for t in render_tasks]
    if include_filter_regex is not None:
        render_tasks_args = [t for t in render_tasks_args if include_filter_regex.search(t.filename.as_posix())]

    if verbose:
        print(f"Will generate ", end="")
        for task in render_tasks_args:
            print(f"{task.filename} ", end="")
        print()

    all_obj: Optional[OpenSCADObject] = None
    for task_args in render_tasks_args:
        task_args.filename = output_scad_basename.joinpath(task_args.filename)
        if verbose:
            task_args.scad_object += scad_inline(f'echo("Writing {task_args.filename.as_posix()}");\n')
        if all_obj is None:
            all_obj = task_args.scad_object.translate(task_args.position)
        else:
            all_obj += task_args.scad_object.translate(task_args.position)

    scad_filename = f"{output_scad_basename}{all_filename}.scad"
    if all_obj is not None and len(render_tasks_args) > 1:
        all_obj.save_as_scad(scad_filename)

    with Pool() as pool:
        pool.map(_render_to_file, render_tasks_args)
