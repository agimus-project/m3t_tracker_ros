#!/usr/bin/env python

import argparse
import numpy as np
import pathlib
import pym3t
import trimesh


class colors:
    """Defines codes used by terminal to color the text."""

    GREEN = "\033[92m"
    RED = "\033[91m"
    WHITE = "\033[0m"


def print_c(text: str, color: colors) -> None:
    """Prints the text with applied color and brings back the terminal to the default white.

    :param text: Input text to color.
    :type text: str
    :param color: Color constant to apply to the text.
    :type color: colors
    """
    print(f"{color}{text}{colors.WHITE}")


def parse_script_input() -> argparse.Namespace:
    """Defines and parses input arguments.

    :return: Parsed input arguments.
    :rtype: argparse.Namespace
    """
    parser = argparse.ArgumentParser(
        prog="prepare_sparse_views",
        description="Convert any mesh to ``.obj`` format and create sparse views for it.",
    )
    parser.add_argument(
        "-i",
        "--input-path",
        dest="input_path",
        type=str,
        required=True,
        help="Path to a directory with objects to convert.",
    )
    parser.add_argument(
        "-o",
        "--output-path",
        dest="output_path",
        type=str,
        required=True,
        help="Path to which generated objects have to be stored.",
    )
    parser.add_argument(
        "-m",
        "--mesh-format",
        dest="mesh_format",
        type=str,
        default="ply",
        help="Mesh file extension indicating what format to look for. "
        + "[param is case sensitive].",
    )
    parser.add_argument(
        "-s",
        "--mesh-scale",
        dest="mesh_scale",
        type=float,
        default=0.001,
        help="Scale of the input mesh, used to normalize it",
    )
    parser.add_argument(
        "-d",
        "--use-depth",
        dest="use_depth",
        action="store_true",
        help="Whether to generate depth model of the objects.",
    )
    parser.add_argument(
        "--filter-objects",
        dest="filter_objects",
        type=str,
        nargs="+",
        help="Only convert objects with filtered names. "
        + "Passed as a list separated with spaces.",
    )
    parser.add_argument(
        "--overwrite",
        dest="overwrite",
        action="store_true",
        help="Overwrites previously created files.",
    )
    parser.add_argument(
        "--use-random-seed",
        dest="use_random_seed",
        action="store_true",
        help="If a random seed is used to sample points.",
    )
    parser.add_argument(
        "--sphere-radius",
        dest="sphere_radius",
        type=float,
        default=0.8,
        help="Distance from the object center to the center of a virtual camera.",
    )
    parser.add_argument(
        "--stride-depth-offset",
        dest="stride_depth_offset",
        type=float,
        default=0.002,
        help="Distance between points in meter that are used to calculate depth offsets.",
    )
    parser.add_argument(
        "--max-radius-depth-offset",
        dest="max_radius_depth_offset",
        type=float,
        default=0.05,
        help="Maximum radius in meter for the calculation of depth offsets that "
        + "are used for occlusion handling.",
    )
    parser.add_argument(
        "--n-divides",
        dest="n_divides",
        type=int,
        default=4,
        help="Number of times an icosahedron is divided to generate viewpoints "
        + "on a sphere. It thereby controls the number of template views.",
    )
    parser.add_argument(
        "--n-points",
        dest="n_points",
        type=int,
        default=200,
        help="Number of points that are sampled for each view.",
    )
    parser.add_argument(
        "--image-size",
        dest="image_size",
        type=int,
        default=2000,
        help="Size of images that are rendered for each view.",
    )

    return parser.parse_args()


def process_meshes() -> None:
    """Finds all the files expected to be converted, creates copies in ``.obj`` format
    and creates binary files with spare views for region and depth models.

    :raises IOError: Input folder doesn't exist.
    :raises IOError: Output folder doesn't exist.
    :raises IOError: Input folder doesn't have any files of expected type.
    """
    args = parse_script_input()

    input_path = pathlib.Path(args.input_path).absolute()
    if not input_path.is_dir():
        raise IOError(f"Path '{input_path.as_posix()}' doesn't exist!")

    output_folder_path = pathlib.Path(args.output_path).absolute()
    if not output_folder_path.is_dir():
        raise IOError(f"Path '{output_folder_path.as_posix()}' doesn't exist!")

    input_meshes_paths = input_path.glob(f"*.{args.mesh_format}")
    input_meshes_paths = [f for f in input_meshes_paths if f.is_file()]

    if args.filter_objects is not None:
        input_meshes_paths = [
            f for f in input_meshes_paths if f.stem in args.filter_objects
        ]
    if len(input_meshes_paths) == 0:
        raise IOError(
            f"No files of a type '.{args.mesh_format}' found in the source directory!"
        )

    # Sort by file names
    input_meshes_paths.sort(key=lambda path: path.stem)

    for input_file in input_meshes_paths:
        object_name = input_file.stem
        output_path = (output_folder_path / object_name).as_posix()
        output_obj_path = output_path + ".obj"
        output_m3t_rmb_path = output_path + ".m3t_rmb"  # M3T Region Model Binary
        output_m3t_dmb_path = output_path + ".m3t_dmb"  # M3T Depth Model Binary

        for file in [output_obj_path, output_m3t_rmb_path, output_m3t_dmb_path]:
            f = pathlib.Path(file)
            if f.is_file() and args.overwrite:
                f.unlink()

        # Convert given mesh file to ``.obj`` file format
        mesh = trimesh.load(input_file, force="mesh").apply_scale(args.mesh_scale)
        mesh.export(output_obj_path)

        body = pym3t.Body(
            name=object_name,
            geometry_path=output_obj_path,
            geometry_unit_in_meter=1.0,
            geometry_counterclockwise=1,
            geometry_enable_culling=1,
            geometry2body_pose=np.eye(4),
        )
        body.SetUp()

        # Generate Region Model and save it in ``.m3t_rmb`` file
        pym3t.RegionModel(
            object_name,
            body,
            output_m3t_rmb_path,
            args.sphere_radius,
            args.n_divides,
            args.n_points,
            args.max_radius_depth_offset,
            args.stride_depth_offset,
            args.use_random_seed,
            args.image_size,
        ).SetUp()

        if args.use_depth:
            # Generate Region Model and save it in ``.m3t_dmb`` file
            pym3t.DepthModel(
                object_name,
                body,
                output_m3t_dmb_path,
                args.sphere_radius,
                args.n_divides,
                args.n_points,
                args.max_radius_depth_offset,
                args.stride_depth_offset,
                args.use_random_seed,
                args.image_size,
            ).SetUp()

    print()  # Empty line
    depth_info = ", '.m3t_dmb'" if args.use_depth else ""
    print("Generated files of a type: ['.obj', '.m3t_rmb'" + depth_info + "]")
    print("Resulting object classes:")
    for input_file in input_meshes_paths:
        print(f"\t{input_file.stem}")


def main() -> None:
    """Entry point function for the CLI."""
    try:
        process_meshes()
    except Exception as err:
        print_c(str(err), colors.RED)
        exit(2)
    print_c("All files were successfully converted.", colors.GREEN)
    exit(0)


if __name__ == "__main__":
    main()
