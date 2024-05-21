from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import pathlib
from typing import Dict, List, Tuple, Union

import pym3t

from m3t_tracker_ros.utils import params_to_dict

# Automatically generated file
from m3t_tracker_ros.m3t_tracker_ros_parameters import m3t_tracker_ros  # noqa: E402


@dataclass
class TrackedObject:
    id: str
    class_id: str
    body2world_pose: npt.NDArray[np.float32]


@dataclass
class KnownObjectConfig:
    id: str
    class_id: str
    optimizer_name: str
    histogram: pym3t.ColorHistogram


class CachedTracker:
    def __init__(self, params: m3t_tracker_ros.Params) -> None:
        self._params = params
        self._objects_with_valid_paths = self._check_dataset_path(
            self._params.dataset_path,
            self._params.tracked_objects,
            self._params.use_depth,
        )
        if len(self._objects_with_valid_paths != len(self._params.tracked_objects)):
            diff = set(self._params.tracked_objects) - set(
                self._objects_with_valid_paths.keys()
            )
            self.get_logger().error(f"Filed to load models for objects: {diff}!")

        self._params_dict = params_to_dict(self._params)

        # Initialize M3T tracker
        self._dummy_color_camera = pym3t.DummyColorCamera()
        self._dummy_color_camera.camera2world_pose = np.eye(4)
        if self._params.use_depth:
            self._dummy_depth_camera = pym3t.DummyDepthCamera()

        self._object_cache = self._initialize_object_cache(
            self._objects_with_valid_paths
        )
        if not self.tracker.SetUp():
            e = RuntimeError("Failed to initialize tracker!")
            self.get_logger().error(str(e))
            raise e

        # Iteration counter
        self._tracker_iter_cnt = 0

        # Initialize type counter
        self._object_type_counter = {id: 0 for id in self._object_cache.keys()}

        # Cashed objects
        self._known_object_histograms = Dict[str, KnownObjectConfig]

        # tracker = pym3t.Tracker("tracker", synchronize_cameras=False)
        # self.tracker.n_corr_iterations = self._params.tracker.n_corr_iterations
        # self.tracker.n_update_iterations = self._params.tracker.n_update_iterations

    def depth2color_pose(self, pose: npt.NDArray[np.float32]) -> None:
        self._dummy_depth_camera.camera2world_pose = pose

    def track_image(
        self,
        image_color: npt.NDArray[np.uint8],
        k_color: npt.NDArray[np.float32],
        image_depth: npt.NDArray[np.float16],
        k_depth: npt.NDArray[np.float32],
    ) -> List[TrackedObject]:
        image_color
        k_color
        image_depth
        k_depth

    def _clear_type_counter(self) -> None:
        self._object_type_counter = {id: 0 for id in self._object_cache.keys()}

    def _get_and_bump_type_counter(self, class_id: str) -> str:
        name = f"{class_id}_{self._object_type_counter[class_id]}"
        self._object_type_counter[class_id] += 1
        return name

    def update_tracked_objects(self, objects: List[TrackedObject]) -> None:
        self._clear_type_counter()
        body_names = {
            self._get_and_bump_type_counter(obj.class_id): obj.class_id
            for obj in objects
        }

        # Create list of unused optimizers
        body_names_set = set(body_names.keys())
        unused_optimizers_names = [
            opt.name
            for opt in self._tacker.optimizers
            if opt.name.removesuffix("_optimizer") in body_names_set
        ]

        # Remove unused
        for opt_name in unused_optimizers_names:
            self._tracker.DeleteOptimizer(opt_name)

        # Create new required optimizers
        optimizers_names = set(opt.name for opt in self._tacker.optimizers)
        objects_without_optimizer = {
            name: class_id
            for name, class_id in body_names.items()
            if name not in optimizers_names
        }

        # Match optimizer with objects
        new_optimizers = {
            name + "_optimizer": self._create_cached_optimizer(name, class_id)
            for name, class_id in objects_without_optimizer.items()
        }
        current_optimizers = {obj.name: obj for obj in self._tacker.optimizers}

        # Seed known objects with modalities
        for body_name, object in zip(body_names, objects):
            opt_name = body_name + "_optimizer"
            opt_dict = (
                new_optimizers if opt_name in new_optimizers else current_optimizers
            )

            # Use saved histograms
            if object.id != "":
                self._set_histograms(
                    opt_dict[opt_name],
                    self._known_object_histograms[object.id].histogram,
                )

            opt_dict[opt_name].root_link.body.body2world_pose = object.body2world_pose

        # Setup tracker
        for opt in new_optimizers:
            self._tracker.AddOptimizer(opt)
        self._tracker.SetUp()

    # def update_histograms(self, objects: List[TrackedObject]) -> None:
    #     histograms = {}

    def _create_cached_optimizer(self, name, class_id: str) -> pym3t.Optimizer:
        opt = self._object_cache[class_id]
        opt.name = name + "_optimizer"
        opt.root_link.name = name + "_link"
        opt.root_link.body.name = name + "_body"
        return opt

    def _set_histograms(
        self, optimizer: pym3t.Optimizer, histograms: pym3t.ColorHistograms
    ) -> pym3t.Optimizer:
        next(
            filter(
                lambda mod: "_region_model" in mod.name, optimizer.root_link.modalities
            )
        ).UseSharedColorHistograms(histograms)

    def _get_histograms(self, optimizer: pym3t.Optimizer) -> pym3t.ColorHistograms:
        return next(
            filter(
                lambda mod: "_region_model" in mod.name, optimizer.root_link.modalities
            )
        ).color_histograms

    def _initialize_object_cache(
        self, objects_with_valid_paths: Dict[str : Tuple[pathlib.Path]]
    ) -> Dict[str : pym3t.Optimizer]:
        object_cache = {}

        for object_name in objects_with_valid_paths.keys():
            object_files = objects_with_valid_paths[object_name]
            use_region_modality = self._params.get_entry(
                object_name
            ).use_region_modality
            use_depth_modality = (
                self._params.use_depth
                and self._params.get_entry(object_name).use_depth_modality
            )

            if not (use_region_modality or use_depth_modality):
                self.get_logger().warn(
                    f"Object '{object_name}' has no modality enabled! "
                    "It will not be used when tracking!"
                )
                continue

            body = pym3t.Body(
                name=object_name,
                geometry_path=object_files["obj"].as_posix(),
                geometry_unit_in_meter=self._params.geometry_unit_in_meter,
                geometry_counterclockwise=True,
                geometry_enable_culling=True,
                geometry2body_pose=np.eye(4),
            )
            link = pym3t.Link(object_name + "_link", body)

            if use_region_modality:
                region_model = pym3t.RegionModel(
                    object_name + "_region_model",
                    body,
                    object_files["m3t_rmb"].as_posix(),
                )
                region_modality = pym3t.RegionModality(
                    object_name + "_region_modality",
                    body,
                    self._dummy_color_camera,
                    region_model,
                )
                region_modality = self._update_object_config(
                    region_modality, self._params_dict[object_name]["region_modality"]
                )
                link.AddModality(region_modality)

            if use_depth_modality:
                depth_model = pym3t.DepthModel(
                    object_name + "_depth_model",
                    body,
                    object_files["m3t_dmb"].as_posix(),
                )
                depth_modality = pym3t.DepthModality(
                    object_name + "_depth_modality",
                    body,
                    self._dummy_depth_camera,
                    depth_model,
                )
                depth_modality = self._update_object_config(
                    depth_modality, self._params_dict[object_name]["depth_modality"]
                )
                link.AddModality(depth_modality)

            optimizer = pym3t.Optimizer(
                object_name + "_optimizer",
                link,
            )
            optimizer = self._update_object_config(
                optimizer, self._params_dict[object_name]["optimizer"]
            )

            object_cache[object_name] = optimizer

        if len(object_cache) == 0:
            e = RuntimeError(
                "All of the objects turned out to be invalid during configuration. "
                "Unable to start the node, exiting!"
            )
            self.get_logger().error(str(e))
            raise e

        return object_cache

    def _update_object_config(
        self,
        modality: Union[pym3t.RegionModality, pym3t.DepthModality, pym3t.Optimizer],
        params_dict: dict,
    ) -> Union[pym3t.RegionModality, pym3t.DepthModality, pym3t.Optimizer]:
        """Directly converts code-generated ROS parameters converted into dictionary
        into modality or optimizer data structure configuration.

        :param modality: Region or Depth modality or Optimizer configuration structure.
        :type modality: Union[pym3t.RegionModality, pym3t.DepthModality, pym3t.Optimizer]
        :param params_dict: Dictionary with ROS parameters extracted for a given modality
            type or optimizer configuration.
        :type params_dict: dict
        :return: Update modality config
        :rtype: Union[pym3t.RegionModality, pym3t.DepthModality, pym3t.Optimizer]
        """
        for key, val in params_dict.items():
            # Check if the bound object has a parameter
            if hasattr(modality, key):
                # Set new parameter value
                setattr(modality, key, val)

    def _check_dataset_path(
        self, dataset_path: pathlib.Path, tracked_objects: List[str], use_depth: bool
    ) -> Dict[str : Tuple[pathlib.Path]]:
        """Assigns paths to files required by tracked objects. hecks if files with
        extensions ``.obj`` and ``.m3t_rmb`` exists for objects specified with the parameter
        ``tracked_objects`` exists. If depth is expected objects with extension ``.m3t_dmb``
        are also checked.

        :param dataset_path: Path to the dataset of objects. With assumption the folder was
            already pre checked in parameter validation.
        :type dataset_path: pathlib.Path
        :param tracked_objects: List of object names to load to track.
        :type tracked_objects: List[str]
        :param use_depth: Whether to expect depth data.
        :type use_depth: bool

        :raises RuntimeError: Failed to math object name with filenames in the directory.
        :return: Dictionary with following structure:
            ``tracked_object_name``: {
                ``obj``: pathlib.Path,
                ``m3t_rmb``: pathlib.Path,
                ``m3t_dmb``: Union[None, pathlib.Path],
            }
            The value for ``m3t_dmb`` is set to none if ``use_depth`` is equal to False
        :rtype: Dict[str : Tuple[pathlib.Path]]
        """

        # Filter only files from the directory
        files_in_dir = [x for x in dataset_path if x.is_file()]
        objects_with_valid_paths = {}
        for object_name in tracked_objects:
            try:
                # ``.obj``: 3D mesh file
                # ``.m3t_rmb``: Precomputed region modality binary file
                # ``.m3t_dmb``: Precomputed depth modality binary file
                file_extensions = ["obj", "m3t_rmb"]
                if use_depth:
                    file_extensions.append("m3t_dmb")
                # Find all files matching object name and file extension
                object_files = {
                    extension: next(
                        file_name
                        for file_name in files_in_dir
                        if object_name + "." + extension in file_name.as_posix()
                    )
                    for extension in file_extensions
                }
                objects_with_valid_paths.update({object_name: object_files})
            except StopIteration:
                continue

        if len(objects_with_valid_paths) == 0:
            raise RuntimeError(
                "None of the provided objects could be found in the "
                f"folder '{dataset_path.as_posix()}'. Unable to start the node, exiting!"
            )

        return objects_with_valid_paths

    def _get_optimizer_color_histogram(
        self, optimizer: pym3t.Optimizer
    ) -> pym3t.ColorHistogram:
        return optimizer.root_link.modality[0].color_histograms[0]
