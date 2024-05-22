from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import pathlib
import time
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
    class_id: str
    last_used_stamp: float
    histogram: pym3t.ColorHistogram


@dataclass
class OptimizerParams:
    class_id: str
    last_used_stamp: float


class CachedTracker:
    def __init__(self, params: m3t_tracker_ros.Params) -> None:
        self._params = params
        self._classes_with_valid_paths = self._check_dataset_path(
            self._params.dataset_path,
            self._params.tracked_objects,
            self._params.use_depth,
        )
        if len(self._classes_with_valid_paths != len(self._params.tracked_objects)):
            diff = set(self._params.tracked_objects) - set(
                self._classes_with_valid_paths.keys()
            )
            self.get_logger().error(f"Filed to load models for objects: {diff}!")

        self._params_dict = params_to_dict(self._params)

        # Initialize M3T tracker
        self._dummy_color_camera = pym3t.DummyColorCamera()
        self._dummy_color_camera.camera2world_pose = np.eye(4)
        if self._params.use_depth:
            self._dummy_depth_camera = pym3t.DummyDepthCamera()

        self._object_cache = self._initialize_object_cache(
            self._classes_with_valid_paths
        )
        if not self.tracker.SetUp():
            e = RuntimeError("Failed to initialize tracker!")
            self.get_logger().error(str(e))
            raise e

        # Iteration counter
        self._tracker_iter_cnt = 0
        self._histogram_timeout = 5.0
        self._optimizer_lifetime = 5.0

        # Initialize type counter
        self._object_type_counter = {id: 0 for id in self._object_cache.keys()}

        # Cashed objects
        self._last_objects_order = List[TrackedObject]
        self._assigned_optimizers_names = List[str]
        self._known_objects_configs = Dict[str, KnownObjectConfig]
        self._registered_optimizers = Dict[str, OptimizerParams]

        self._do_track_objects = False
        self._first_setup = True
        self._tracker = pym3t.Tracker("tracker", synchronize_cameras=False)
        self.tracker.n_corr_iterations = self._params.tracker.n_corr_iterations
        self.tracker.n_update_iterations = self._params.tracker.n_update_iterations

    def track_image(
        self,
        image_color: npt.NDArray[np.uint8],
        k_color: npt.NDArray[np.float32],
        image_depth: Union[npt.NDArray[np.float16], None],
        k_depth: Union[npt.NDArray[np.float32], None],
        depth2color_pose: Union[npt.NDArray[np.float32], None],
    ) -> List[TrackedObject]:
        if not self._tracker.set_up:
            return None

        self._dummy_color_camera.image = image_color
        self._dummy_color_camera.intrinsics = self._image_data_to_intrinsics(
            k_color, image_color.shape
        )

        if image_depth and k_depth and depth2color_pose:
            self._dummy_depth_camera.camera2world_pose = depth2color_pose
            self._dummy_depth_camera.image = image_depth
            self._dummy_depth_camera.intrinsics = self._image_data_to_intrinsics(
                k_depth, image_depth.shape
            )

        # 0 is just a dummy value, as it is not used in the C++ code
        if not self._tracker.UpdateCameras(0):
            raise RuntimeError("Received images are invalid!")

        if self._self._tracker_iter_cnt == 0:
            self._tracker.ExecuteStartingStep(self._self._tracker_iter_cnt)

        self._tracker.ExecuteTrackingStep(self._self._tracker_iter_cnt)
        self._self._tracker_iter_cnt += 1

        # Create a map of optimizer name to held body pose
        optimizers = {
            optimizer.name: optimizer.root_link.body[0].body2world_pose
            for optimizer in self._tracker.optimizer
        }
        # Update body poses
        for i in range(len(self._last_objects_order)):
            optimizer_name = self._assigned_optimizers_names[i]
            self._last_objects_order[i].body2world_pose = optimizers[optimizer_name]
        return self._last_objects_order

    def update_tracked_objects(self, objects: List[TrackedObject]) -> None:
        # If nothing to track skip the rest
        self._last_objects_order = objects
        self._do_track_objects = len(objects) != 0
        if not self._do_track_objects:
            return

        now = time.time()
        self._clear_type_counter()
        self._assigned_optimizers_names = [""] * len(self._last_objects_order)
        optimizer_names = [optimizer.name for optimizer in self._tracker.optimizer]
        for i, obj in enumerate(self._last_objects_order):
            name = self._get_and_bump_type_counter(obj.class_id)
            obj_optimizer_name = name + "_optimizer"
            self._assigned_optimizers_names[i] = [obj_optimizer_name]
            # If optimizer for a given object type was already registered, fetch it
            if obj_optimizer_name in optimizer_names:
                optimizer = next(
                    filter(
                        lambda optimizer: optimizer.name == obj_optimizer_name,
                        self._tracker.optimizer,
                    )
                )
                self._tracker.DeleteOptimizer(obj_optimizer_name)
            # If no optimizer was found create a new one
            else:
                optimizer = self._assemble_object_optimizer(name, obj.class_id)

            # Object has its own track
            if obj.id != "":
                # If tracked object is seen for the first time initialize it
                if obj.id not in self._known_objects_configs:
                    self._known_objects_configs = KnownObjectConfig(
                        class_id=obj.class_id,
                        last_used_stamp=now,
                        histogram=pym3t.ColorHistogram(),
                    )
                old_class_id = self._known_objects_configs[object.id].class_id
                if old_class_id != obj.class_id:
                    raise RuntimeError(
                        f"'class_id' of tracked object '{object.id}' "
                        f"changed from '{obj.class_id}' to '{old_class_id}'!"
                    )
                # Histogram is still valid and not too old
                if (
                    now - self._known_objects_configs[object.id].last_used_stamp
                    < self._histogram_timeout
                ):
                    histogram = self._known_objects_configs[object.id].histogram
                self._known_objects_configs[object.id].last_used_stamp = now
            # Unknown object or histogram is no longer valid
            else:
                histogram = pym3t.ColorHistogram()
            histogram.SetUp()

            # Find region modality object and set its histogram
            modality = next(
                filter(
                    lambda modality: "_region_model" in modality.name,
                    optimizer.root_link.modalities,
                )
            )
            modality.UseSharedColorHistograms(histogram)
            modality.SetUp()
            optimizer.root_link.SetUp()

            optimizer.root_link.body[0].body2world_pose = object.body2world_pose
            optimizer.SetUp()

            self._tracker.AddOptimizer(optimizer)
            self._registered_optimizers[optimizer.name] = OptimizerParams(
                class_id=obj.class_id, last_used_stamp=now
            )

        # Remove no longer used optimizers. Update of the list is not needed
        for optimizer_name in optimizer_names:
            if (
                now - self._registered_optimizers[optimizer_name].last_used_stamp
                < self._optimizer_lifetime
            ):
                self._tracker.DeleteOptimizer(optimizer_name)

        if not self._tracker.SetUp(set_up_all_objects=self._first_setup):
            raise RuntimeError(
                "Failed to " "initialize"
                if self._first_setup
                else "reinitialize updated " "tracker!"
            )
        self._first_setup = False

    def store_known_object_data(self, objects: List[TrackedObject]) -> None:
        optimizers = {
            optimizer.name: optimizer for optimizer in self._tracker.optimizer
        }
        known_objects = [(i, obj) for i, obj in enumerate(objects) if obj.id != ""]
        for i, obj in known_objects:
            optimizer = optimizers[self._assigned_optimizers_names[i]]
            self._known_objects_configs[obj.id].histogram = next(
                filter(
                    lambda modality: "_region_model" in modality.name,
                    optimizer.root_link.modalities,
                )
            ).color_histograms[0]

    def _image_data_to_intrinsics(
        self, camera_k: npt.NDArray[np.float64], im_shape: Tuple[int]
    ) -> pym3t.Intrinsics:
        """Converts ROS camera info message into pym3t Intrinsics object.

        :param camera_info: ROS message with camera data fields.
        :type camera_info: sensor_msgs.msg.CameraInfo
        :return: M3T object holding intrinsics parameters of the camera.
        :rtype: pym3t.Intrinsics
        """
        return pym3t.Intrinsics(
            fu=camera_k[0],
            fv=camera_k[4],
            ppu=camera_k[2],
            ppv=camera_k[5],
            width=im_shape[0],
            height=im_shape[1],
        )

    def _clear_type_counter(self) -> None:
        self._object_type_counter = {id: 0 for id in self._object_cache.keys()}

    def _get_and_bump_type_counter(self, class_id: str) -> str:
        name = f"{class_id}_{self._object_type_counter[class_id]}"
        self._object_type_counter[class_id] += 1
        return name

    def _match_class_types(self, class_id: str) -> Tuple[bool]:
        class_params = self._params.get_entry(class_id)
        optimizer_class = "global" if class_params.use_global_optimizer else class_id
        rm_class = "global" if class_params.use_global_region_modality else class_id
        dm_class = "global" if class_params.use_global_depth_modality else class_id
        return optimizer_class, rm_class, dm_class

    def _get_used_modalities(self, class_id) -> bool:
        _, region_modality_class, depth_modality_class = self._match_class_types(
            class_id
        )
        use_region_modality = self._params.get_entry(
            region_modality_class
        ).use_region_modality
        use_depth_modality = (
            self._params.use_depth
            and self._params.get_entry(depth_modality_class).use_depth_modality
        )
        return use_region_modality, use_depth_modality

    def update_params(self, params: m3t_tracker_ros.Params) -> None:
        self._params = params
        self._params_dict = params_to_dict(self._params)

        self.tracker.n_corr_iterations = self._params.tracker.n_corr_iterations
        self.tracker.n_update_iterations = self._params.tracker.n_update_iterations

        optimizers_with_class = [
            (optimizer, self._registered_optimizers[optimizer.name])
            for optimizer in self._tracker.optimizer
            if optimizer.name in self._registered_optimizers.keys()
        ]
        for optimizer, class_id in optimizers_with_class:
            optimizer_class, region_modality_class, depth_modality_class = (
                self._match_class_types(class_id)
            )

            if not any(self._get_used_modalities(class_id)):
                continue

            self._tracker.DeleteOptimizer(optimizer.name)
            optimizer = self._update_object_config(
                optimizer, self._params_dict[optimizer_class]["optimizer"]
            )

            for idx, modality in enumerate(optimizer.root_link.modality):
                if "region_modality" in modality.name:
                    optimizer.root_link.modality[idx] = self._update_object_config(
                        optimizer.root_link.modality[idx],
                        self._params_dict[region_modality_class]["region_modality"],
                    )
                if "depth_modality" in modality.name:
                    optimizer.root_link.modality[idx] = self._update_object_config(
                        optimizer.root_link.modality[idx],
                        self._params_dict[depth_modality_class]["depth_modality"],
                    )
                optimizer.root_link.modality[idx].SetUp()

            optimizer.SetUp()
            self._tracker.AddOptimizer(optimizer)

        # If any objects were update reinitialize the tracker
        if len(optimizers_with_class) > 0:
            if not self._tracker.SetUp(set_up_all_objects=self._first_setup):
                raise RuntimeError("Failed to reinitialize updated tracker!")

    def _assemble_object_optimizer(
        self, object_name: str, class_id: str
    ) -> pym3t.Optimizer:
        optimizer_class, region_modality_class, depth_modality_class = (
            self._match_class_types(class_id)
        )

        use_region_modality, use_depth_modality = self._get_used_modalities(class_id)
        if not (use_region_modality or use_depth_modality):
            raise RuntimeError(
                f"Object type '{class_id}' has no modality enabled! "
                "It will not be used when tracking!"
            )

        object_files = self._classes_with_valid_paths[class_id]
        body = pym3t.Body(
            name=object_name,
            geometry_path=object_files["obj"].as_posix(),
            geometry_unit_in_meter=self._params.geometry_unit_in_meter,
            geometry_counterclockwise=True,
            geometry_enable_culling=True,
            geometry2body_pose=np.eye(4),
        )
        body.SetUp()
        link = pym3t.Link(object_name + "_link", body)

        if use_region_modality:
            region_model = pym3t.RegionModel(
                object_name + "_region_model",
                body,
                object_files["m3t_rmb"].as_posix(),
            )
            region_model.SetUp()
            region_modality = pym3t.RegionModality(
                object_name + "_region_modality",
                body,
                self._dummy_color_camera,
                region_model,
            )
            region_modality = self._update_object_config(
                region_modality,
                self._params_dict[region_modality_class]["region_modality"],
            )
            region_modality.SetUp()
            link.AddModality(region_modality)

        if use_depth_modality:
            depth_model = pym3t.DepthModel(
                object_name + "_depth_model",
                body,
                object_files["m3t_dmb"].as_posix(),
            )
            depth_model.SetUp()
            depth_modality = pym3t.DepthModality(
                object_name + "_depth_modality",
                body,
                self._dummy_depth_camera,
                depth_model,
            )
            depth_modality = self._update_object_config(
                depth_modality,
                self._params_dict[depth_modality_class]["depth_modality"],
            )
            depth_modality.SetUp()
            link.AddModality(depth_modality)

        optimizer = pym3t.Optimizer(
            object_name + "_optimizer",
            link,
        )
        optimizer = self._update_object_config(
            optimizer, self._params_dict[optimizer_class]["optimizer"]
        )

        return optimizer

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
            # Skip object called global as it is only for purposes of using parameters
            if object_name == "global":
                continue

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
