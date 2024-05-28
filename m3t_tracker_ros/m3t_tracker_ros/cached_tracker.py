from dataclasses import dataclass
import numpy as np
import numpy.typing as npt
import pathlib
import time
from typing import Dict, List, Tuple, Union

import pym3t


@dataclass
class TrackedObject:
    """Structure holding information about currently tracked object.
    Used to pass objects to the ``CachedTracker`` class."""

    # Unique identifier of an object. Not not known set to empty string
    id: str
    # Class id of tracked object
    class_id: str
    # Pose relative to camera frame
    body2camera_pose: npt.NDArray[np.float64]


@dataclass
class KnownObjectConfig:
    """Structure to hold data about objects with unique identifiers.
    Used to cache their histograms."""

    # Class id of tracked object to compare if it didn't change
    class_id: str
    # Time stamp when it was last see
    last_used_stamp: float
    # Histogram used during last encounter
    histogram: Union[None, pym3t.ColorHistogram] = None


@dataclass
class OptimizerParams:
    """Structure to hold parameters of the optimizers.
    Used for time caching of the pym3t.Optimizer objects."""

    class_id: str
    last_used_stamp: float


class CachedTracker:
    """Wrapper on top of pym3t.Tracker class. Adds ability to dynamically add and remove
    tracked objects with temporal cashing for performance.
    """

    def __init__(self, params: dict) -> None:
        """Checks validity of passed configuration of the pym3t.Tracker and initializes it.

        :param params: Dictionary with configuration of the tracker.
        :type params: dict
        :raises RuntimeError: Failed to load objects from the given path.
        """
        self._params = params
        self._classes_with_valid_paths = self._check_dataset_path(
            self._params["dataset_path"],
            self._params["tracked_objects"],
            self._params["use_depth"],
        )
        if len(self._classes_with_valid_paths != len(self._params["tracked_objects"])):
            diff = set(self._params["tracked_objects"]) - set(
                self._classes_with_valid_paths.keys()
            )
            raise RuntimeError(f"Filed to load models for objects: {diff}!")

        # Tracker OpenGL interface
        self._renderer_geometry = pym3t.RendererGeometry("renderer_geometry")

        # Initialize M3T tracker
        self._dummy_color_camera = pym3t.DummyColorCamera()
        self._dummy_color_camera.camera2world_pose = np.eye(4)

        if self._params["use_depth"]:
            self._dummy_depth_camera = pym3t.DummyDepthCamera()
            self._focused_depth_depth_renderer = pym3t.FocusedBasicDepthRenderer(
                f"focused_depth_depth_renderer_{self._dummy_depth_camera.name}",
                self._renderer_geometry,
                self._dummy_depth_camera,
            )
        else:
            self._focused_color_depth_renderer = pym3t.FocusedBasicDepthRenderer(
                f"focused_color_depth_renderer_{self._dummy_color_camera.name}",
                self._renderer_geometry,
                self._dummy_color_camera,
            )
        self._color_silhouette_renderer = pym3t.FocusedSilhouetteRenderer(
            "color_silhouette_renderer",
            self._renderer_geometry,
            self._dummy_color_camera.name,
        )

        self._object_cache = self._initialize_object_cache(
            self._classes_with_valid_paths
        )

        # Iteration counter
        self._tracker_iter_cnt = 0

        # Type counter
        self._object_type_counter = {id: 0 for id in self._object_cache.keys()}

        # Cache timeouts
        self._histogram_timeout = 5.0
        self._optimizer_lifetime = 5.0

        # Cashed objects
        # Stores currently tracked object references
        self._last_objects_order = List[TrackedObject]
        # List with optimizer names in the same order as objects from
        # ``self._last_objects_order`` to which given optimizer is assigned
        self._assigned_optimizers_names = List[str]
        # Dict with configurations of objects with known ``id```
        self._known_objects_configs = Dict[str, KnownObjectConfig]
        # Stores time stamps of optimizers that don't need reinitialization
        self._registered_optimizers = Dict[str, OptimizerParams]

        self._do_track_objects = False
        self._first_setup = True
        self._tracker = pym3t.Tracker("tracker", synchronize_cameras=False)
        self._tracker.n_corr_iterations = self._params["tracker"]["n_corr_iterations"]
        self._tracker.n_update_iterations = self._params["tracker"][
            "n_update_iterations"
        ]

    def track_image(
        self,
        color_image: npt.NDArray[np.uint8],
        color_camera_k: npt.NDArray[np.float32],
        depth_image: Union[npt.NDArray[np.float16], None],
        depth_camera_k: Union[npt.NDArray[np.float32], None],
        depth2color_pose: Union[npt.NDArray[np.float32], None],
    ) -> List[TrackedObject]:
        """Performs tracking step over single set of images.

        :param color_image: OpenCV style RBG8 color image.
        :type color_image: npt.NDArray[np.uint8]
        :param color_camera_k: Matrix with intrinsic parameters of the color camera.
        :type color_camera_k: npt.NDArray[np.float64]
        :param depth_image: OpenCV style CV_16UC1 depth image. None if not used.
        :type depth_image: Union[None, npt.NDArray[np.float16]]
        :param depth_camera_k:  Matrix with intrinsic parameters of the depth camera.
            None if not used.
        :type depth_camera_k: Union[None, npt.NDArray[np.float64]]
        :param depth2color_pose: Pose between depth camera and color camera.
            None if not used.
        :type depth2color_pose: Union[None, npt.NDArray[np.float32]]
        :raises RuntimeError: Tracker is not set up.
        :raises RuntimeError: Passed images are invalid and tracker failed to do next step.
        :return: List of refined poses, based on internally stored data.
        :rtype: List[TrackedObject]
        """
        if not self._tracker.set_up:
            return RuntimeError("Tracker was not set up!")

        # If nothing to track raise an exception
        if len(self._last_objects_order) == 0:
            return RuntimeError("Nothing to track!")

        self._dummy_color_camera.image = color_image
        self._dummy_color_camera.intrinsics = self._image_data_to_intrinsics(
            color_camera_k, color_image.shape
        )

        if depth_image and depth_camera_k and depth2color_pose:
            self._dummy_depth_camera.camera2world_pose = depth2color_pose
            self._dummy_depth_camera.image = depth_image
            self._dummy_depth_camera.intrinsics = self._image_data_to_intrinsics(
                depth_camera_k, depth_image.shape
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
            self._last_objects_order[i].body2camera_pose = optimizers[optimizer_name]
        return self._last_objects_order

    def update_tracked_objects(
        self, objects: List[TrackedObject], only_update_poses: bool
    ) -> None:
        """Updates internal list of tracked objects. Creates new optimizer if given object
        type is missing one. Removes optimizers if not sued for long time and manages
        history of histograms for known objects.

        :param objects: List of the objects to store for tracking.
        :type objects: List[TrackedObject]
        :raises RuntimeError: Class id of known object changed.
        :raises RuntimeError: Reinitialization of the tracker failed.
        """
        # If nothing to track skip the rest
        self._last_objects_order = objects
        if len(self._last_objects_order) == 0:
            return

        now = time.time()
        # Clear the counter to reassign labels once again
        self._clear_type_counter()
        # Reset list with ordered optimizers names
        self._assigned_optimizers_names = [""] * len(self._last_objects_order)
        # Fetch list of names of the optimizers with their pointers
        optimizer_names = [optimizer.name for optimizer in self._tracker.optimizer]

        for i, obj in enumerate(self._last_objects_order):
            if not only_update_poses:
                # Generate new "unique" name for a given object
                name = self._get_and_bump_type_counter(obj.class_id)
                obj_optimizer_name = name + "_optimizer"
                # Save name of the optimizer in the same order as the received object
                self._assigned_optimizers_names[i] = [obj_optimizer_name]
                # If optimizer for a given object type was already registered, fetch it
                if obj_optimizer_name in optimizer_names:
                    optimizer = next(
                        filter(
                            lambda optimizer: optimizer.name == obj_optimizer_name,
                            self._tracker.optimizer,
                        )
                    )
                    # Remove optimizer of a given name from the tracker
                    self._tracker.DeleteOptimizer(obj_optimizer_name)
                # If no optimizer was found create a new one
                else:
                    optimizer = self._assemble_object_optimizer(name, obj.class_id)

                # Find pointer region modality object to configure its histogram
                region_modality = next(
                    filter(
                        lambda modality: "_region_model" in modality.name,
                        optimizer.root_link.modalities,
                    )
                )
                optimizer.root_link.optimizer.DeleteModality(region_modality.name)

                # Object has its own track
                if obj.id != "":
                    # If tracked object is seen for the first time initialize it
                    if obj.id not in self._known_objects_configs:
                        obj_cfg = KnownObjectConfig(
                            class_id=obj.class_id,
                            last_used_stamp=now,
                        )
                        obj_cfg.histogram = self._reload_histogram_params(
                            obj_cfg, name + "_" + obj.id + "_color_histogram"
                        )
                        self._known_objects_configs[obj.id] = obj_cfg

                    assigned_class_id = self._known_objects_configs[object.id].class_id
                    if assigned_class_id != obj.class_id:
                        raise RuntimeError(
                            f"'class_id' of tracked object '{object.id}' "
                            f"changed from '{obj.class_id}' to '{assigned_class_id}'!"
                        )
                    histogram = self._known_objects_configs[object.id].histogram
                    # Histogram is too old to be considered still valid, reset it
                    if (
                        now - self._known_objects_configs[object.id].last_used_stamp
                        > self._histogram_timeout
                    ):
                        # SetUp clears values stored in the histogram
                        histogram.SetUp()
                    self._known_objects_configs[object.id].last_used_stamp = now
                    region_modality.UseSharedColorHistograms(histogram)
                # Unknown object or histogram is no longer valid
                else:
                    region_modality.DoNotUseSharedColorHistograms()

                region_modality.SetUp()
                optimizer.root_link.optimizer.AddModality(region_modality)
                optimizer.root_link.SetUp()

            optimizer.root_link.body[0].body2world_pose = object.body2camera_pose
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
        """Stores histograms of objects with known ids.

        :param objects: List of objects to extract the known ones.
        :type objects: List[TrackedObject]
        """
        # Filter objects with known ids
        known_objects = [(i, obj) for i, obj in enumerate(objects) if obj.id != ""]
        for i, obj in known_objects:
            # Match optimizers given to the object and extract its name
            object_name = self._assigned_optimizers_names[i].remove_suffix("_optimizer")
            # Convert the name to histogram's name
            hist_name = object_name + "_" + obj.id + "_color_histogram"
            # Find histogram with the given name
            self._known_objects_configs[obj.id].histogram = next(
                filter(
                    lambda histogram: hist_name == histogram.name,
                    self._tracker.color_histograms,
                )
            )

    def update_params(self, params: dict) -> None:
        """Updates internally stored parameters of the node. Loops over all internal
        optimizers, updates their parameters and reinitialize the tracker.

        :param params: Dictionary with configuration of the tracker.
        :type params: dict
        :raises RuntimeError: Failed to update tracker parameters.
        """
        self._params = params
        self._tracker.n_corr_iterations = self._params["tracker"]["n_corr_iterations"]
        self._tracker.n_update_iterations = self._params["tracker"][
            "n_update_iterations"
        ]

        # Obtain list of pointers to optimizers with their class ids
        optimizers_with_class = [
            (optimizer, self._registered_optimizers[optimizer.name])
            for optimizer in self._tracker.optimizer
            if optimizer.name in self._registered_optimizers.keys()
        ]
        for optimizer, class_id in optimizers_with_class:
            (
                optimizer_class,
                region_modality_class,
                depth_modality_class,
                texture_modality_class,
            ) = self._match_class_types(class_id)

            # Decouple optimizer from the tracker
            self._tracker.DeleteOptimizer(optimizer.name)
            # Update optimizer config
            optimizer = self._update_object_config(
                optimizer, self._params[optimizer_class]["optimizer"]
            )

            # Iterate over possible modalities
            for idx, modality in enumerate(optimizer.root_link.modality):
                if "region_modality" in modality.name:
                    optimizer.root_link.modality[idx] = self._update_object_config(
                        optimizer.root_link.modality[idx],
                        self._params[region_modality_class]["region_modality"],
                    )
                if "depth_modality" in modality.name:
                    optimizer.root_link.modality[idx] = self._update_object_config(
                        optimizer.root_link.modality[idx],
                        self._params[depth_modality_class]["depth_modality"],
                    )
                if "texture_modality" in modality.name:
                    optimizer.root_link.modality[idx] = self._update_object_config(
                        optimizer.root_link.modality[idx],
                        self._params[texture_modality_class]["texture_modality"],
                    )
                    # Parameter ``descriptor_type`` is exposed as string and has to be
                    # casted to a matching integer
                    optimizer.root_link.modality[
                        idx
                    ].descriptor_type = self._match_texture_descriptor_type(
                        self._params[texture_modality_class]["texture_modality"][
                            "descriptor_type_name"
                        ]
                    )
                optimizer.root_link.modality[idx].SetUp()

            # Reinitialize the optimizer and reattach it to the tracker
            optimizer.SetUp()
            self._tracker.AddOptimizer(optimizer)

        # Update known configs
        for key in self._known_objects_configs.keys():
            hist = self._known_objects_configs[key].histogram
            self._known_objects_configs[key].histogram = self._reload_histogram_params(
                hist
            )

        # If any objects were update reinitialize the tracker
        if len(optimizers_with_class) > 0:
            if not self._tracker.SetUp(set_up_all_objects=self._first_setup):
                raise RuntimeError("Failed to reinitialize updated tracker!")

    def _image_data_to_intrinsics(
        self, camera_k: npt.NDArray[np.float64], im_shape: Tuple[int]
    ) -> pym3t.Intrinsics:
        """Converts matrix with camera intrinsics and its shape to pym3t.Intrinsics object.

        :param camera_k: Intrinsics matrix of the camera.
        :type camera_k: npt.NDArray[np.float64]
        :param im_shape: Dimensions of the image.
        :type im_shape: Tuple[int]
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
        """Clears values held in the ``self._object_type_counter`` dictionary."""
        self._object_type_counter = {id: 0 for id in self._object_cache.keys()}

    def _get_and_bump_type_counter(self, class_id: str) -> str:
        """Creates unique name for given object type composed of its ``class_id`` and
        the number of times it was registered inside of the tracker. After name generation
        counter of its occurrences is increased.

        :param class_id: Name of the class which.
        :type class_id: str
        :return: Unique name identifier.
        :rtype: str
        """
        name = f"{class_id}_{self._object_type_counter[class_id]}"
        self._object_type_counter[class_id] += 1
        return name

    def _match_class_types(self, class_id: str) -> Tuple[str]:
        """Checks if given object class is configured to use own parameters for region
        modality, depth modality, texture modality and optimizer params, or should load
        default, ``global`` configuration.

        :param class_id: _description_
        :type class_id: str
        :return: Tuple with names of classes which parameters should be later loaded.
            Order: optimizer class, region modality class, depth modality class,
            texture modality class.
        :rtype: Tuple[str]
        """
        class_params = self._params[class_id]
        optimizer_class = "global" if class_params["use_global_optimizer"] else class_id
        rm_class = "global" if class_params["use_global_region_modality"] else class_id
        dm_class = "global" if class_params["use_global_depth_modality"] else class_id
        tm_class = "global" if class_params["use_global_texture_modality"] else class_id
        return optimizer_class, rm_class, dm_class, tm_class

    def _assemble_object_optimizer(
        self, object_name: str, class_id: str
    ) -> pym3t.Optimizer:
        """Creates new optimizer and sets it up for a given class.

        :param object_name: Name to assign to all the internal objects of the optimizer.
        :type object_name: str
        :param class_id: Class of which parameters should be loaded to the optimizer.
        :type class_id: str
        :raises ValueError: Region modality expects measuring of the occlusions,
            but depth is not enabled.
        :raises ValueError: Texture modality expects measuring of the occlusions,
            but depth is not enabled.
        :return: Newly created optimizer object.
        :rtype: pym3t.Optimizer
        """
        (
            optimizer_class,
            region_modality_class,
            depth_modality_class,
            texture_modality_class,
        ) = self._match_class_types(class_id)

        object_files = self._classes_with_valid_paths[class_id]
        body = pym3t.Body(
            name=object_name,
            geometry_path=object_files["obj"].as_posix(),
            geometry_unit_in_meter=1.0,
            geometry_counterclockwise=True,
            geometry_enable_culling=True,
            geometry2body_pose=np.eye(4),
        )
        body.SetUp()
        link = pym3t.Link(object_name + "_link", body)

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
            self._params[region_modality_class]["region_modality"],
        )

        if self._params[region_modality_class]["region_modality"]["measure_occlusions"]:
            if self._params["use_depth"]:
                region_modality.MeasureOcclusions(self._dummy_depth_camera)
            else:
                raise ValueError(
                    f"Object '{object_name}' expects to measure occlusions for "
                    "'region_modality', but param 'use_depth' is set to 'false'!"
                )
        if self._params[region_modality_class]["region_modality"]["model_occlusion"]:
            self._focused_color_depth_renderer.AddReferencedBody(body)
            region_modality.ModelOcclusions(self._focused_color_depth_renderer)

        region_modality.SetUp()
        link.AddModality(region_modality)

        if (
            self._params["use_depth"]
            and self._params[depth_modality_class]["use_depth_modality"]
        ):
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
                self._params[depth_modality_class]["depth_modality"],
            )

            if self._params[depth_modality_class]["depth_modality"][
                "measure_occlusions"
            ]:
                depth_modality.MeasureOcclusions()

            if self._params[depth_modality_class]["depth_modality"]["model_occlusion"]:
                self._focused_depth_depth_renderer.AddReferencedBody(body)
                depth_modality.ModelOcclusions(self._focused_depth_depth_renderer)

            depth_modality.SetUp()
            link.AddModality(depth_modality)

        if self._params[texture_modality_class]["use_texture_modality"]:
            self._color_silhouette_renderer.AddReferencedBody(body)
            texture_modality = pym3t.TextureModality(
                object_name + "_texture_modality",
                body,
                self._dummy_color_camera,
                self._color_silhouette_renderer,
            )
            texture_modality = self._update_object_config(
                texture_modality,
                self._params[texture_modality_class]["texture_modality"],
            )
            # Parameter ``descriptor_type`` is exposed as string and has to be
            # casted to a matching integer
            texture_modality.descriptor_type = self._match_texture_descriptor_type(
                self._params[texture_modality_class]["texture_modality"][
                    "descriptor_type_name"
                ]
            )

            if self._params[texture_modality_class]["texture_modality"][
                "measure_occlusions"
            ]:
                if self._params["use_depth"]:
                    texture_modality.MeasureOcclusions(self._dummy_depth_camera)
                else:
                    raise ValueError(
                        f"Object '{object_name}' expects to measure occlusions for "
                        "'texture_modality', but param 'use_depth' is set to 'false'!"
                    )
            if self._params[texture_modality_class]["texture_modality"][
                "model_occlusion"
            ]:
                if not self._focused_color_depth_renderer.IsBodyReferenced(body.name):
                    self._focused_color_depth_renderer.AddReferencedBody(body)
                texture_modality.ModelOcclusions(self._focused_color_depth_renderer)

            link.AddModality(texture_modality)

        optimizer = pym3t.Optimizer(
            object_name + "_optimizer",
            link,
        )
        optimizer = self._update_object_config(
            optimizer, self._params[optimizer_class]["optimizer"]
        )

        return optimizer

    def _update_object_config(
        self,
        modality: Union[
            pym3t.RegionModality,
            pym3t.DepthModality,
            pym3t.TextureModality,
            pym3t.Optimizer,
        ],
        params_dict: dict,
    ) -> Union[
        pym3t.RegionModality,
        pym3t.DepthModality,
        pym3t.TextureModality,
        pym3t.Optimizer,
    ]:
        """Directly injects data from code-generated ROS parameters converted to
        a dictionary into modality or optimizer data structure configuration.

        :param modality: Region or Depth modality or Optimizer configuration structure.
        :type modality: Union[pym3t.RegionModality, pym3t.DepthModality,
            pym3t.TextureModality, pym3t.Optimizer]
        :param params_dict: Dictionary with ROS parameters extracted for a given modality
            type or optimizer configuration.
        :type params_dict: dict
        :return: Updated modality config.
        :rtype: Union[pym3t.RegionModality, pym3t.DepthModality,
            pym3t.TextureModality, pym3t.Optimizer]
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

    def _match_texture_descriptor_type(self, description_type: str) -> int:
        """Matches string names of descriptors with ones from ``m3t/texture_modality.h`` file.

        :param description_type: Name of the descriptor type of TextureModality class.
        :type description_type: str
        :return: Integer representation of the descriptor type.
        :rtype: int
        """
        return {"BRISK": 0, "DAISY": 1, "FREAK": 2, "SIFT": 3, "ORB": 4, "ORB_CUDA": 5}[
            description_type
        ]

    def _reload_histogram_params(
        self, obj_config: KnownObjectConfig, hist_name: Union[None, str] = None
    ) -> pym3t.ColorHistograms:
        """Return pym3t.ColorHistograms with currently up to date configuration.
        If passed histogram didn't change, the same histogram is returned. If parameters
        changed or ``None`` was set to histogram field of ``obj_config``, new histogram
        is created and set up.

        :param obj_config: Config of known objects.
        :type obj_config: KnownObjectConfig
        :param hist_name: If None is passed on ``obj_config.histogram`` this is the
            name of the new histogram, defaults to None.
        :type hist_name: Union[None, str], optional
        :return: Correctly configured histogram.
        :rtype: pym3t.ColorHistograms
        """
        class_id = (
            "global"
            if self._params[obj_config.class_id]["use_global_region_modality"]
            else obj_config.class_id
        )
        # Get region modality params
        rmp = self._params[class_id]["region_modality"]
        # If old histogram was passed and parameters didn't change
        if (
            obj_config.histogram is None
            or obj_config.histogram.n_bins != rmp["n_bins"]
            or obj_config.histogram.learning_rate_f != rmp["learning_rate_f"]
            or obj_config.histogram.learning_rate_b != rmp["learning_rate_b"]
        ):
            new_hist = pym3t.ColorHistograms(
                name=(
                    obj_config.histogram.name
                    if obj_config.histogram is not None
                    else hist_name
                ),
                n_bins=rmp["n_bins"],
                learning_rate_f=rmp["learning_rate_f"],
                learning_rate_b=rmp["learning_rate_b"],
            )
            new_hist.SetUp()
            return new_hist
        return obj_config.histogram
