from dataclasses import dataclass
import math
import numpy as np
import numpy.typing as npt
import pathlib
import re
from typing import List, Tuple, Union

import pym3t


@dataclass
class TrackedObject:
    """Structure holding information about currently tracked object.
    Used to pass objects to the ``SpecializedTracker`` class."""

    # Unique identifier of an object. Not not known set to empty string
    id: str
    # Class id of tracked object
    class_id: str
    # Pose relative to camera frame
    body2camera_pose: npt.NDArray[np.float64]


class SpecializedTracker:
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

        self._prefix, self._postfix = (
            self._params["filename_format"]
            .removesuffix(".${file_fmt}")
            .split("${class_id}")
        )
        self._pattern = re.compile(self._params["class_id_regex"])

        def _get_inner_class_id(outer_class_id: "str") -> str:
            matches = self._pattern.findall(outer_class_id)
            if len(matches) == 0:
                raise RuntimeError(
                    f"Failed to find match with '{self._params['class_id_regex']}' "
                    f"in the class id string '{outer_class_id}'!"
                )
            raw_class_id = matches[0]
            return self._prefix + raw_class_id + self._postfix

        # Create a map to convert received class id to the their inner names
        self._outer_to_inner_classes = {
            obj: _get_inner_class_id(obj) for obj in self._params["tracked_objects"]
        }
        # Invert the dependency for fast reconstruction
        self._inner_to_outer_classes = {
            v: k for k, v in self._outer_to_inner_classes.items()
        }

        inner_class_names = list(self._inner_to_outer_classes.keys())

        # Check if list of the objects is not empty
        if not len(inner_class_names):
            raise ValueError("No objects passed to track!")

        self._valid_class_ids = self._check_dataset_path(
            pathlib.Path(self._params["dataset_path"]),
            inner_class_names,
            self._params["use_depth"],
        )

        # If not all objects were possible to be loaded
        if len(self._valid_class_ids) - len(inner_class_names):
            diff = set(inner_class_names) - set(self._valid_class_ids.keys())
            raise RuntimeError(f"Filed to load models for objects: {diff}!")

        # Tracker OpenGL interface
        self._renderer_geometry = pym3t.RendererGeometry("renderer_geometry")

        # Initialize M3T tracker
        self._dummy_color_camera = pym3t.DummyColorCamera("dummy_color_camera")
        self._dummy_color_camera.camera2world_pose = np.eye(4)

        if self._params["use_depth"]:
            self._dummy_depth_camera = pym3t.DummyDepthCamera("dummy_depth_camera")
            self._focused_depth_depth_renderer = pym3t.FocusedBasicDepthRenderer(
                f"focused_depth_depth_renderer_{self._dummy_depth_camera.name}",
                self._renderer_geometry,
                self._dummy_depth_camera,
            )
        self._focused_color_depth_renderer = pym3t.FocusedBasicDepthRenderer(
            f"focused_color_depth_renderer_{self._dummy_color_camera.name}",
            self._renderer_geometry,
            self._dummy_color_camera,
        )
        self._color_silhouette_renderer = pym3t.FocusedSilhouetteRenderer(
            "color_silhouette_renderer",
            self._renderer_geometry,
            self._dummy_color_camera,
        )

        # Cashed objects
        # Stores currently tracked object references
        self._last_objects_order = List[TrackedObject]
        # Dict[str, str]
        self._known_objects = {}

        self._counter = 0
        self._execute_starting_step = True
        self._tracker = pym3t.Tracker("tracker", synchronize_cameras=False)
        self._tracker.n_corr_iterations = self._params["tracker"]["n_corr_iterations"]
        self._tracker.n_update_iterations = self._params["tracker"][
            "n_update_iterations"
        ]

        self._disable_pose = np.eye(4)
        self._disable_pose[3, 2] = -2.0

        optimizers = {
            class_id: [
                self._assemble_object_optimizer(class_id, f"_{i}")
                for i in range(
                    self._params[self._inner_to_outer_classes[class_id]][
                        "max_instances"
                    ]
                )
            ]
            for class_id in self._valid_class_ids
        }

        for optimizer_type_list in optimizers.values():
            for optimizer in optimizer_type_list:
                self._tracker.AddOptimizer(optimizer)

        if not self._tracker.SetUp():
            raise RuntimeError("Failed to initialize the tracker!")

        # Convert pointers to loaded optimizers into their labels for future matching
        self._preloaded_optimizers = {
            class_id: [optimizer.name for optimizer in optimizer_type_list]
            for class_id, optimizer_type_list in optimizers.items()
        }

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
            raise RuntimeError("Nothing to track!")

        self._dummy_color_camera.image = color_image

        # Updating intrinsics requires to set up the camera
        new_k = self._image_data_to_intrinsics(color_camera_k, color_image.shape)
        old_k = self._dummy_color_camera.intrinsics
        reset_tracker = False
        if old_k is None or not self._intrinsics_equal(new_k, old_k):
            self._dummy_color_camera.intrinsics = new_k
            reset_tracker = True

        if (
            depth_image is not None
            and depth_camera_k is not None
            and depth2color_pose is not None
        ):
            self._dummy_depth_camera.image = depth_image

            new_k = self._image_data_to_intrinsics(depth_camera_k, depth_image.shape)
            old_k = self._dummy_depth_camera.intrinsics
            if old_k is None or not self._intrinsics_equal(new_k, old_k):
                self._dummy_depth_camera.intrinsics = new_k
                reset_tracker = True

            old_pose = self._dummy_depth_camera.camera2world_pose
            if old_pose is None or not np.isclose(old_pose, depth2color_pose).any():
                self._dummy_depth_camera.camera2world_pose = depth2color_pose
                reset_tracker = True

        if reset_tracker:
            if not self._tracker.SetUp():
                raise RuntimeError("Failed to initialize the tracker!")

        # 0 is just a dummy value, as it is not used in the C++ code
        if not self._tracker.UpdateCameras(0):
            raise RuntimeError("Received images are invalid!")

        if self._execute_starting_step:
            self._tracker.ExecuteStartingStep(0)
            self._execute_starting_step = False

        self._tracker.ExecuteTrackingStep(self._counter)
        self._counter += 1

        # Create a map of optimizer name to held body pose
        optimizers_poses = {
            optimizer.name: optimizer.root_link.body.body2world_pose
            for optimizer in self._tracker.optimizers
        }

        # Update body poses
        for i, track in enumerate(self._last_objects_order):
            self._last_objects_order[i].body2camera_pose = optimizers_poses[
                self._known_objects[track.id]
            ]

        return self._last_objects_order

    def update_tracked_objects(self, objects: List[TrackedObject]) -> None:
        """Updates internal
        list of tracked objects. Creates new optimizer if given object
        type is missing one. Removes optimizers if not sued for long time and manages
        history of histograms for known objects.

        :param objects: List of the objects to store for tracking.
        :type objects: List[TrackedObject]
        :raises RuntimeError: Class id of known object changed.
        :raises RuntimeError: Reinitialization of the tracker failed.
        """
        self._last_objects_order = objects

        # If nothing to track skip the rest
        if len(self._last_objects_order) == 0:
            return

        optimizers_map = {
            optimizer.name: optimizer for optimizer in self._tracker.optimizers
        }
        for optimizer in optimizers_map.values():
            optimizer.root_link.body.body2world_pose = self._disable_pose

        for track in self._last_objects_order:
            # Convert class id format
            class_id = self._outer_to_inner_classes[track.class_id]
            # The track was not previously known
            if track.id not in self._known_objects:
                # If not new tracks can be assigned for a given object, ignore it
                if class_id not in self._preloaded_optimizers:
                    raise RuntimeError(f"Unknown class id: '{class_id}'")
                if len(self._preloaded_optimizers[class_id]) == 0:
                    raise RuntimeError(f"No more optimizers for class id: '{class_id}'")
                # Pick first available optimizer and remove it from the list
                self._known_objects[track.id] = self._preloaded_optimizers[
                    class_id
                ].pop(0)

            optimizer = optimizers_map[self._known_objects[track.id]]
            optimizer.root_link.body.body2world_pose = track.body2camera_pose

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

        def _update_optimizer(optimizer: pym3t.Optimizer) -> None:
            # Update optimizer config
            optimizer = self._update_object_config(optimizer, self._params["optimizer"])

            modality_types = ["region_modality", "depth_modality", "texture_modality"]

            # Iterate over possible modalities
            for idx, modality in enumerate(optimizer.root_link.modality):
                for modality_type in modality_types:
                    if modality_type in modality.name:
                        mod_params = self._params[modality_type]
                        optimizer.root_link.modality[idx] = self._update_object_config(
                            optimizer.root_link.modality[idx],
                            mod_params,
                        )
                        if modality_type == "texture_modality":
                            # Parameter ``descriptor_type`` is exposed as string
                            # and has to be TextureModality::DescriptorType enum
                            optimizer.root_link.modality[idx].descriptor_type = getattr(
                                pym3t.DescriptorType,
                                self._params["texture_modality"][
                                    "descriptor_type_name"
                                ],
                            )
                        continue

        # Loop over keys and indexes to explicitly modify elements of dictionary and lists
        for class_id in self._preloaded_optimizers.keys():
            for i in len(self._preloaded_optimizers[class_id]):
                optimizer = self._preloaded_optimizers[class_id][i]
                self._preloaded_optimizers[class_id][i] = _update_optimizer(optimizer)

        # Update optimizers being a part of the tracker
        for optimizer in self._tracker.optimizers:
            # Decouple optimizer from the tracker
            self._tracker.DeleteOptimizer(optimizer.name)
            # Update its params
            optimizer = _update_optimizer(optimizer)
            # Reattach it
            self._tracker.AddOptimizer(optimizer)

        if not self._tracker.SetUp():
            raise RuntimeError("Failed to initialize the tracker!")

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
            width=im_shape[1],
            height=im_shape[0],
        )

    def _intrinsics_equal(self, k_1: pym3t.Intrinsics, k_2: pym3t.Intrinsics) -> bool:
        return (
            math.isclose(k_1.fu, k_2.fu)
            and math.isclose(k_1.fv, k_2.fv)
            and math.isclose(k_1.ppu, k_2.ppu)
            and math.isclose(k_1.ppv, k_2.ppv)
            and k_1.width == k_2.width
            and k_1.height == k_2.height
        )

    def _assemble_object_optimizer(
        self, class_id: str, postfix: str
    ) -> pym3t.Optimizer:
        """Creates new optimizer and sets it up for a given class.

        :param class_id: Class of which parameters should be loaded to the optimizer.
        :type class_id: str
        :param postfix: Postfix to append to ``class_id`` when creating names for the
            underlying objects.
        :type postfix: str
        :raises ValueError: Region modality expects measuring of the occlusions,
            but depth is not enabled.
        :raises ValueError: Texture modality expects measuring of the occlusions,
            but depth is not enabled.
        :return: Newly created optimizer object.
        :rtype: pym3t.Optimizer
        """
        object_name = class_id + postfix

        object_files = self._valid_class_ids[class_id]
        body = pym3t.Body(
            name=object_name,
            geometry_path=object_files["obj"].as_posix(),
            geometry_unit_in_meter=1.0,
            geometry_counterclockwise=True,
            geometry_enable_culling=True,
            geometry2body_pose=np.eye(4),
        )

        self._renderer_geometry.AddBody(body)
        link = pym3t.Link(object_name + "_link", body)

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
            region_modality,
            self._params["region_modality"],
        )

        if self._params["region_modality"]["measure_occlusions"]:
            if self._params["use_depth"]:
                region_modality.MeasureOcclusions(self._dummy_depth_camera)
            else:
                raise ValueError(
                    f"Object '{object_name}' expects to measure occlusions for "
                    "'region_modality', but param 'use_depth' is set to 'false'!"
                )
        if self._params["region_modality"]["model_occlusion"]:
            self._focused_color_depth_renderer.AddReferencedBody(body)
            region_modality.ModelOcclusions(self._focused_color_depth_renderer)

        link.AddModality(region_modality)

        if self._params["use_depth"] and self._params["use_depth_modality"]:
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
                depth_modality,
                self._params["depth_modality"],
            )

            if self._params["depth_modality"]["measure_occlusions"]:
                depth_modality.MeasureOcclusions()

            if self._params["depth_modality"]["model_occlusion"]:
                self._focused_depth_depth_renderer.AddReferencedBody(body)
                depth_modality.ModelOcclusions(self._focused_depth_depth_renderer)

            link.AddModality(depth_modality)

        if self._params["use_texture_modality"]:
            self._color_silhouette_renderer.AddReferencedBody(body)
            texture_modality = pym3t.TextureModality(
                object_name + "_texture_modality",
                body,
                self._dummy_color_camera,
                self._color_silhouette_renderer,
            )
            texture_modality = self._update_object_config(
                texture_modality,
                self._params["texture_modality"],
            )
            # Parameter ``descriptor_type`` is exposed as string
            # and has to be TextureModality::DescriptorType enum
            texture_modality.descriptor_type = getattr(
                pym3t.DescriptorType,
                self._params["texture_modality"]["descriptor_type_name"],
            )

            if self._params["texture_modality"]["measure_occlusions"]:
                if self._params["use_depth"]:
                    texture_modality.MeasureOcclusions(self._dummy_depth_camera)
                else:
                    raise ValueError(
                        f"Object '{object_name}' expects to measure occlusions for "
                        "'texture_modality', but param 'use_depth' is set to 'false'!"
                    )
            if self._params["texture_modality"]["model_occlusion"]:
                if not self._focused_color_depth_renderer.IsBodyReferenced(body.name):
                    self._focused_color_depth_renderer.AddReferencedBody(body)
                texture_modality.ModelOcclusions(self._focused_color_depth_renderer)

            link.AddModality(texture_modality)

        optimizer = pym3t.Optimizer(
            object_name + "_optimizer",
            link,
        )
        optimizer = self._update_object_config(optimizer, self._params["optimizer"])

        return optimizer

    def _update_object_config(
        self,
        class_object: Union[
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
            if hasattr(class_object, key):
                # Set new parameter value
                setattr(class_object, key, val)

        return class_object

    def _check_dataset_path(
        self, dataset_path: pathlib.Path, tracked_objects: List[str], use_depth: bool
    ):  # -> Dict[str : Tuple[pathlib.Path]]:
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
        files_in_dir = [x for x in dataset_path.glob("*") if x.is_file()]
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
