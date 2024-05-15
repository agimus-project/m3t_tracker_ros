# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
from generate_parameter_library_py.python_validators import ParameterValidators

import m3t_tracker_ros.custom_validation as custom_validators


class m3t_tracker_ros:
    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        dataset_path = None
        dataset_name = None
        use_depth = False
        depth_scale = 0.001
        geometry_unit_in_meter = 0.001
        n_corr_iterations = 4
        n_update_iterations = 2
        tikhonov_parameter_rotation = 1000.0
        tikhonov_parameter_translation = 30000.0
        models = ["region_model", "depth_model"]
        tracked_objects = [""]

        class __MapModels:
            sphere_radius = 0.8
            n_divides = 4
            n_points_max = 200
            max_radius_depth_offset = 0.05
            stride_depth_offset = 0.002
            use_random_seed = False
            image_size = 2000

        __map_type = __MapModels

        def add_entry(self, name):
            if not hasattr(self, name):
                setattr(self, name, self.__map_type())
            return getattr(self, name)

        def get_entry(self, name):
            return getattr(self, name)

        class __MapTrackedObjects:
            class __RegionModality:
                n_lines_max = 200
                use_adaptive_coverage = False
                max_radius_depth_offset = 0.0
                min_continuous_distance = 3.0
                function_length = 8
                distribution_length = 12
                function_amplitude = 0.43
                function_slope = 0.5
                learning_rate = 1.3
                n_global_iterations = 1
                scales = [6.0, 4.0, 2.0, 1.0]
                standard_deviations = [15.0, 5.0, 3.5, 1.5]

                class __Histogram:
                    n_bins = 16
                    learning_rate_f = 0.2
                    learning_rate_b = 0.2
                    unconsidered_line_length = 0.5
                    max_considered_line_length = 20.0

                histogram = __Histogram()

                class __Occlusion:
                    n_unoccluded_iterations = 10
                    min_n_unoccluded_lines = 0
                    measure_occlusions = False
                    measured_depth_offset_radius = 0.01
                    measured_occlusion_radius = 0.01
                    measured_occlusion_threshold = 0.03
                    model_occlusions = False
                    modeled_depth_offset_radius = 0.01
                    modeled_occlusion_radius = 0.01
                    modeled_occlusion_threshold = 0.03

                occlusion = __Occlusion()

            region_modality = __RegionModality()

            class __DepthModality:
                n_points_max = 200
                use_adaptive_coverage = False
                reference_surface_area = 0.0
                stride_length = 0.005
                considered_distances = [0.05, 0.02, 0.01]
                standard_deviations = [0.05, 0.03, 0.02]

                class __Occlusion:
                    n_unoccluded_iterations = 10
                    min_n_unoccluded_points = 0
                    measure_occlusions = False
                    measured_depth_offset_radius = 0.01
                    measured_occlusion_radius = 0.01
                    measured_occlusion_threshold = 0.03
                    model_occlusions = False
                    modeled_depth_offset_radius = 0.01
                    modeled_occlusion_radius = 0.01
                    modeled_occlusion_threshold = 0.03

                occlusion = __Occlusion()

            depth_modality = __DepthModality()

        __map_type = __MapTrackedObjects

        def add_entry(self, name):
            if not hasattr(self, name):
                setattr(self, name, self.__map_type())
            return getattr(self, name)

        def get_entry(self, name):
            return getattr(self, name)

    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = m3t_tracker_ros.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("m3t_tracker_ros." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.sphere_radius"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0.0
                    descriptor.floating_point_range[-1].to_value = float("inf")
                    parameter = entry.sphere_radius
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt(param, 0.0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.sphere_radius",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.sphere_radius: "
                        + validation_result,
                    )
                entry.sphere_radius = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.n_divides"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.integer_range.append(IntegerRange())
                    descriptor.integer_range[-1].from_value = 1
                    descriptor.integer_range[-1].to_value = 2**31 - 1
                    parameter = entry.n_divides
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt_eq(param, 1)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.n_divides",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.n_divides: "
                        + validation_result,
                    )
                entry.n_divides = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.n_points_max"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.integer_range.append(IntegerRange())
                    descriptor.integer_range[-1].from_value = 1
                    descriptor.integer_range[-1].to_value = 2**31 - 1
                    parameter = entry.n_points_max
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt_eq(param, 1)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.n_points_max",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.n_points_max: "
                        + validation_result,
                    )
                entry.n_points_max = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.max_radius_depth_offset"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0.0
                    descriptor.floating_point_range[-1].to_value = float("inf")
                    parameter = entry.max_radius_depth_offset
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt(param, 0.0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.max_radius_depth_offset",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.max_radius_depth_offset: "
                        + validation_result,
                    )
                entry.max_radius_depth_offset = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.stride_depth_offset"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0.0
                    descriptor.floating_point_range[-1].to_value = float("inf")
                    parameter = entry.stride_depth_offset
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt(param, 0.0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.stride_depth_offset",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.stride_depth_offset: "
                        + validation_result,
                    )
                entry.stride_depth_offset = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.use_random_seed"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    parameter = entry.use_random_seed
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                entry.use_random_seed = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.image_size"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.integer_range.append(IntegerRange())
                    descriptor.integer_range[-1].from_value = 1
                    descriptor.integer_range[-1].to_value = 2**31 - 1
                    parameter = entry.image_size
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt_eq(param, 1)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.image_size",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.image_size: "
                        + validation_result,
                    )
                entry.image_size = param.value

        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "dataset_path":
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.dataset_path = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "dataset_name":
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.dataset_name = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "use_depth":
                    updated_params.use_depth = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "depth_scale":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.depth_scale = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "geometry_unit_in_meter":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.geometry_unit_in_meter = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "n_corr_iterations":
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.n_corr_iterations = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "n_update_iterations":
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.n_update_iterations = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "tikhonov_parameter_rotation":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.tikhonov_parameter_rotation = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "tikhonov_parameter_translation":
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.tikhonov_parameter_translation = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "models":
                    validation_result = ParameterValidators.fixed_size(param, 2)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    validation_result = ParameterValidators.subset_of(
                        param, "region_model", "depth_model"
                    )
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    validation_result = ParameterValidators.unique(param)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.models = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "tracked_objects":
                    updated_params.tracked_objects = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.n_lines_max"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.n_lines_max = (
                        param.value
                    )
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.use_adaptive_coverage"
                ):
                    updated_params.__map_tracked_objects.region_modality.use_adaptive_coverage = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.max_radius_depth_offset"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.max_radius_depth_offset = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.min_continuous_distance"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.min_continuous_distance = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.function_length"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.function_length = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.distribution_length"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.distribution_length = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.function_amplitude"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.function_amplitude = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.function_slope"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.function_slope = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.learning_rate"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.learning_rate = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.n_global_iterations"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.n_global_iterations = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_ + "__map_tracked_objects.region_modality.scales"
                ):
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.scales = (
                        param.value
                    )
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.standard_deviations"
                ):
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.standard_deviations = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.histogram.n_bins"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.histogram.n_bins = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.histogram.learning_rate_f"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.histogram.learning_rate_f = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.histogram.learning_rate_b"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.histogram.learning_rate_b = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.histogram.unconsidered_line_length"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.histogram.unconsidered_line_length = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.histogram.max_considered_line_length"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.histogram.max_considered_line_length = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.n_unoccluded_iterations"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.occlusion.n_unoccluded_iterations = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.min_n_unoccluded_lines"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.occlusion.min_n_unoccluded_lines = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.measure_occlusions"
                ):
                    updated_params.__map_tracked_objects.region_modality.occlusion.measure_occlusions = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.measured_depth_offset_radius"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.occlusion.measured_depth_offset_radius = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.measured_occlusion_radius"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.occlusion.measured_occlusion_radius = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.measured_occlusion_threshold"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.occlusion.measured_occlusion_threshold = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.model_occlusions"
                ):
                    updated_params.__map_tracked_objects.region_modality.occlusion.model_occlusions = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.modeled_depth_offset_radius"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.occlusion.modeled_depth_offset_radius = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.modeled_occlusion_radius"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.occlusion.modeled_occlusion_radius = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.modeled_occlusion_threshold"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.region_modality.occlusion.modeled_occlusion_threshold = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.n_points_max"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.n_points_max = (
                        param.value
                    )
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.use_adaptive_coverage"
                ):
                    updated_params.__map_tracked_objects.depth_modality.use_adaptive_coverage = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.reference_surface_area"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.reference_surface_area = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.stride_length"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.stride_length = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.considered_distances"
                ):
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.considered_distances = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.standard_deviations"
                ):
                    validation_result = ParameterValidators.not_empty(param)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.standard_deviations = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.n_unoccluded_iterations"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 1)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.occlusion.n_unoccluded_iterations = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.min_n_unoccluded_points"
                ):
                    validation_result = ParameterValidators.gt_eq(param, 0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.occlusion.min_n_unoccluded_points = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.measure_occlusions"
                ):
                    updated_params.__map_tracked_objects.depth_modality.occlusion.measure_occlusions = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.measured_depth_offset_radius"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.occlusion.measured_depth_offset_radius = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.measured_occlusion_radius"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.occlusion.measured_occlusion_radius = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.measured_occlusion_threshold"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.occlusion.measured_occlusion_threshold = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.model_occlusions"
                ):
                    updated_params.__map_tracked_objects.depth_modality.occlusion.model_occlusions = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.modeled_depth_offset_radius"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.occlusion.modeled_depth_offset_radius = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_radius"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_radius = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_threshold"
                ):
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        return SetParametersResult(
                            successful=False, reason=validation_result
                        )
                    updated_params.__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_threshold = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

            # update dynamic parameters
            for param in parameters:
                for value_1 in updated_params.models:
                    param_name = f"{self.prefix_}{value_1}.sphere_radius"
                    if param.name == param_name:
                        validation_result = ParameterValidators.gt(param, 0.0)
                        if validation_result:
                            return SetParametersResult(
                                successful=False, reason=validation_result
                            )

                        updated_params.get_entry(value_1).sphere_radius = param.value
                        self.logger_.debug(
                            param.name
                            + ": "
                            + param.type_.name
                            + " = "
                            + str(param.value)
                        )

                for value_1 in updated_params.models:
                    param_name = f"{self.prefix_}{value_1}.n_divides"
                    if param.name == param_name:
                        validation_result = ParameterValidators.gt_eq(param, 1)
                        if validation_result:
                            return SetParametersResult(
                                successful=False, reason=validation_result
                            )

                        updated_params.get_entry(value_1).n_divides = param.value
                        self.logger_.debug(
                            param.name
                            + ": "
                            + param.type_.name
                            + " = "
                            + str(param.value)
                        )

                for value_1 in updated_params.models:
                    param_name = f"{self.prefix_}{value_1}.n_points_max"
                    if param.name == param_name:
                        validation_result = ParameterValidators.gt_eq(param, 1)
                        if validation_result:
                            return SetParametersResult(
                                successful=False, reason=validation_result
                            )

                        updated_params.get_entry(value_1).n_points_max = param.value
                        self.logger_.debug(
                            param.name
                            + ": "
                            + param.type_.name
                            + " = "
                            + str(param.value)
                        )

                for value_1 in updated_params.models:
                    param_name = f"{self.prefix_}{value_1}.max_radius_depth_offset"
                    if param.name == param_name:
                        validation_result = ParameterValidators.gt(param, 0.0)
                        if validation_result:
                            return SetParametersResult(
                                successful=False, reason=validation_result
                            )

                        updated_params.get_entry(
                            value_1
                        ).max_radius_depth_offset = param.value
                        self.logger_.debug(
                            param.name
                            + ": "
                            + param.type_.name
                            + " = "
                            + str(param.value)
                        )

                for value_1 in updated_params.models:
                    param_name = f"{self.prefix_}{value_1}.stride_depth_offset"
                    if param.name == param_name:
                        validation_result = ParameterValidators.gt(param, 0.0)
                        if validation_result:
                            return SetParametersResult(
                                successful=False, reason=validation_result
                            )

                        updated_params.get_entry(
                            value_1
                        ).stride_depth_offset = param.value
                        self.logger_.debug(
                            param.name
                            + ": "
                            + param.type_.name
                            + " = "
                            + str(param.value)
                        )

                for value_1 in updated_params.models:
                    param_name = f"{self.prefix_}{value_1}.use_random_seed"
                    if param.name == param_name:
                        updated_params.get_entry(value_1).use_random_seed = param.value
                        self.logger_.debug(
                            param.name
                            + ": "
                            + param.type_.name
                            + " = "
                            + str(param.value)
                        )

                for value_1 in updated_params.models:
                    param_name = f"{self.prefix_}{value_1}.image_size"
                    if param.name == param_name:
                        validation_result = ParameterValidators.gt_eq(param, 1)
                        if validation_result:
                            return SetParametersResult(
                                successful=False, reason=validation_result
                            )

                        updated_params.get_entry(value_1).image_size = param.value
                        self.logger_.debug(
                            param.name
                            + ": "
                            + param.type_.name
                            + " = "
                            + str(param.value)
                        )

            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "dataset_path"):
                descriptor = ParameterDescriptor(
                    description="Global path to the precomputed dataset of object modalities.",
                    read_only=False,
                )
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(
                    self.prefix_ + "dataset_path", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "dataset_name"):
                descriptor = ParameterDescriptor(
                    description="Name of the dataset to validate corespondance with vision_info messages.",
                    read_only=False,
                )
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(
                    self.prefix_ + "dataset_name", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "use_depth"):
                descriptor = ParameterDescriptor(
                    description="Whether to expect depth images to be published.",
                    read_only=False,
                )
                parameter = updated_params.use_depth
                self.node_.declare_parameter(
                    self.prefix_ + "use_depth", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "depth_scale"):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.depth_scale
                self.node_.declare_parameter(
                    self.prefix_ + "depth_scale", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "geometry_unit_in_meter"):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.geometry_unit_in_meter
                self.node_.declare_parameter(
                    self.prefix_ + "geometry_unit_in_meter", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "n_corr_iterations"):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.n_corr_iterations
                self.node_.declare_parameter(
                    self.prefix_ + "n_corr_iterations", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "n_update_iterations"):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.n_update_iterations
                self.node_.declare_parameter(
                    self.prefix_ + "n_update_iterations", parameter, descriptor
                )

            if not self.node_.has_parameter(
                self.prefix_ + "tikhonov_parameter_rotation"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.tikhonov_parameter_rotation
                self.node_.declare_parameter(
                    self.prefix_ + "tikhonov_parameter_rotation", parameter, descriptor
                )

            if not self.node_.has_parameter(
                self.prefix_ + "tikhonov_parameter_translation"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.tikhonov_parameter_translation
                self.node_.declare_parameter(
                    self.prefix_ + "tikhonov_parameter_translation",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(self.prefix_ + "models"):
                descriptor = ParameterDescriptor(
                    description="Walkaround to create two namespaces with the same set of parameters.",
                    read_only=True,
                )
                parameter = updated_params.models
                self.node_.declare_parameter(
                    self.prefix_ + "models", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "tracked_objects"):
                descriptor = ParameterDescriptor(
                    description="IDs of the objects found in the precomputed modalities folder used to track them.",
                    read_only=True,
                )
                parameter = updated_params.tracked_objects
                self.node_.declare_parameter(
                    self.prefix_ + "tracked_objects", parameter, descriptor
                )

            if not self.node_.has_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.n_lines_max"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = (
                    updated_params.__map_tracked_objects.region_modality.n_lines_max
                )
                self.node_.declare_parameter(
                    self.prefix_ + "__map_tracked_objects.region_modality.n_lines_max",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.use_adaptive_coverage"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                parameter = updated_params.__map_tracked_objects.region_modality.use_adaptive_coverage
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.use_adaptive_coverage",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.max_radius_depth_offset"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.max_radius_depth_offset
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.max_radius_depth_offset",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.min_continuous_distance"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.min_continuous_distance
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.min_continuous_distance",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.function_length"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = (
                    updated_params.__map_tracked_objects.region_modality.function_length
                )
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.function_length",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.distribution_length"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.__map_tracked_objects.region_modality.distribution_length
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.distribution_length",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.function_amplitude"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.function_amplitude
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.function_amplitude",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.function_slope"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = (
                    updated_params.__map_tracked_objects.region_modality.function_slope
                )
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.function_slope",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.learning_rate"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = (
                    updated_params.__map_tracked_objects.region_modality.learning_rate
                )
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.learning_rate",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.n_global_iterations"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.__map_tracked_objects.region_modality.n_global_iterations
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.n_global_iterations",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.scales"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                parameter = updated_params.__map_tracked_objects.region_modality.scales
                self.node_.declare_parameter(
                    self.prefix_ + "__map_tracked_objects.region_modality.scales",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.standard_deviations"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                parameter = updated_params.__map_tracked_objects.region_modality.standard_deviations
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.standard_deviations",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.histogram.n_bins"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.__map_tracked_objects.region_modality.histogram.n_bins
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.histogram.n_bins",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.histogram.learning_rate_f"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.histogram.learning_rate_f
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.histogram.learning_rate_f",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.histogram.learning_rate_b"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.histogram.learning_rate_b
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.histogram.learning_rate_b",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.histogram.unconsidered_line_length"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.histogram.unconsidered_line_length
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.histogram.unconsidered_line_length",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.histogram.max_considered_line_length"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.histogram.max_considered_line_length
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.histogram.max_considered_line_length",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.n_unoccluded_iterations"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.__map_tracked_objects.region_modality.occlusion.n_unoccluded_iterations
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.n_unoccluded_iterations",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.min_n_unoccluded_lines"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.__map_tracked_objects.region_modality.occlusion.min_n_unoccluded_lines
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.min_n_unoccluded_lines",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.measure_occlusions"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                parameter = updated_params.__map_tracked_objects.region_modality.occlusion.measure_occlusions
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.measure_occlusions",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.measured_depth_offset_radius"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.occlusion.measured_depth_offset_radius
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.measured_depth_offset_radius",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.measured_occlusion_radius"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.occlusion.measured_occlusion_radius
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.measured_occlusion_radius",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.measured_occlusion_threshold"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.occlusion.measured_occlusion_threshold
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.measured_occlusion_threshold",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.model_occlusions"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                parameter = updated_params.__map_tracked_objects.region_modality.occlusion.model_occlusions
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.model_occlusions",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.modeled_depth_offset_radius"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.occlusion.modeled_depth_offset_radius
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.modeled_depth_offset_radius",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.modeled_occlusion_radius"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.occlusion.modeled_occlusion_radius
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.modeled_occlusion_radius",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.modeled_occlusion_threshold"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.region_modality.occlusion.modeled_occlusion_threshold
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.region_modality.occlusion.modeled_occlusion_threshold",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "__map_tracked_objects.depth_modality.n_points_max"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = (
                    updated_params.__map_tracked_objects.depth_modality.n_points_max
                )
                self.node_.declare_parameter(
                    self.prefix_ + "__map_tracked_objects.depth_modality.n_points_max",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.use_adaptive_coverage"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                parameter = updated_params.__map_tracked_objects.depth_modality.use_adaptive_coverage
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.use_adaptive_coverage",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.reference_surface_area"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.depth_modality.reference_surface_area
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.reference_surface_area",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "__map_tracked_objects.depth_modality.stride_length"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = (
                    updated_params.__map_tracked_objects.depth_modality.stride_length
                )
                self.node_.declare_parameter(
                    self.prefix_ + "__map_tracked_objects.depth_modality.stride_length",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.considered_distances"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                parameter = updated_params.__map_tracked_objects.depth_modality.considered_distances
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.considered_distances",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.standard_deviations"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                parameter = updated_params.__map_tracked_objects.depth_modality.standard_deviations
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.standard_deviations",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.n_unoccluded_iterations"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.__map_tracked_objects.depth_modality.occlusion.n_unoccluded_iterations
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.n_unoccluded_iterations",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.min_n_unoccluded_points"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 0
                descriptor.integer_range[-1].to_value = 2**31 - 1
                parameter = updated_params.__map_tracked_objects.depth_modality.occlusion.min_n_unoccluded_points
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.min_n_unoccluded_points",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.measure_occlusions"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                parameter = updated_params.__map_tracked_objects.depth_modality.occlusion.measure_occlusions
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.measure_occlusions",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.measured_depth_offset_radius"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.depth_modality.occlusion.measured_depth_offset_radius
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.measured_depth_offset_radius",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.measured_occlusion_radius"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.depth_modality.occlusion.measured_occlusion_radius
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.measured_occlusion_radius",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.measured_occlusion_threshold"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.depth_modality.occlusion.measured_occlusion_threshold
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.measured_occlusion_threshold",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.model_occlusions"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                parameter = updated_params.__map_tracked_objects.depth_modality.occlusion.model_occlusions
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.model_occlusions",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.modeled_depth_offset_radius"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.depth_modality.occlusion.modeled_depth_offset_radius
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.modeled_depth_offset_radius",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_radius"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_radius
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_radius",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_threshold"
            ):
                descriptor = ParameterDescriptor(
                    description="@mfoumy plz help.", read_only=False
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0
                descriptor.floating_point_range[-1].to_value = float("inf")
                parameter = updated_params.__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_threshold
                self.node_.declare_parameter(
                    self.prefix_
                    + "__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_threshold",
                    parameter,
                    descriptor,
                )

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "dataset_path")
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "dataset_path",
                    param.value,
                    "Invalid value set during initialization for parameter dataset_path: "
                    + validation_result,
                )
            updated_params.dataset_path = param.value
            param = self.node_.get_parameter(self.prefix_ + "dataset_name")
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "dataset_name",
                    param.value,
                    "Invalid value set during initialization for parameter dataset_name: "
                    + validation_result,
                )
            updated_params.dataset_name = param.value
            param = self.node_.get_parameter(self.prefix_ + "use_depth")
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            updated_params.use_depth = param.value
            param = self.node_.get_parameter(self.prefix_ + "depth_scale")
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "depth_scale",
                    param.value,
                    "Invalid value set during initialization for parameter depth_scale: "
                    + validation_result,
                )
            updated_params.depth_scale = param.value
            param = self.node_.get_parameter(self.prefix_ + "geometry_unit_in_meter")
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "geometry_unit_in_meter",
                    param.value,
                    "Invalid value set during initialization for parameter geometry_unit_in_meter: "
                    + validation_result,
                )
            updated_params.geometry_unit_in_meter = param.value
            param = self.node_.get_parameter(self.prefix_ + "n_corr_iterations")
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "n_corr_iterations",
                    param.value,
                    "Invalid value set during initialization for parameter n_corr_iterations: "
                    + validation_result,
                )
            updated_params.n_corr_iterations = param.value
            param = self.node_.get_parameter(self.prefix_ + "n_update_iterations")
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "n_update_iterations",
                    param.value,
                    "Invalid value set during initialization for parameter n_update_iterations: "
                    + validation_result,
                )
            updated_params.n_update_iterations = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "tikhonov_parameter_rotation"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "tikhonov_parameter_rotation",
                    param.value,
                    "Invalid value set during initialization for parameter tikhonov_parameter_rotation: "
                    + validation_result,
                )
            updated_params.tikhonov_parameter_rotation = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "tikhonov_parameter_translation"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "tikhonov_parameter_translation",
                    param.value,
                    "Invalid value set during initialization for parameter tikhonov_parameter_translation: "
                    + validation_result,
                )
            updated_params.tikhonov_parameter_translation = param.value
            param = self.node_.get_parameter(self.prefix_ + "models")
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.fixed_size(param, 2)
            if validation_result:
                raise InvalidParameterValueException(
                    "models",
                    param.value,
                    "Invalid value set during initialization for parameter models: "
                    + validation_result,
                )
            validation_result = ParameterValidators.subset_of(
                param, "region_model", "depth_model"
            )
            if validation_result:
                raise InvalidParameterValueException(
                    "models",
                    param.value,
                    "Invalid value set during initialization for parameter models: "
                    + validation_result,
                )
            validation_result = ParameterValidators.unique(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "models",
                    param.value,
                    "Invalid value set during initialization for parameter models: "
                    + validation_result,
                )
            updated_params.models = param.value
            param = self.node_.get_parameter(self.prefix_ + "tracked_objects")
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            updated_params.tracked_objects = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.n_lines_max"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.n_lines_max",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.n_lines_max: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.n_lines_max = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.use_adaptive_coverage"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            updated_params.__map_tracked_objects.region_modality.use_adaptive_coverage = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.max_radius_depth_offset"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.max_radius_depth_offset",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.max_radius_depth_offset: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.max_radius_depth_offset = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.min_continuous_distance"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.min_continuous_distance",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.min_continuous_distance: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.min_continuous_distance = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.function_length"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.function_length",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.function_length: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.function_length = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.distribution_length"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.distribution_length",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.distribution_length: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.distribution_length = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.function_amplitude"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.function_amplitude",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.function_amplitude: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.function_amplitude = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.function_slope"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.function_slope",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.function_slope: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.function_slope = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.learning_rate"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.learning_rate",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.learning_rate: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.learning_rate = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.n_global_iterations"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.n_global_iterations",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.n_global_iterations: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.n_global_iterations = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.scales"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.scales",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.scales: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.scales = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.standard_deviations"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.standard_deviations",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.standard_deviations: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.standard_deviations = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_ + "__map_tracked_objects.region_modality.histogram.n_bins"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.histogram.n_bins",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.histogram.n_bins: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.histogram.n_bins = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.histogram.learning_rate_f"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.histogram.learning_rate_f",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.histogram.learning_rate_f: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.histogram.learning_rate_f = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.histogram.learning_rate_b"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.histogram.learning_rate_b",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.histogram.learning_rate_b: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.histogram.learning_rate_b = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.histogram.unconsidered_line_length"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.histogram.unconsidered_line_length",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.histogram.unconsidered_line_length: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.histogram.unconsidered_line_length = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.histogram.max_considered_line_length"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.histogram.max_considered_line_length",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.histogram.max_considered_line_length: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.histogram.max_considered_line_length = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.n_unoccluded_iterations"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.occlusion.n_unoccluded_iterations",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.occlusion.n_unoccluded_iterations: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.occlusion.n_unoccluded_iterations = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.min_n_unoccluded_lines"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.occlusion.min_n_unoccluded_lines",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.occlusion.min_n_unoccluded_lines: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.occlusion.min_n_unoccluded_lines = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.measure_occlusions"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            updated_params.__map_tracked_objects.region_modality.occlusion.measure_occlusions = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.measured_depth_offset_radius"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.occlusion.measured_depth_offset_radius",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.occlusion.measured_depth_offset_radius: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.occlusion.measured_depth_offset_radius = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.measured_occlusion_radius"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.occlusion.measured_occlusion_radius",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.occlusion.measured_occlusion_radius: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.occlusion.measured_occlusion_radius = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.measured_occlusion_threshold"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.occlusion.measured_occlusion_threshold",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.occlusion.measured_occlusion_threshold: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.occlusion.measured_occlusion_threshold = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.model_occlusions"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            updated_params.__map_tracked_objects.region_modality.occlusion.model_occlusions = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.modeled_depth_offset_radius"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.occlusion.modeled_depth_offset_radius",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.occlusion.modeled_depth_offset_radius: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.occlusion.modeled_depth_offset_radius = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.modeled_occlusion_radius"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.occlusion.modeled_occlusion_radius",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.occlusion.modeled_occlusion_radius: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.occlusion.modeled_occlusion_radius = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.region_modality.occlusion.modeled_occlusion_threshold"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.region_modality.occlusion.modeled_occlusion_threshold",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.region_modality.occlusion.modeled_occlusion_threshold: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.region_modality.occlusion.modeled_occlusion_threshold = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "__map_tracked_objects.depth_modality.n_points_max"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.n_points_max",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.n_points_max: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.n_points_max = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.use_adaptive_coverage"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            updated_params.__map_tracked_objects.depth_modality.use_adaptive_coverage = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.reference_surface_area"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.reference_surface_area",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.reference_surface_area: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.reference_surface_area = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "__map_tracked_objects.depth_modality.stride_length"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.stride_length",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.stride_length: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.stride_length = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.considered_distances"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.considered_distances",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.considered_distances: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.considered_distances = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.standard_deviations"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.not_empty(param)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.standard_deviations",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.standard_deviations: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.standard_deviations = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.n_unoccluded_iterations"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 1)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.occlusion.n_unoccluded_iterations",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.occlusion.n_unoccluded_iterations: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.occlusion.n_unoccluded_iterations = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.min_n_unoccluded_points"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt_eq(param, 0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.occlusion.min_n_unoccluded_points",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.occlusion.min_n_unoccluded_points: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.occlusion.min_n_unoccluded_points = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.measure_occlusions"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            updated_params.__map_tracked_objects.depth_modality.occlusion.measure_occlusions = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.measured_depth_offset_radius"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.occlusion.measured_depth_offset_radius",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.occlusion.measured_depth_offset_radius: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.occlusion.measured_depth_offset_radius = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.measured_occlusion_radius"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.occlusion.measured_occlusion_radius",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.occlusion.measured_occlusion_radius: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.occlusion.measured_occlusion_radius = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.measured_occlusion_threshold"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.occlusion.measured_occlusion_threshold",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.occlusion.measured_occlusion_threshold: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.occlusion.measured_occlusion_threshold = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.model_occlusions"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            updated_params.__map_tracked_objects.depth_modality.occlusion.model_occlusions = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.modeled_depth_offset_radius"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.occlusion.modeled_depth_offset_radius",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.occlusion.modeled_depth_offset_radius: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.occlusion.modeled_depth_offset_radius = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_radius"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_radius",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.occlusion.modeled_occlusion_radius: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_radius = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_threshold"
            )
            self.logger_.debug(
                param.name + ": " + param.type_.name + " = " + str(param.value)
            )
            validation_result = ParameterValidators.gt(param, 0.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_threshold",
                    param.value,
                    "Invalid value set during initialization for parameter __map_tracked_objects.depth_modality.occlusion.modeled_occlusion_threshold: "
                    + validation_result,
                )
            updated_params.__map_tracked_objects.depth_modality.occlusion.modeled_occlusion_threshold = param.value

            # declare and set all dynamic parameters

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.sphere_radius"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0.0
                    descriptor.floating_point_range[-1].to_value = float("inf")
                    parameter = entry.sphere_radius
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt(param, 0.0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.sphere_radius",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.sphere_radius: "
                        + validation_result,
                    )
                entry.sphere_radius = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.n_divides"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.integer_range.append(IntegerRange())
                    descriptor.integer_range[-1].from_value = 1
                    descriptor.integer_range[-1].to_value = 2**31 - 1
                    parameter = entry.n_divides
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt_eq(param, 1)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.n_divides",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.n_divides: "
                        + validation_result,
                    )
                entry.n_divides = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.n_points_max"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.integer_range.append(IntegerRange())
                    descriptor.integer_range[-1].from_value = 1
                    descriptor.integer_range[-1].to_value = 2**31 - 1
                    parameter = entry.n_points_max
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt_eq(param, 1)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.n_points_max",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.n_points_max: "
                        + validation_result,
                    )
                entry.n_points_max = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.max_radius_depth_offset"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0.0
                    descriptor.floating_point_range[-1].to_value = float("inf")
                    parameter = entry.max_radius_depth_offset
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt(param, 0.0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.max_radius_depth_offset",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.max_radius_depth_offset: "
                        + validation_result,
                    )
                entry.max_radius_depth_offset = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.stride_depth_offset"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0.0
                    descriptor.floating_point_range[-1].to_value = float("inf")
                    parameter = entry.stride_depth_offset
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt(param, 0.0)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.stride_depth_offset",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.stride_depth_offset: "
                        + validation_result,
                    )
                entry.stride_depth_offset = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.use_random_seed"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    parameter = entry.use_random_seed
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                entry.use_random_seed = param.value

            for value_1 in updated_params.models:
                updated_params.add_entry(value_1)
                entry = updated_params.get_entry(value_1)
                param_name = f"{self.prefix_}{value_1}.image_size"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(
                        description="@mfoumy plz help.", read_only=False
                    )
                    descriptor.integer_range.append(IntegerRange())
                    descriptor.integer_range[-1].from_value = 1
                    descriptor.integer_range[-1].to_value = 2**31 - 1
                    parameter = entry.image_size
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(
                    param.name + ": " + param.type_.name + " = " + str(param.value)
                )
                validation_result = ParameterValidators.gt_eq(param, 1)
                if validation_result:
                    raise InvalidParameterValueException(
                        "__map_models.image_size",
                        param.value,
                        "Invalid value set during initialization for parameter __map_models.image_size: "
                        + validation_result,
                    )
                entry.image_size = param.value

            self.update_internal_params(updated_params)
