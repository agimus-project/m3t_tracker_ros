#!/usr/bin/env python

import numpy as np

import pym3t

if __name__ == "__main__":
    renderer_geometry = pym3t.RendererGeometry("renderer geometry")

    # Setup body model and properties
    body_name = "obj_000005"
    obj_model_path = f"{body_name}.obj"
    body = pym3t.Body(
        name=body_name,
        geometry_path=obj_model_path,
        geometry_unit_in_meter=0.001,
        geometry_counterclockwise=1,
        geometry_enable_culling=1,
        geometry2body_pose=np.eye(4),
    )
    renderer_geometry.AddBody(body)
    link = pym3t.Link(body_name + "_link", body)

    color_camera = pym3t.DummyColorCamera("cam_color")
    # color_camera.color2depth_pose = tq_to_SE3(cam_intrinsics['trans_d_c'], cam_intrinsics['quat_d_c_xyzw'])
    # color_camera.intrinsics = pym3t.Intrinsics(**cam_intrinsics['intrinsics_color'])

    region_model_path = body_name + "_region_model.bin"
    region_model = pym3t.RegionModel(
        body_name + "_region_model", body, region_model_path
    )
    region_modality = pym3t.RegionModality(
        body_name + "_region_modality", body, color_camera, region_model
    )
    link.AddModality(region_modality)

    optimizer = pym3t.Optimizer(body_name + "_optimizer", link)
    tracker = pym3t.Tracker("tracker", synchronize_cameras=False)
    tracker.AddOptimizer(optimizer)
    tracker.SetUp()

    # # Creating models and modalities for all specified objects
    # for bname in self.bodies:
    #     body = self.bodies[bname]
    #     link = self.links[bname]

    #     self.renderer_geometry.AddBody(body)

    #     if self.use_region_modality:
    #         self.region_models[bname] = create_region_model(body, self.cfg.region_model, self.tmp_dir)
    #     if self.use_depth_modality:
    #         self.depth_models[bname] = create_depth_model(body, self.cfg.depth_model, self.tmp_dir)

    #     for i, rmc in self.cfg.region_modalities.items():
    #         depth_cam = self.depth_cameras[i] if i in self.depth_cameras else None
    #         occl_renderer = self.self.occl_color_renderers[i] if i in self.occl_color_renderers else None
    #         region_modality = create_region_modality(self.region_models[bname], body, self.color_cameras[i], rmc, depth_cam, occl_renderer)
    #         link.AddModality(region_modality)

    #     for i, dmc in self.cfg.depth_modalities.items():
    #         occl_renderer = self.self.occl_depth_renderers[i] if i in self.occl_depth_renderers else None
    #         depth_modality = create_depth_modality(self.depth_models[bname], body, self.depth_cameras[i], dmc, occl_renderer)
    #         link.AddModality(depth_modality)

    #     # Add remove optimizers?
    #     self.optimizers[bname] = pym3t.Optimizer(bname+'_optimizer', link, self.cfg.tikhonov_parameter_rotation, self.cfg.tikhonov_parameter_translation)
    #     self.tracker.AddOptimizer(self.optimizers[bname])
