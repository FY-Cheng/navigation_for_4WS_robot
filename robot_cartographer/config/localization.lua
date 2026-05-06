include "robot_lds_3D.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
return options