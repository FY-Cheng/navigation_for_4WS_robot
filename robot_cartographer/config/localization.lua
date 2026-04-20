include "robot_lds_3D.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85
return options