file(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/rubbing_hand/msg"
  "../src/rubbing_hand/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/rubbing_hand/msg/__init__.py"
  "../src/rubbing_hand/msg/_Float64.py"
  "../src/rubbing_hand/msg/_Float64Array.py"
  "../src/rubbing_hand/msg/_dynamixel_msg.py"
  "../src/rubbing_hand/msg/_dynamixel_param_msg.py"
  "../src/rubbing_hand/msg/_inhand.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
