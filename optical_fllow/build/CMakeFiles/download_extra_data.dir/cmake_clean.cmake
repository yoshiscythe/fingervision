file(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/rubbing_hand/msg"
  "../src/rubbing_hand/srv"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/download_extra_data.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
