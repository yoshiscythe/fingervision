file(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/rubbing_hand/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/download_extra_data.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
