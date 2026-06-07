file(REMOVE_RECURSE
  "../../lib/liblvr2_static.a"
  "../../lib/liblvr2_static.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/lvr2_static.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
