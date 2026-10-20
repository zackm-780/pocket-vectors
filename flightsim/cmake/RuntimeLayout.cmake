# Assembles the runtime layout next to the binary so it runs in place:
#   <build>/jsbsim/{aircraft,engine,systems}   stock JSBSim files for the core fleet + overlay
#   <build>/assets/...                         profiles, scenarios, ...
# Re-runs only when something under assets/ changes (or on first build).

set(FSIM_CORE_FLEET c172p c310 global5000 737 B747)
# WP F4: extended fleet (additive; same copy rule as the core fleet).
set(FSIM_EXTENDED_FLEET pa28 c182 J3Cub DHC6 A320 787-8 MD11)

function(fsim_add_runtime_layout main_target)
  set(src_assets ${CMAKE_CURRENT_SOURCE_DIR}/assets)
  set(dst_jsbsim ${CMAKE_BINARY_DIR}/jsbsim)
  set(dst_assets ${CMAKE_BINARY_DIR}/assets)
  set(stamp ${CMAKE_BINARY_DIR}/runtime_layout.stamp)

  set(cmds
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${dst_jsbsim}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${dst_jsbsim}/aircraft
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${FSIM_JSBSIM_SOURCE_DIR}/engine  ${dst_jsbsim}/engine
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${FSIM_JSBSIM_SOURCE_DIR}/systems ${dst_jsbsim}/systems)
  foreach(ac IN LISTS FSIM_CORE_FLEET FSIM_EXTENDED_FLEET)
    list(APPEND cmds COMMAND ${CMAKE_COMMAND} -E copy_directory
         ${FSIM_JSBSIM_SOURCE_DIR}/aircraft/${ac} ${dst_jsbsim}/aircraft/${ac})
  endforeach()
  list(APPEND cmds
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${src_assets}/jsbsim-overlay ${dst_jsbsim}
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${dst_assets}
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${src_assets} ${dst_assets}
    COMMAND ${CMAKE_COMMAND} -E touch ${stamp})

  file(GLOB_RECURSE asset_files CONFIGURE_DEPENDS ${src_assets}/*)
  add_custom_command(OUTPUT ${stamp} ${cmds} DEPENDS ${asset_files}
                     COMMENT "Assembling runtime layout (jsbsim/, assets/)" VERBATIM)
  add_custom_target(fsim_runtime ALL DEPENDS ${stamp})
  add_dependencies(${main_target} fsim_runtime)
endfunction()
