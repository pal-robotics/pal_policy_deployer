cmake_minimum_required(VERSION 3.8)
project(pal_policy_deployer)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(pal_module_cmake_FOUND)
  pal_register_modules(
    module/20_pal_policy_deployer.yaml
  )
elseif(DEFINED ENV{PAL_DISTRO})
  message(FATAL_ERROR "The pal_module_cmake package was not found. Modules not registered.")
endif()

include(cmake/fetch_models.cmake)
pal_policy_models_from_yaml(${CMAKE_CURRENT_SOURCE_DIR}/models.yaml)
# Re-run the fetch when the manifest changes.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS models.yaml)
# Models fetched from the registry land here; merged with the source models/ tree on install.
file(MAKE_DIRECTORY ${PAL_MODELS_BINARY_DIR})
install(DIRECTORY ${PAL_MODELS_BINARY_DIR}/ DESTINATION share/${PROJECT_NAME}/models)

if(BUILD_TESTING)
  add_test(NAME test_fetch_models
    COMMAND ${CMAKE_COMMAND}
      -DHELPER=${CMAKE_CURRENT_SOURCE_DIR}/cmake/fetch_models.cmake
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_fetch_models
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/test_fetch_models.cmake)
  add_test(NAME test_handle_models
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_handle_models.py)
endif()

if(ament_cmake_auto_VERSION VERSION_LESS "2.6.0")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE config models launch)
else()
  ament_auto_package(INSTALL_TO_SHARE config models launch)
endif()
