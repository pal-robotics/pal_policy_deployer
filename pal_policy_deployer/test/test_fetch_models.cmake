# Tests cmake/fetch_models.cmake against a fake registry served over file://.
# Usage: cmake -DHELPER=<path/to/fetch_models.cmake> -DWORK_DIR=<scratch dir> -P test_fetch_models.cmake

file(REMOVE_RECURSE "${WORK_DIR}")
set(registry "${WORK_DIR}/registry")
file(WRITE "${registry}/packages/ml_models/7/files/a.onnx" "model-a")
file(SHA256 "${registry}/packages/ml_models/7/files/a.onnx" sha_a)
file(WRITE "${WORK_DIR}/src/models/local/b.onnx" "local-b")

set(PAL_MODEL_REGISTRY_URL "file://${registry}")
set(PAL_MODELS_SOURCE_DIR "${WORK_DIR}/src/models")
set(PAL_MODELS_BINARY_DIR "${WORK_DIR}/bin/models")
set(ENV{GITLAB_TOKEN} "fake-token")
include("${HELPER}")

# Local file wins: nothing is downloaded.
pal_policy_model_file(7 local/b.onnx 0000000000000000000000000000000000000000000000000000000000000000)
if(EXISTS "${PAL_MODELS_BINARY_DIR}/local/b.onnx")
  message(FATAL_ERROR "local model was downloaded although it exists in the source tree")
endif()

# Missing file is fetched into the binary dir under its manifest path.
pal_policy_model_file(7 body/task/a.onnx ${sha_a})
file(READ "${PAL_MODELS_BINARY_DIR}/body/task/a.onnx" content)
if(NOT content STREQUAL "model-a")
  message(FATAL_ERROR "downloaded model has unexpected content: '${content}'")
endif()

# YAML manifest: every file of an entry lands in the entry's directory.
file(WRITE "${WORK_DIR}/models.yaml"
  "body/yaml_task:\n"
  "  model: stable/body_task\n"
  "  version: 1.0.0\n"
  "  version_id: 7\n"
  "  files:\n"
  "    a.onnx: ${sha_a}\n")
pal_policy_models_from_yaml("${WORK_DIR}/models.yaml")
if(NOT EXISTS "${PAL_MODELS_BINARY_DIR}/body/yaml_task/a.onnx")
  message(FATAL_ERROR "model listed in the YAML manifest was not fetched")
endif()

# A manifest without entries is valid.
file(WRITE "${WORK_DIR}/empty.yaml" "# no models yet\n")
pal_policy_models_from_yaml("${WORK_DIR}/empty.yaml")

# Failure cases abort configure; each runs in its own cmake process.
function(expect_failure name call expected_msg)
  set(script "${WORK_DIR}/${name}.cmake")
  file(WRITE "${script}"
    "set(PAL_MODEL_REGISTRY_URL \"${PAL_MODEL_REGISTRY_URL}\")\n"
    "set(PAL_MODELS_SOURCE_DIR \"${PAL_MODELS_SOURCE_DIR}\")\n"
    "set(PAL_MODELS_BINARY_DIR \"${WORK_DIR}/bin_${name}/models\")\n"
    "include(\"${HELPER}\")\n"
    "${call}\n")
  execute_process(COMMAND ${CMAKE_COMMAND} -E env ${ARGN} ${CMAKE_COMMAND} -P "${script}"
    RESULT_VARIABLE rc ERROR_VARIABLE err OUTPUT_QUIET)
  if(rc EQUAL 0)
    message(FATAL_ERROR "${name}: expected failure, got success")
  endif()
  if(NOT err MATCHES "${expected_msg}")
    message(FATAL_ERROR "${name}: error does not mention '${expected_msg}':\n${err}")
  endif()
endfunction()

expect_failure(wrong_hash
  "pal_policy_model_file(7 body/task/a.onnx 1111111111111111111111111111111111111111111111111111111111111111)"
  "[Hh][Aa][Ss][Hh]")
expect_failure(missing_version
  "pal_policy_model_file(99 body/task/a.onnx ${sha_a})"
  "Failed to fetch")
expect_failure(no_token
  "pal_policy_model_file(7 body/task/a.onnx ${sha_a})"
  "README"
  --unset=GITLAB_TOKEN)

file(WRITE "${WORK_DIR}/no_files.yaml" "body/yaml_task:\n  version_id: 7\n")
expect_failure(yaml_missing_files
  "pal_policy_models_from_yaml(\"${WORK_DIR}/no_files.yaml\")"
  "files")

message(STATUS "test_fetch_models: all checks passed")
