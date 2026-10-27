# Fetches the shipped policy models from the GitLab Model Registry at configure time.
# See the "Model registry" section of the README.

if(NOT DEFINED PAL_MODEL_REGISTRY_URL)
  set(PAL_MODEL_REGISTRY_URL
    "https://gitlab.pal-robotics.com/api/v4/projects/control%2Fpal_policy_deployer"
    CACHE STRING "GitLab project API URL whose model registry holds the policy models")
endif()
if(NOT DEFINED PAL_MODELS_SOURCE_DIR)
  set(PAL_MODELS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models")
endif()
if(NOT DEFINED PAL_MODELS_BINARY_DIR)
  set(PAL_MODELS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/models")
endif()

# pal_policy_model_file(<version_id> <relpath> <sha256>)
#
# Makes models/<relpath> available for installation.
#   version_id: numeric model version id, the last number of the version page URL
#               (.../-/ml/models/<model_id>/versions/<version_id>).
#   relpath:    install path below share/pal_policy_deployer/models. The file is stored
#               in the registry version under its basename.
#   sha256:     expected checksum of the downloaded file.
#
# If ${PAL_MODELS_SOURCE_DIR}/<relpath> exists it is used as is and nothing is fetched.
# Otherwise the file is downloaded to ${PAL_MODELS_BINARY_DIR}/<relpath>, authenticated
# with $GITLAB_TOKEN.
# Fails configure if no token is set, the download fails or the checksum mismatches.
function(pal_policy_model_file version_id relpath sha256)
  if(EXISTS "${PAL_MODELS_SOURCE_DIR}/${relpath}")
    return()
  endif()

  # Keep a verified copy across re-configures instead of downloading every time.
  set(dst "${PAL_MODELS_BINARY_DIR}/${relpath}")
  if(EXISTS "${dst}")
    file(SHA256 "${dst}" actual)
    if(actual STREQUAL sha256)
      return()
    endif()
  endif()

  if("$ENV{GITLAB_TOKEN}" STREQUAL "")
    message(FATAL_ERROR
      "Policy model '${relpath}' is not in ${PAL_MODELS_SOURCE_DIR} and GITLAB_TOKEN is not set "
      "to fetch it from the model registry. See the 'Model registry' section of the README.")
  endif()
  # The ml_models package endpoint only accepts Bearer auth (PRIVATE-TOKEN gets 401/404).
  set(header "Authorization: Bearer $ENV{GITLAB_TOKEN}")

  get_filename_component(name "${relpath}" NAME)
  set(url "${PAL_MODEL_REGISTRY_URL}/packages/ml_models/${version_id}/files/${name}")
  message(STATUS "Fetching policy model ${relpath} from ${url}")
  # The hash is checked by hand: with EXPECTED_HASH, file(DOWNLOAD) reports a failed
  # transfer (e.g. 404) as a hash mismatch and hides the real error.
  file(DOWNLOAD "${url}" "${dst}"
    HTTPHEADER "${header}"
    TLS_VERIFY ON
    STATUS status)
  list(GET status 0 code)
  if(NOT code EQUAL 0)
    # file(DOWNLOAD) leaves an empty/partial file behind on failure.
    file(REMOVE "${dst}")
    list(GET status 1 reason)
    message(FATAL_ERROR "Failed to fetch policy model '${relpath}' from ${url}: ${reason}")
  endif()
  file(SHA256 "${dst}" actual)
  if(NOT actual STREQUAL sha256)
    file(REMOVE "${dst}")
    message(FATAL_ERROR
      "SHA256 mismatch for policy model '${relpath}' from ${url}: "
      "expected ${sha256}, got ${actual}")
  endif()
endfunction()

# pal_policy_models_from_yaml(<manifest.yaml>)
#
# Calls pal_policy_model_file() for every file listed in a YAML manifest of the form
#   <relpath of the model directory below models/>:
#     model: <registry model name>      # informative only
#     version: <registry version>       # informative only
#     version_id: <numeric version id>
#     files:
#       <file name>: <sha256>
# Needs python3 with PyYAML. Fails configure if the manifest cannot be read.
set(_PAL_READ_MODELS_MANIFEST "${CMAKE_CURRENT_LIST_DIR}/read_models_manifest.py")
function(pal_policy_models_from_yaml manifest)
  find_program(PAL_PYTHON3 python3 REQUIRED)
  execute_process(COMMAND ${PAL_PYTHON3} "${_PAL_READ_MODELS_MANIFEST}" "${manifest}"
    RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR "Cannot read policy model manifest ${manifest}:\n${err}")
  endif()
  string(STRIP "${out}" out)
  if(out STREQUAL "")
    return()
  endif()
  string(REPLACE "\n" ";" lines "${out}")
  foreach(line IN LISTS lines)
    string(REPLACE "|" ";" fields "${line}")
    pal_policy_model_file(${fields})
  endforeach()
endfunction()
