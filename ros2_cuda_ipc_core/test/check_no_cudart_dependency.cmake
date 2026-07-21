# Verify the ELF dependency list rather than relying on link-line inspection:
# transitive dependencies are what matter to downstream Runtime API users.
find_program(LDD_EXECUTABLE ldd)
if(NOT LDD_EXECUTABLE)
  message(FATAL_ERROR "ldd is required for the libcudart dependency check")
endif()
execute_process(COMMAND "${LDD_EXECUTABLE}" "${LIBRARY}"
  RESULT_VARIABLE status OUTPUT_VARIABLE dependencies ERROR_VARIABLE errors)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "ldd failed for ${LIBRARY}: ${errors}")
endif()
if(dependencies MATCHES "libcudart\\.so")
  message(FATAL_ERROR "${LIBRARY} must not depend on libcudart:\n${dependencies}")
endif()
