if(NOT DEFINED binary)
  message(FATAL_ERROR "binary must be set")
endif()

find_program(LDD_EXECUTABLE ldd REQUIRED)
execute_process(
  COMMAND "${LDD_EXECUTABLE}" "${binary}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE dependencies
  ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Unable to inspect ${binary}: ${error}")
endif()
if(dependencies MATCHES "libcudart\\.so")
  message(FATAL_ERROR "${binary} unexpectedly depends on libcudart.so:\n${dependencies}")
endif()
if(NOT dependencies MATCHES "libcuda\\.so")
  message(FATAL_ERROR "${binary} must depend on libcuda.so:\n${dependencies}")
endif()
