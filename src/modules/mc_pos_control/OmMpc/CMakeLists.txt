# Reuse the exact specialized float32 backend validated by native/SIH replay.
set(QP_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/qpDUNES)
set(OM_FLOAT_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/qpDUNES)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${OM_FLOAT_SOURCE}/specialize_float.py)
set(qp_sources)
foreach(qp_file dual_qp.c matrix_vector.c setup_qp.c qpdunes_utils.c stage_qp_solver_clipping.c)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${QP_SOURCE}/${qp_file})
  set(qp_output ${CMAKE_CURRENT_BINARY_DIR}/float_${qp_file})
  execute_process(COMMAND ${PYTHON_EXECUTABLE} ${OM_FLOAT_SOURCE}/specialize_float.py
    ${QP_SOURCE}/${qp_file} ${qp_output} RESULT_VARIABLE generator_result)
  if(NOT generator_result EQUAL 0)
    message(FATAL_ERROR "OM-MPC float32 specialization failed")
  endif()
  if(qp_file STREQUAL "dual_qp.c")
    file(READ ${qp_output} qp_dual_source)
    string(REPLACE "for ((*itCntr) = 1; (*itCntr) <= qpData->options.maxIter; ++(*itCntr)) {"
      "for ((*itCntr) = 1; (*itCntr) <= qpData->options.maxIter; ++(*itCntr)) {\nif (om_float_om_qp_deadline_expired()) { return QPDUNES_ERR_ITERATION_LIMIT_REACHED; }"
      qp_dual_source "${qp_dual_source}")
    string(FIND "${qp_dual_source}" "if (om_float_om_qp_deadline_expired())" hook_position)
    if(hook_position EQUAL -1)
      message(FATAL_ERROR "qpDUNES iteration deadline hook missing")
    endif()
    file(WRITE ${qp_output} "extern int om_float_om_qp_deadline_expired(void);\n${qp_dual_source}")
  endif()
  list(APPEND qp_sources ${qp_output})
endforeach()
px4_add_library(om_qpdunes ${qp_sources}
  ${CMAKE_CURRENT_SOURCE_DIR}/OmQpSolver.cpp ${CMAKE_CURRENT_SOURCE_DIR}/OmQpArena.c)
# Shared bound for SITL and NuttX. Arena allocation follows the requested N;
# allocation failure still rejects setup while the standby PID owns output.
target_compile_definitions(om_qpdunes PUBLIC OM_MPC_MAX_HORIZON=20)
target_include_directories(om_qpdunes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OM_FLOAT_SOURCE}
  PRIVATE ${QP_SOURCE}/include ${QP_SOURCE}/include/qp ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(om_qpdunes PRIVATE __USE_SINGLE_PRECISION__ __SIMPLE_BOUNDS_ONLY__
  __SUPPRESS_ALL_OUTPUT__ __SUPPRESS_ALL_WARNINGS__ calloc=om_float_om_qp_calloc free=om_float_om_qp_free)
target_compile_options(om_qpdunes PRIVATE -Wno-error -O3 "SHELL:-include ${OM_FLOAT_SOURCE}/float_prefix.h")
target_compile_options(om_qpdunes PRIVATE $<$<COMPILE_LANGUAGE:C>:-fsingle-precision-constant>
  $<$<COMPILE_LANGUAGE:C>:-Werror=implicit-function-declaration>)
# The arena's close function must call libc free, not its solver no-op free.
set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/OmQpArena.c PROPERTIES COMPILE_OPTIONS "-Ucalloc;-Ufree")
