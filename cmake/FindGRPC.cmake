set(_GRPC_LIBS_DIR ${GRPC_ROOT}/lib)
set(_GRPC_LIBS64_DIR ${GRPC_ROOT}/lib64)
set(_GRPC_INCLUDE_DIR ${GRPC_ROOT}/include)

#find_path(PROTOBUF_INCLUDE_DIR google/protobuf/service.h)
find_path(PROTOBUF_INCLUDE_DIR google/protobuf/service.h PATHS ${_GRPC_INCLUDE_DIR} )

# Find gRPC include directory
find_path(GRPC_INCLUDE_DIR grpc/grpc.h PATHS ${_GRPC_INCLUDE_DIR} )

# set libs dependencies for my repo
set (_GRPC_CLIENT_ORDERED_DEPS libgrpc++_reflection.a libgrpc++.a libgrpc_unsecure.a libgpr.a
  libabsl_symbolize.a libabsl_bad_any_cast_impl.a libabsl_bad_optional_access.a
  libabsl_bad_variant_access.a libabsl_base.a libabsl_city.a
  libabsl_civil_time.a libabsl_cord.a libabsl_cord_internal.a
  libabsl_cordz_functions.a libabsl_cordz_handle.a libabsl_cordz_info.a
  libabsl_cordz_sample_token.a libabsl_debugging_internal.a
  libabsl_demangle_internal.a libabsl_examine_stack.a
  libabsl_exponential_biased.a libabsl_failure_signal_handler.a libabsl_flags.a
  libabsl_flags_commandlineflag.a libabsl_flags_commandlineflag_internal.a
  libabsl_flags_config.a libabsl_flags_internal.a libabsl_flags_marshalling.a
  libabsl_flags_parse.a libabsl_flags_private_handle_accessor.a
  libabsl_flags_program_name.a libabsl_flags_reflection.a libabsl_flags_usage.a
  libabsl_flags_usage_internal.a libabsl_graphcycles_internal.a libabsl_hash.a
  libabsl_hashtablez_sampler.a libabsl_int128.a libabsl_leak_check.a
  libabsl_log_severity.a libabsl_low_level_hash.a libabsl_malloc_internal.a
  libabsl_periodic_sampler.a libabsl_random_distributions.a
  libabsl_random_internal_distribution_test_util.a
  libabsl_random_internal_platform.a libabsl_random_internal_pool_urbg.a
  libabsl_random_internal_randen.a libabsl_random_internal_randen_hwaes.a
  libabsl_random_internal_randen_hwaes_impl.a
  libabsl_random_internal_randen_slow.a libabsl_random_internal_seed_material.a
  libabsl_random_seed_gen_exception.a libabsl_random_seed_sequences.a
  libabsl_raw_hash_set.a libabsl_raw_logging_internal.a libabsl_scoped_set_env.a
  libabsl_spinlock_wait.a libabsl_stacktrace.a libabsl_status.a
  libabsl_statusor.a libabsl_strerror.a libabsl_str_format_internal.a
  libabsl_strings.a libabsl_strings_internal.a
  libabsl_synchronization.a libabsl_throw_delegate.a libabsl_time.a
  libabsl_time_zone.a libaddress_sorting.a libprotobuf.a libupb.a libcares.a
  libz.a)

set(GRPC_CLIENT_LIBS)
foreach (DEP ${_GRPC_CLIENT_ORDERED_DEPS})
  if (EXISTS ${_GRPC_LIBS_DIR}/${DEP})
    list(APPEND GRPC_CLIENT_LIBS ${_GRPC_LIBS_DIR}/${DEP} )
  elseif (EXISTS ${_GRPC_LIBS64_DIR}/${DEP})
    list(APPEND GRPC_CLIENT_LIBS ${_GRPC_LIBS64_DIR}/${DEP} )
  else()
    message( SEND_ERROR "Missing gRPC dependency ${DEP} " )
  endif()
endforeach()
