# Strict warning flags for production firmware (applied per-target, never globally
# so third-party code like GoogleTest stays warning-clean on its own terms).
set(FOC_STRICT_C_FLAGS
    -Wall -Wextra -Wpedantic -Wconversion -Werror
    -Wdouble-promotion -Wshadow -Wstrict-prototypes -Wmissing-prototypes
)
set(FOC_STRICT_CXX_FLAGS
    -Wall -Wextra -Wpedantic -Wconversion -Werror
    -Wdouble-promotion -Wshadow -Wnon-virtual-dtor -Wold-style-cast
)

# NOTE: strict flags are applied per-target in the root CMakeLists.txt via
# target_compile_options (never globally, so GTest stays clean). This module
# only handles optional sanitizers.

# Sanitizers for SIL debug builds (address + undefined). Applied only when
# FOC_ENABLE_SANITIZERS=ON and compiler supports them.
option(FOC_ENABLE_SANITIZERS "Enable ASan/UBSan for SIL builds" ON)
if(FOC_ENABLE_SANITIZERS AND TARGET_SIL AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    include(CheckCCompilerFlag)
    check_c_compiler_flag("-fsanitize=address,undefined" FOC_HAS_SAN)
    if(FOC_HAS_SAN)
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address,undefined)
    endif()
endif()
