if(CONFIG_ZMK_JOYCON2)
  zephyr_library()
  zephyr_library_include_directories(include)
  zephyr_library_sources_ifdef(CONFIG_ZMK_JOYCON2_DEBUG_PRINT
    src/joycon2_debug_print.c
    src/behavior_joycon2_debug_print.c
  )
endif()
