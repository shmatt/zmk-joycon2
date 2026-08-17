if(CONFIG_ZMK_JOYCON2)
  zephyr_library()
  zephyr_library_include_directories(include)
  # ZMK's own app/include (zmk/hid.h, drivers/behavior.h, etc.) is private to
  # its own app target and isn't propagated to sibling out-of-tree modules.
  zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)
  zephyr_library_sources_ifdef(CONFIG_ZMK_JOYCON2_DEBUG_PRINT
    src/joycon2_debug_print.c
    src/behavior_joycon2_debug_print.c
  )
endif()
