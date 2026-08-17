# HID report generation (src/hid.c) only exists on the central side of a
# split build (or on non-split boards) — mirror ZMK's own app/CMakeLists.txt
# guard so this doesn't try to link against zmk_hid_* on a peripheral image.
if(CONFIG_ZMK_JOYCON2 AND ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL))
  zephyr_library()
  zephyr_library_include_directories(include)
  # ZMK's own app/include (zmk/hid.h, drivers/behavior.h, etc.) is private to
  # its own app target and isn't propagated to sibling out-of-tree modules.
  zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)
  zephyr_library_sources_ifdef(CONFIG_ZMK_JOYCON2_DEBUG_PRINT
    src/joycon2_debug_print.c
    src/behavior_joycon2_debug_print.c
  )
  zephyr_library_sources_ifdef(CONFIG_ZMK_JOYCON2_CENTRAL_TEST
    src/central_survival_test.c
    src/behavior_joycon2_central_test.c
  )
  zephyr_library_sources_ifdef(CONFIG_ZMK_JOYCON2_CONNECT
    src/joycon2_connection.c
    src/behavior_joycon2_connect.c
  )
endif()
