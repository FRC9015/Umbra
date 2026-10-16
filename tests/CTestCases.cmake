# These are ordinary CTest failures. Known defects are labeled for filtering,
# never marked WILL_FAIL or changed to passing expectations.
set(UMBRA_FIXED_CASES
    basis.identity basis.axes basis.mount
    fixed.identity fixed.mount_forward fixed.mount_rear fixed.mount_left
    fixed.mount_right fixed.mount_height fixed.mount_rotated
    fixed.updated_starboard fixed.updated_port
    yaw.zero yaw.positive90 yaw.negative90 yaw.180 yaw.270 yaw.wrap
    yaw.tilt yaw.combined
    optical.zero optical.radial optical.tangential optical.edge
    optical.alternate_intrinsics
    tag.single tag.multi tag.rotated tag.distance tag.planar
    range.tag_distance
    math.euler math.quaternion math.corner_order
    invalid.nonfinite_mount invalid.degenerate_points invalid.bad_intrinsics
    invalid.singular_mount invalid.malformed_config
    field.reference
    benchmark.synthetic_pose
)
set(UMBRA_CONFIG_CASES config.starboard config.port)

foreach(case_name IN LISTS UMBRA_FIXED_CASES UMBRA_CONFIG_CASES)
    add_test(NAME ${case_name} COMMAND umbra_pose_tests ${case_name})
    if(case_name MATCHES "^range\.")
        set_tests_properties(${case_name} PROPERTIES LABELS "known_defect")
    else()
        set_tests_properties(${case_name} PROPERTIES LABELS "baseline")
    endif()
endforeach()
