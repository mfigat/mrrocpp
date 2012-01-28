add_library(epos epos.cc)
	
target_link_libraries(epos canopen)

# Test utilities
add_executable(test test.cc)
target_link_libraries (test epos rt)

add_executable(sctest sctest.cc)
target_link_libraries (sctest epos rt)

add_executable(ipmtest ipmtest.cc)
target_link_libraries (ipmtest epos rt)

add_executable(cantest cantest.cc)
target_link_libraries (cantest epos)

add_executable(canopen_reset_node canopen_reset_node.cc)
target_link_libraries (canopen_reset_node epos)

# SMB testing. 
add_executable(smb_synchro_test smb_synchro_test.cc)
target_link_libraries (smb_synchro_test epos)

add_executable(smb_move_test smb_move_test.cc)
target_link_libraries (smb_move_test epos)


install(TARGETS epos DESTINATION lib)
