add_library(airspy_one_contract STATIC src/protocol.c)
target_include_directories(airspy_one_contract PUBLIC include)
target_compile_features(airspy_one_contract PUBLIC c_std_11)
set_target_properties(airspy_one_contract PROPERTIES
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO)

add_executable(airspy_one_contract_test tests/protocol_test.c)
target_link_libraries(airspy_one_contract_test PRIVATE airspy_one_contract)
add_test(NAME contract.protocol COMMAND airspy_one_contract_test)
