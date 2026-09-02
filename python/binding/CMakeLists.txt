# CMakeLists.
#
# SPDX-FileCopyrightText: 2014-2023 Institute for Automation of Complex Power Systems, RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

set(VILLAS_PYTHON_INSTALL_DIR
    "${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
    CACHE STRING "Installation directory for the Python package"
)

pybind11_add_module(python-bindings module.cpp)

target_link_libraries(python-bindings PRIVATE villas)

set_target_properties(python-bindings PROPERTIES
    OUTPUT_NAME _core
    INSTALL_RPATH "$ORIGIN"
)

execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import pybind11_stubgen"
    RESULT_VARIABLE PYBIND11_STUBGEN_MISSING
    OUTPUT_QUIET
    ERROR_QUIET
)

if(NOT PYBIND11_STUBGEN_MISSING)
    add_custom_command(TARGET python-bindings POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E env
            PYTHONPATH=$<TARGET_FILE_DIR:python-bindings>
            ${Python3_EXECUTABLE} -m pybind11_stubgen _core
                --output-dir ${CMAKE_CURRENT_BINARY_DIR}
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/py.typed
        VERBATIM
    )

    install(
        FILES
            ${CMAKE_CURRENT_BINARY_DIR}/_core.pyi
            ${CMAKE_CURRENT_BINARY_DIR}/py.typed
        COMPONENT python
        DESTINATION ${VILLAS_PYTHON_INSTALL_DIR}/villas/node
    )
endif()

install(
    TARGETS python-bindings
    COMPONENT python
    LIBRARY DESTINATION ${VILLAS_PYTHON_INSTALL_DIR}/villas/node
)
