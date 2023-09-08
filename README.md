# VTKm-GRAPH

Library implementing a VTK-m node-graph abstraction with support for ANARI
mappers.

## Build + install

Building VTKm-GRAPH requires the following:

- CMake 3.17+
- C++17 compiler
- [VTK-m](https://github.com/Kitware/VTK-m) 2.1
- [ANARI-SDK](https://github.com/KhronosGroup/ANARI-SDK) 0.7.0+

VTK-m and ANARI-SDK can be found via placing their installation locations on
`CMAKE_PREFIX_PATH`.

The single `libvtkm_graph` will install to `${CMAKE_INSTALL_PREFIX}/lib`, and is
usable with any VTKm/ANARI app if either it is installed to the same location as
the ANARI-SDK or `libvtkm_graph` is placed on `LD_LIBRARY_PATH` respectively.

VTKm-GRAPH is currently only tested on Linux, but Windows support is planned.

## Using VTKm-GRAPH from an install with CMake

VTKm-GRAPH installs exports a CMake target for the main library:
`vtkm_graph::vtkm_graph`.  This target is found with CMake via
`find_package(vtkm_graph)` in the downstream project.

When the package is found, it will look for VTK-m + ANARI to ensure those
dependencies are present. Use `CMAKE_PREFIX_PATH` to point them just like when
building VTKm-GRAPH itself.
