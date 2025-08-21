# Viskores-GRAPH

Library implementing a VTK-m node-graph abstraction with support for ANARI
mappers.

## Build + install

Building Viskores-GRAPH requires the following:

- CMake 3.17+
- C++17 compiler
- [VTK-m](https://github.com/Kitware/VTK-m) 2.1
- [ANARI-SDK](https://github.com/KhronosGroup/ANARI-SDK) 0.7.0+

VTK-m and ANARI-SDK can be found via placing their installation locations on
`CMAKE_PREFIX_PATH`.

The single `libviskores_graph` will install to `${CMAKE_INSTALL_PREFIX}/lib`, and is
usable with any Viskores/ANARI app if either it is installed to the same location as
the ANARI-SDK or `libviskores_graph` is placed on `LD_LIBRARY_PATH` respectively.

Viskores-GRAPH is currently only tested on Linux, but Windows support is planned.

### Using the superbuild

You can build ANARI-SDK, VTK-m, and Viskores-GRAPH all in a single build directory
using the superbuild found in `scripts/superbuild`. Simply run a standard
CMake config + build using that CMakeLists.txt and all three projects will be
installed to `CMAKE_INSTALL_PREFIX`. Note that the superbuild is a separate
entity to building Viskores-GRAPH by hand using the CMakeLists.txt found in the
root source directory.

## Using Viskores-GRAPH from an install with CMake

Viskores-GRAPH installs exports a CMake target for the main library:
`viskores_graph::viskores_graph`.  This target is found with CMake via
`find_package(viskores_graph)` in the downstream project.

When the package is found, it will look for VTK-m + ANARI to ensure those
dependencies are present. Use `CMAKE_PREFIX_PATH` to point them just like when
building Viskores-GRAPH itself.
