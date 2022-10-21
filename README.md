# VTKm-ANARI

Library for mapping VTK-m datasets to ANARI scene objects.

## Build + install

Building VTKm-ANARI requires the following:

- CMake 3.17+
- C++17 compiler
- [VTK-m](https://github.com/Kitware/VTK-m) 1.9
- [ANARI-SDK](https://github.com/KhronosGroup/ANARI-SDK)

VTK-m and ANARI-SDK can be found via placing their installation locations on
`CMAKE_PREFIX_PATH`.

Note that VTKm must be built with `VTKm_HIDE_PRIVATE_SYMBOLS=OFF`, otherwise
missing symbol errors will show up when linking VTKm-ANARI.

The single `libvtkm_anari` will install to `${CMAKE_INSTALL_PREFIX}/lib`, and is
usable with any VTKm/ANARI app if either it is installed to the same location as
the ANARI-SDK or `libvtkm_anari` is placed on `LD_LIBRARY_PATH` respectively.

VTKm-ANARI is currently only tested on Linux, but Windows support is planned.

## Using VTKm-ANARI from an install with CMake

VTKm-ANARI installs exports a CMake target for the main library:
`vtkm_anari::vtkm_anari`.  This target is found with CMake via
`find_package(vtkm_anari)` in the downstream project.

When the package is found, it will look for VTK-m + ANARI to ensure those
dependencies are present. Use `CMAKE_PREFIX_PATH` to point them just like when
building VTKm-ANARI itself.
