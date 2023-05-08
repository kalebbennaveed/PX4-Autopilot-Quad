px4_add_module(
  MODULE modules__quad_control
  MAIN quad_control
  COMPILE_FLAGS
  SRCS
    QuadControl.cpp
    QuadControl.hpp
    Controllers/MixerQuadratic.hpp
    Controllers/MixerQuadratic.cpp
    Controllers/GeometricController.hpp
    Controllers/GeometricController.cpp
    Controllers/DiffFlat.hpp
  DEPENDS
)

