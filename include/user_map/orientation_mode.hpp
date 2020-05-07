#ifndef ORIENTATION_MODE_HPP
#define ORIENTATION_MODE_HPP

namespace user_map {

  enum OrientationMode
  {
    none,
    tangent,
    parallel,
    fixed
  };

  inline OrientationMode OrientationMode_from_value(int value) {
    return  static_cast<OrientationMode>(value / 1000);
  }

  inline int value_from_OrientationMode(OrientationMode mode, int angle) {
    return 1000*mode + angle;
  }

}

#endif // ORIENTATION_MODE_HPP
