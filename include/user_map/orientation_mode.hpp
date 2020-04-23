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

}

#endif // ORIENTATION_MODE_HPP
