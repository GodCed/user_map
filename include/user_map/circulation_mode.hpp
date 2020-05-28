#ifndef CIRCULATION_MODE_HPP
#define CIRCULATION_MODE_HPP


namespace user_map {

  namespace Circulation {

    enum Mode
    {
      preferred = 0,
      keep_out = 100
    };

    inline Mode modeFromValue(int value) {
      return static_cast<Mode>(value);
    }

    inline int valueFromMode(Mode mode) {
      return mode;
    }
  }

}



#endif // CIRCULATION_MODE_HPP
