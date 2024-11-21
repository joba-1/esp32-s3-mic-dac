#pragma once

#include <array>
#include <esp32-hal-rgb-led.h>

namespace rgb {

template <unsigned IM,  // max intensity <= 0xff (only used if R == true)
          unsigned IL,  // intensity limit <= im
          bool     RI>  // reduce lower intensities
class Circle {
public:
  typedef enum color { R, G, B } color_t;

  Circle() : _rgb{}, _up{}, _c0(0), _c1(1), _i(0) {}

  // set specific color
  void set(unsigned r, unsigned g, unsigned b) {
    _rgb[0] = min(r, IL);
    _rgb[1] = min(g, IL);
    _rgb[2] = min(b, IL);

    update(_rgb[0], _rgb[1], _rgb[2]);
  }

  // intensity of r, g or b goes up or down one step
  void next(color_t col) {
    unsigned c = (unsigned)col;
    if (c < _rgb.size()) {
      if (_rgb[c] == 0 && !_up[c])
        _up[c] = true;
      else if (_rgb[c] == IL && _up[c])
        _up[c] = false;

      _rgb[c] += _up[c] ? 1 : -1;

      update(_rgb[0], _rgb[1], _rgb[2]);
    }
  }

  // next color in rainbow circle
  Circle &operator++() {
    _rgb[_c0] = IL - _i;
    _rgb[_c1] = _i;
    _rgb[3 - _c0 - _c1] = 0; // set() or next() might have changed it

    if constexpr (RI) {
      // for some leds it looks better with reduced lower intensities
      _rgb[_c0] *= _rgb[_c0]; // square...
      _rgb[_c0] /= IM;        // ...and scale
      _rgb[_c1] *= _rgb[_c1];
      _rgb[_c1] /= IM;
    }

    update(_rgb[0], _rgb[1], _rgb[2]);

    // select intensity and affected colors
    if (++_i > IL) {
      _i = 0;
      if (++_c0 >= _rgb.size()) _c0 = 0;
      if (++_c1 >= _rgb.size()) _c1 = 0;
    }

    return *this;
  }

protected:
  // set pixel to current rgb values
  virtual void update(unsigned r, unsigned g, unsigned b) const = 0;

private:
  std::array<unsigned, 3> _rgb; // rgb* colors
  std::array<bool, 3> _up;      // single color directions
  unsigned _c0;                 // index of decreasing...
  unsigned _c1;                 // ...and increasing color
  unsigned _i;                  // intensity [0..il]
};


template <unsigned IM, unsigned IL, bool R>
class NeoCircle : public Circle<IM, IL, R> {
public:
  NeoCircle(int p) : _p(p) {}

private:
  void update(unsigned r, unsigned g, unsigned b) const {
    rgbLedWrite(_p, r, g, b);
  }

  int _p;
};


template <unsigned IL> class NeoRawCircle : public NeoCircle<0xff, IL, false> {
public:
  NeoRawCircle(int p) : NeoCircle<0xff, IL, false>(p) {}
};

} // namespace rgb
