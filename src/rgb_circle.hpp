#pragma once

#include <array>
#include <esp32-hal-rgb-led.h>

namespace rgb {

template <
    unsigned IM,  // max intensity <= 0xff
    unsigned IL,  // intensity limit <= im
    bool R >      // reduce lower intensities
class Circle {
public:
    Circle() : _rgb{0}, _c0(0), _c1(1), _i(0) {}
    
    Circle &operator++() {
        _rgb[_c0] = IL - _i; 
        _rgb[_c1] = _i;

        if (R) {
            // for some leds it looks better with reduced lower intensities
            _rgb[_c0] *= _rgb[_c0];  // square...
            _rgb[_c0] /= IM;         // ...and scale
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
    virtual void update( unsigned r, unsigned g, unsigned b ) const = 0;

private:
    std::array<unsigned, 3> _rgb;  // rgb* colors
    unsigned _c0;  // index of decreasing...
    unsigned _c1;  // ...and increasing color
    unsigned _i;   // intensity [0..il]
};


template <unsigned IM, unsigned IL, bool R>
class NeoCircle : public Circle<IM, IL, R> {
public:
    NeoCircle( int p ) : _p(p) {}

private:
    void update( unsigned r, unsigned g, unsigned b ) const {
        // rgbLedWrite(_p, r, g, b);
        neopixelWrite(_p, r, g, b);
    }

    int _p;

    NeoCircle();
    NeoCircle( NeoCircle & );
};

}  // namespace rgb
