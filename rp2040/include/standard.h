#ifndef STD_H
#define STD_H

#include <inttypes.h>
#include <stdbool.h>
#include <math.h>


/* stringify a define, e.g. one that was not quoted */
#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)

#define PTR(_f) &_f

#ifndef FALSE
#define FALSE false
#endif
#ifndef TRUE
#define TRUE true
#endif

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

/* Unit (void) values */
typedef uint8_t unit_t;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_4
#define M_PI_4 (M_PI/4)
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI/2)
#endif


#ifndef bit_is_set
#define bit_is_set(x, b) ((x >> b) & 0x1)
#endif

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

#define SetBit(a, n) a |= (1 << n)
#define ClearBit(a, n) a &= ~(1 << n)

#define NormRadAngle(x) { \
    while (x > M_PI) x -= 2 * M_PI; \
    while (x < -M_PI) x += 2 * M_PI; \
  }
#define DegOfRad(x) ((x) * (180. / M_PI))
#define DeciDegOfRad(x) ((x) * (1800./ M_PI))
#define RadOfDeg(x) ((x) * (M_PI/180.))
#define RadOfDeciDeg(x) ((x) * (M_PI/1800.))

#define MOfCm(_x) (((float)(_x))/100.)
#define MOfMm(_x) (((float)(_x))/1000.)

#define Min(x,y) (x < y ? x : y)
#define Max(x,y) (x > y ? x : y)

#ifndef ABS
#define ABS(val) ((val) < 0 ? -(val) : (val))
#endif

#define BoundUpper(_x, _max) { if (_x > (_max)) _x = (_max);}

#define constrain(amt, low, high) ({ \
    typeof(amt) _amt = (amt); \
    typeof(low) _low = (low); \
    typeof(high) _high = (high); \
    (_amt < _low) ? _low : ((_amt > _high) ? _high : _amt); \
    })


#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }
#define BoundInverted(_x, _min, _max) {           \
    if ((_x < (_min)) && (_x > (_max))) {         \
      if (abs(_x - (_min)) < abs(_x - (_max)))    \
        _x = (_min);                              \
      else                                        \
        _x = (_max);                              \
    }                                             \
  }
#define BoundWrapped(_x, _min, _max) {            \
    if ((_max) > (_min))                          \
      Bound(_x, _min, _max)                       \
      else                                        \
        BoundInverted(_x, _min, _max)             \
      }
#define BoundAbs(_x, _max) Bound(_x, -(_max), (_max))
#define Clip(_x, _min, _max) ( (_x) < (_min) ? (_min) : (_x) > (_max) ? (_max) : (_x) )
#define ClipAbs(x, max) Clip(x, -(max), (max))
// Align makes the value of x a multiple of a1
#define Align(_x, _a1) (_x%_a1 ? _x + (_a1 - (_x%_a1)) : _x )

#define DeadBand(_x, _v) {            \
    if (_x > (_v))                    \
      _x = _x -(_v);                  \
    else if  (_x < -(_v))             \
      _x = _x +(_v);                  \
    else                              \
      _x = 0;                         \
  }

#define Blend(a, b, rho) (((rho)*(a))+(1-(rho))*(b))


static inline bool str_equal(const char *a, const char *b)
{
  int i = 0;
  while (!(a[i] == 0 && b[i] == 0)) {
    if (a[i] != b[i]) { return FALSE; }
    i++;
  }
  return TRUE;
}

struct BoolInt 
{
  bool flag = false;
  uint8_t ErrCode = -1;
};

class Quaternion {
    public:
        float w;
        float x;
        float y;
        float z;
        
        Quaternion() {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }
        
        Quaternion(float nw, float nx, float ny, float nz) {
            w = nw;
            x = nx;
            y = ny;
            z = nz;
        }

        Quaternion getProduct(Quaternion q) {
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

        Quaternion getConjugate() {
            return Quaternion(w, -x, -y, -z);
        }
        
        float getMagnitude() {
            return sqrt(w*w + x*x + y*y + z*z);
        }
        
        void normalize() {
            float m = getMagnitude();
            w /= m;
            x /= m;
            y /= m;
            z /= m;
        }
        
        Quaternion getNormalized() {
            Quaternion r(w, x, y, z);
            r.normalize();
            return r;
        }
};

class VectorInt16 {
    public:
        int16_t x;
        int16_t y;
        int16_t z;

        VectorInt16() {
            x = 0;
            y = 0;
            z = 0;
        }
        
        VectorInt16(int16_t nx, int16_t ny, int16_t nz) {
            x = nx;
            y = ny;
            z = nz;
        }

        float getMagnitude() {
            return sqrt(x*x + y*y + z*z);
        }

        void normalize() {
            float m = getMagnitude();
            x /= m;
            y /= m;
            z /= m;
        }
        
        VectorInt16 getNormalized() {
            VectorInt16 r(x, y, z);
            r.normalize();
            return r;
        }
        
        void rotate(Quaternion *q) {
            // http://www.cprogramming.com/tutorial/3d/quaternions.html
            // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
            // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
            // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1
        
            // P_out = q * P_in * conj(q)
            // - P_out is the output vector
            // - q is the orientation quaternion
            // - P_in is the input vector (a*aReal)
            // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
            Quaternion p(0, x, y, z);

            // quaternion multiplication: q * p, stored back in p
            p = q -> getProduct(p);

            // quaternion multiplication: p * conj(q), stored back in p
            p = p.getProduct(q -> getConjugate());

            // p quaternion is now [0, x', y', z']
            x = p.x;
            y = p.y;
            z = p.z;
        }

        VectorInt16 getRotated(Quaternion *q) {
            VectorInt16 r(x, y, z);
            r.rotate(q);
            return r;
        }
};

class VectorFloat {
    public:
        float x;
        float y;
        float z;

        VectorFloat() {
            x = 0;
            y = 0;
            z = 0;
        }
        
        VectorFloat(float nx, float ny, float nz) {
            x = nx;
            y = ny;
            z = nz;
        }

        float getMagnitude() {
            return sqrt(x*x + y*y + z*z);
        }

        void normalize() {
            float m = getMagnitude();
            x /= m;
            y /= m;
            z /= m;
        }
        
        VectorFloat getNormalized() {
            VectorFloat r(x, y, z);
            r.normalize();
            return r;
        }
        
        void rotate(Quaternion *q) {
            Quaternion p(0, x, y, z);

            // quaternion multiplication: q * p, stored back in p
            p = q -> getProduct(p);

            // quaternion multiplication: p * conj(q), stored back in p
            p = p.getProduct(q -> getConjugate());

            // p quaternion is now [0, x', y', z']
            x = p.x;
            y = p.y;
            z = p.z;
        }

        VectorFloat getRotated(Quaternion *q) {
            VectorFloat r(x, y, z);
            r.rotate(q);
            return r;
        }
};

#ifdef __GNUC__
#  define UNUSED __attribute__((__unused__))
#  define WEAK __attribute__((weak))
#else
#  define UNUSED
#  define WEAK
#endif

#if __GNUC__ >= 7
#  define INTENTIONAL_FALLTHRU __attribute__ ((fallthrough));
#else
#  define INTENTIONAL_FALLTHRU
#endif

#endif /* STD_H */