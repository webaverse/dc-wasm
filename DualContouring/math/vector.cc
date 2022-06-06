#include "vector.h"

Vec _x;
Vec _y;
Vec _z;

Matrix::Matrix(const Vec &position, const Quat &quaternion, const Vec &scale) {
  float *te = this->elements;

  const float x = quaternion.x, y = quaternion.y, z = quaternion.z, w = quaternion.w;
  const float x2 = x + x, y2 = y + y, z2 = z + z;
  const float xx = x * x2, xy = x * y2, xz = x * z2;
  const float yy = y * y2, yz = y * z2, zz = z * z2;
  const float wx = w * x2, wy = w * y2, wz = w * z2;

  const float sx = scale.x, sy = scale.y, sz = scale.z;

  te[ 0 ] = ( 1.0f - ( yy + zz ) ) * sx;
  te[ 1 ] = ( xy + wz ) * sx;
  te[ 2 ] = ( xz - wy ) * sx;
  te[ 3 ] = 0.0f;

  te[ 4 ] = ( xy - wz ) * sy;
  te[ 5 ] = ( 1 - ( xx + zz ) ) * sy;
  te[ 6 ] = ( yz + wx ) * sy;
  te[ 7 ] = 0.0f;

  te[ 8 ] = ( xz + wy ) * sz;
  te[ 9 ] = ( yz - wx ) * sz;
  te[ 10 ] = ( 1 - ( xx + yy ) ) * sz;
  te[ 11 ] = 0.0f;

  te[ 12 ] = position.x;
  te[ 13 ] = position.y;
  te[ 14 ] = position.z;
  te[ 15 ] = 1.0f;
}

Matrix &Matrix::compose(const Vec &position, const Quat &quaternion, const Vec &scale) {
  float *te = this->elements;

  const float x = quaternion.x, y = quaternion.y, z = quaternion.z, w = quaternion.w;
  const float x2 = x + x,	y2 = y + y, z2 = z + z;
  const float xx = x * x2, xy = x * y2, xz = x * z2;
  const float yy = y * y2, yz = y * z2, zz = z * z2;
  const float wx = w * x2, wy = w * y2, wz = w * z2;

  const float sx = scale.x, sy = scale.y, sz = scale.z;

  te[ 0 ] = ( 1.0f - ( yy + zz ) ) * sx;
  te[ 1 ] = ( xy + wz ) * sx;
  te[ 2 ] = ( xz - wy ) * sx;
  te[ 3 ] = 0.0f;

  te[ 4 ] = ( xy - wz ) * sy;
  te[ 5 ] = ( 1.0f - ( xx + zz ) ) * sy;
  te[ 6 ] = ( yz + wx ) * sy;
  te[ 7 ] = 0.0f;

  te[ 8 ] = ( xz + wy ) * sz;
  te[ 9 ] = ( yz - wx ) * sz;
  te[ 10 ] = ( 1.0f - ( xx + yy ) ) * sz;
  te[ 11 ] = 0.0f;

  te[ 12 ] = position.x;
  te[ 13 ] = position.y;
  te[ 14 ] = position.z;
  te[ 15 ] = 1.0f;

  return *this;
}

Matrix &Matrix::decompose(Vec &position, Quat &quaternion, Vec &scale) {
  const float *te = elements;

  float sx = Vec( te[ 0 ], te[ 1 ], te[ 2 ] ).magnitude();
  float sy = Vec( te[ 4 ], te[ 5 ], te[ 6 ] ).magnitude();
  float sz = Vec( te[ 8 ], te[ 9 ], te[ 10 ] ).magnitude();

  // if determine is negative, we need to invert one scale
  const float det = determinant();
  if ( det < 0 ) sx = - sx;

  position.x = te[ 12 ];
  position.y = te[ 13 ];
  position.z = te[ 14 ];

  // scale the rotation part
  Matrix m(*this);

  const float invSX = 1.0f / sx;
  const float invSY = 1.0f / sy;
  const float invSZ = 1.0f / sz;

  m.elements[ 0 ] *= invSX;
  m.elements[ 1 ] *= invSX;
  m.elements[ 2 ] *= invSX;

  m.elements[ 4 ] *= invSY;
  m.elements[ 5 ] *= invSY;
  m.elements[ 6 ] *= invSY;

  m.elements[ 8 ] *= invSZ;
  m.elements[ 9 ] *= invSZ;
  m.elements[ 10 ] *= invSZ;

  quaternion.setFromRotationMatrix(m);

  scale.x = sx;
  scale.y = sy;
  scale.z = sz;

  return *this;
};

void Matrix::lookAt(Vec eye, Vec target, Vec up) {
  float *te = this->elements;
  _z = eye - target;
  if(_z.magnitude_sqr() == 0) {
    _z.z = 1;
  }
  _z.normalize();
  _x = up ^ _z;
  if(_x.magnitude_sqr() == 0) {
    if(abs(up.z) == 1) {
      _z.x += 0.0001;
    } else {
      _z.z += 0.0001;
    }
    _z.normalize();
    _x = up ^ _z;
  }
  _x.normalize();
  _y = _z ^ _x;

  te[ 0 ] = _x.x; te[ 4 ] = _y.x; te[ 8 ] = _z.x;
  te[ 1 ] = _x.y; te[ 5 ] = _y.y; te[ 9 ] = _z.y;
  te[ 2 ] = _x.z; te[ 6 ] = _y.z; te[ 10 ] = _z.z;
}

void Matrix::invert() {
  // based on http://www.euclideanspace.com/maths/algebra/matrix/functions/inverse/fourD/index.htm
  float *te = this->elements,

    n11 = te[ 0 ], n21 = te[ 1 ], n31 = te[ 2 ], n41 = te[ 3 ],
    n12 = te[ 4 ], n22 = te[ 5 ], n32 = te[ 6 ], n42 = te[ 7 ],
    n13 = te[ 8 ], n23 = te[ 9 ], n33 = te[ 10 ], n43 = te[ 11 ],
    n14 = te[ 12 ], n24 = te[ 13 ], n34 = te[ 14 ], n44 = te[ 15 ],

    t11 = n23 * n34 * n42 - n24 * n33 * n42 + n24 * n32 * n43 - n22 * n34 * n43 - n23 * n32 * n44 + n22 * n33 * n44,
    t12 = n14 * n33 * n42 - n13 * n34 * n42 - n14 * n32 * n43 + n12 * n34 * n43 + n13 * n32 * n44 - n12 * n33 * n44,
    t13 = n13 * n24 * n42 - n14 * n23 * n42 + n14 * n22 * n43 - n12 * n24 * n43 - n13 * n22 * n44 + n12 * n23 * n44,
    t14 = n14 * n23 * n32 - n13 * n24 * n32 - n14 * n22 * n33 + n12 * n24 * n33 + n13 * n22 * n34 - n12 * n23 * n34;

  float det = n11 * t11 + n21 * t12 + n31 * t13 + n41 * t14;

  if ( det == 0.f ) {
    std::fill(this->elements, this->elements + 16, 0.f);
    return;
  }

  float detInv = 1.f / det;

  te[ 0 ] = t11 * detInv;
  te[ 1 ] = ( n24 * n33 * n41 - n23 * n34 * n41 - n24 * n31 * n43 + n21 * n34 * n43 + n23 * n31 * n44 - n21 * n33 * n44 ) * detInv;
  te[ 2 ] = ( n22 * n34 * n41 - n24 * n32 * n41 + n24 * n31 * n42 - n21 * n34 * n42 - n22 * n31 * n44 + n21 * n32 * n44 ) * detInv;
  te[ 3 ] = ( n23 * n32 * n41 - n22 * n33 * n41 - n23 * n31 * n42 + n21 * n33 * n42 + n22 * n31 * n43 - n21 * n32 * n43 ) * detInv;

  te[ 4 ] = t12 * detInv;
  te[ 5 ] = ( n13 * n34 * n41 - n14 * n33 * n41 + n14 * n31 * n43 - n11 * n34 * n43 - n13 * n31 * n44 + n11 * n33 * n44 ) * detInv;
  te[ 6 ] = ( n14 * n32 * n41 - n12 * n34 * n41 - n14 * n31 * n42 + n11 * n34 * n42 + n12 * n31 * n44 - n11 * n32 * n44 ) * detInv;
  te[ 7 ] = ( n12 * n33 * n41 - n13 * n32 * n41 + n13 * n31 * n42 - n11 * n33 * n42 - n12 * n31 * n43 + n11 * n32 * n43 ) * detInv;

  te[ 8 ] = t13 * detInv;
  te[ 9 ] = ( n14 * n23 * n41 - n13 * n24 * n41 - n14 * n21 * n43 + n11 * n24 * n43 + n13 * n21 * n44 - n11 * n23 * n44 ) * detInv;
  te[ 10 ] = ( n12 * n24 * n41 - n14 * n22 * n41 + n14 * n21 * n42 - n11 * n24 * n42 - n12 * n21 * n44 + n11 * n22 * n44 ) * detInv;
  te[ 11 ] = ( n13 * n22 * n41 - n12 * n23 * n41 - n13 * n21 * n42 + n11 * n23 * n42 + n12 * n21 * n43 - n11 * n22 * n43 ) * detInv;

  te[ 12 ] = t14 * detInv;
  te[ 13 ] = ( n13 * n24 * n31 - n14 * n23 * n31 + n14 * n21 * n33 - n11 * n24 * n33 - n13 * n21 * n34 + n11 * n23 * n34 ) * detInv;
  te[ 14 ] = ( n14 * n22 * n31 - n12 * n24 * n31 - n14 * n21 * n32 + n11 * n24 * n32 + n12 * n21 * n34 - n11 * n22 * n34 ) * detInv;
  te[ 15 ] = ( n12 * n23 * n31 - n13 * n22 * n31 + n13 * n21 * n32 - n11 * n23 * n32 - n12 * n21 * n33 + n11 * n22 * n33 ) * detInv;
}

void Matrix::setPosition(Vec position) {
  float *te = this->elements;
  
  te[ 12 ] = position.x;
  te[ 13 ] = position.y;
  te[ 14 ] = position.z;
}

Vec &Vec::applyQuaternion(const Quat &q) {
  // const float x = this->x, y = this->y, z = this->z;
  const float qx = q.x, qy = q.y, qz = q.z, qw = q.w;

  // calculate quat * vector

  const float ix = qw * x + qy * z - qz * y;
  const float iy = qw * y + qz * x - qx * z;
  const float iz = qw * z + qx * y - qy * x;
  const float iw = - qx * x - qy * y - qz * z;

  // calculate result * inverse quat

  x = ix * qw + iw * - qx + iy * - qz - iz * - qy;
  y = iy * qw + iw * - qy + iz * - qx - ix * - qz;
  z = iz * qw + iw * - qz + ix * - qy - iy * - qx;

  return *this;
}