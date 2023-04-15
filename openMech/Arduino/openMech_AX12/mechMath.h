#ifndef mechTrig_h
#define mechTrig_h

#include "mechLUT.h"

float sinLUT(float val);
float cosLUT(float val);
float acosLUT(float val);
float asinLUT(float val);
float sqrtNewton(const float x);
float atanApprox(float z);
float atan2Approx(float y, float x);

float lut_sinf(uint32_t phase);
/****************************************************************************************/
/* Sine LUT                                                                             */
/****************************************************************************************/

/* tweak TABLE_BITS if you need a smaller table */
#define TABLE_BITS 8
#define TABLE_SIZE (1 << TABLE_BITS)

float lut_sinf(uint32_t phase)
{
  uint32_t quad_phase;
  int table_loc;
  float x, y[2];

  /* strict aliasing blah blah blah (otherwise gcc will complain if we
   * do this by type-punning) */
  union {
    float f;
    uint32_t i;
  } out;

  /* if we're in the second or fourth quadrant, run the phase backwards */
  quad_phase = (phase & 0x40000000) ? ~phase : phase;

  /* integer part for table index */
  table_loc = (quad_phase & 0x3FFFFFFF) >> (30 - TABLE_BITS);
  y[0] = sine_table[table_loc];
  y[1] = sine_table[table_loc + 1];

  /* fractional part for linear interpolation */
  x = (quad_phase & ((1 << (30 - TABLE_BITS)) - 1)) / (float) (1 << (30 - TABLE_BITS));
  out.f = y[0] + ((y[1] - y[0]) * x);

  /* invert the second half of the wave by manually
   * setting the float's sign bit */
  out.i |= phase & 0x80000000;
  return out.f;
}
/****************************************************************************************/
/* Sin Lookup Table                                                                     */
/****************************************************************************************/
#define CHAR_BIT  (8)
//int const mask = sizeof(int) * CHAR_BIT - 1;


float sinLUT(float val){

//  int sign = 1;
////  word idx = word(R2C*val);;
//  int32_t idx = (int32_t)(R2C*val);

// // A few Different Methods to get the sign
//  sign = (val > 0) - (val < 0); // -1, 0, or +1
//
//  //Abs Value
////  idx = (val < 0) ? -(unsigned)idx : idx;  
//  int const mask = sizeof(int) * CHAR_BIT - 1;
//  idx = (idx + mask) ^ mask;

  
//  if(idx<0){
//    idx = -idx;
//    sign = -1;
//  }
//  idx &= 1023U;

  
  /* if we're in the second or fourth quadrant, run the phase backwards */
  //uint32_t idx_quad = (idx & 0x4000) ? ~idx : idx;

//  if(val<0){
//    val = -val;
//    sign = -1;
//  }
  
//  idx = word(R2C*val);

//  idx &= 1023U;
//  idx = word(R2C*val);
  
//  if(idx<0){
//    idx = -val;
//    sign = -1;
//  }
//  
//  if(idx < 256){
//    idx = idx;
//  }
//  else if (idx<512){
//    idx = 511 - idx;
//  }
//  else if (idx<768){
//    idx = idx - 512;
//    sign = -sign;
//  }
//  else{
//    idx = 1023 - idx;
//    sign = -sign;
//  }

//  int sign = 1;
//  word idx;
//
//  if(val<0){
//    val = -val;
//    sign = -1;
//  }
//  
//  idx = word(R2C*val);
//
////  while(idx>1023){
////    idx = idx -1023;
////  }
//  idx &= 1023U;

  short sign = 1;
  short idx;

  if(val<0){
    val = -val;
    sign = -1;
  }
  
  idx = short(R2C*val);
  while(idx>1023){
    idx = idx -1023;
  }
//  idx &= 1023U;
  
  float sine_val = sign * sine_table[idx];
  return sine_val;
}
/****************************************************************************************/
/* Approximate atan & atan2                                                             */
/****************************************************************************************/
// https://www.dsprelated.com/showarticle/1052.php
// Polynomial approximating arctangenet on the range -1,1.
// Max error < 0.005 (or 0.29 degrees)
float atanApprox(float z)
{
    const float n1 = 0.97239411f;
    const float n2 = -0.19194795f;
    return (n1 + n2 * z * z) * z;
}

float atan2Approx(float y, float x)
{
    if (x != 0.0f)
    {
        if (fabsf(x) > fabsf(y))
        {
            const float z = y / x;
            if (x > 0.0)
            {
                // atan2(y,x) = atan(y/x) if x > 0
                return atanApprox(z);
            }
            else if (y >= 0.0)
            {
                // atan2(y,x) = atan(y/x) + PI if x < 0, y >= 0
                return atanApprox(z) + PI;
            }
            else
            {
                // atan2(y,x) = atan(y/x) - PI if x < 0, y < 0
                return atanApprox(z) - PI;
            }
        }
        else // Use property atan(y/x) = PI/2 - atan(x/y) if |y/x| > 1.
        {
            const float z = x / y;
            if (y > 0.0)
            {
                // atan2(y,x) = PI/2 - atan(x/y) if |y/x| > 1, y > 0
                return -atanApprox(z) + HalfPI;
            }
            else
            {
                // atan2(y,x) = -PI/2 - atan(x/y) if |y/x| > 1, y < 0
                return -atanApprox(z) - HalfPI;
            }
        }
    }
    else
    {
        if (y > 0.0f) // x = 0, y > 0
        {
            return HalfPI;
        }
        else if (y < 0.0f) // x = 0, y < 0
        {
            return -HalfPI;
        }
    }
    return 0.0f; // x,y = 0. Could return NaN instead.
}
/****************************************************************************************/
/* Sqrt - Fastest but not as Precise                                                    */
/****************************************************************************************/
float sqrtNewton(const float x)  
// Reference: http://ilab.usc.edu/wiki/index.php/Fast_Square_Root
// Algorithm: Log base 2 approximation and Newton's Method
{
  union
  {
    int i;
    float x;
  } u;

  u.x = x;
  u.i = (1<<29) + (u.i >> 1) - (1<<22); 
  return u.x;
} 


/****************************************************************************************/
/* COS Lookup Table                                                                     */
/****************************************************************************************/
float cosLUT(float val){

  word idx;

  if(val<0){
    val = -val;
  }
  
  idx = word(R2C*val);
  idx = (idx + 256U)&1023U;

  float cos_val = sine_table[idx];
  return cos_val;

}
/****************************************************************************************/
/* acos Lookup Table                                                                     */
/****************************************************************************************/
float acosLUT(float val){

  word idx;

  // Make sure val is between [-1 1)
  if(val>1.0F-C2Two){
    val = 1-C2Two;
  }
  else if(val<-1.0F){
    val = -1.0F;
  }
  // Shift range between [0 2)
  val = val+1.0F;
  
  idx = word(Two2C*val);

  float acos_val = acos_table[idx];
  return acos_val;
}
/****************************************************************************************/
/* Asin Lookup Table                                                                     */
/****************************************************************************************/
float asinLUT(float val){

  word idx;

  // Make sure val is between [-1 1)
  if(val>1.0F-C2Two){
    val = 1-C2Two;
  }
  else if(val<-1.0F){
    val = -1.0F;
  }
  // Shift range between [0 2)
  val = val+1.0F;
  
  idx = word(Two2C*val);

  float acos_val = acos_table[idx];
  float asin_val = -(acos_val - HalfPI);
  
  return asin_val;
}
/****************************************************************************************/
/* End Mech Trig Header                                                                 */
/****************************************************************************************/
#endif
