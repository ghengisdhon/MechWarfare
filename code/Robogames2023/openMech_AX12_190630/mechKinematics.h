#ifndef mechKine_h
#define mechKine_h

/****************************************************************************************/
/* Mech Kinematics Function Prototypes                                                  */
/****************************************************************************************/
void  Kinematics(float x0, float y0, float z0, float L0, float L1, float L2, float Phi,
                 float *X, float *Y, float *Z, float *Theta);
                 
void  InverseKinematics(float x0, float y0, float z0, float x2, float y2, float z2,
                        float L0, float L1, float l2, float phi, float *Theta);
                        
void  SingleRotation(float Yaw, float *X, float *Y, float *Z);
void  DoubleRotation(float Pitch, float Yaw, float H, float *X, float *Y, float *Z);
/****************************************************************************************/
/* Rotation about Yaw Axis                                                              */
/****************************************************************************************/
void SingleRotation(float Yaw, float *X, float *Y, float *Z) {
  int i;

  float cYaw = mechCos(Yaw);
  float sYaw = mechSin(Yaw);
  float xtemp, ytemp, ztemp;
  

  for (i = 0; i <= 3; i++) {
    xtemp = X[i];
    ytemp = Y[i];
    ztemp = Z[i];
    
    X[i] =  cYaw * xtemp + sYaw * ytemp;
    Y[i] = -sYaw * xtemp + cYaw * ytemp;
    Z[i] =  ztemp;
  }
}
/****************************************************************************************/
/* Rotation about Pitch/Yaw Axis and Height                                             */
/****************************************************************************************/
void DoubleRotation(float Pitch, float Yaw, float H, float *X, float *Y, float *Z) {
  int i;
  float cYaw   = mechCos(Yaw);
  float sYaw   = mechSin(Yaw);
  float cPitch = mechCos(Pitch);
  float sPitch = mechSin(Pitch);
  float xtemp, ytemp, ztemp;
 
  for (i = 0; i <= 3; i++) {
    xtemp = X[i];
    ytemp = Y[i];
    ztemp = Z[i];
    
    X[i] =  cPitch * cYaw * xtemp + sYaw * ytemp    + cYaw   * sPitch * ztemp;
    Y[i] = -cPitch * sYaw * xtemp + cYaw * ytemp    - sPitch * sYaw * ztemp;
    Z[i] = -sPitch * xtemp                          + cPitch * ztemp          + H;
  }
}
/****************************************************************************************/
/* Calculate Inverse Kinematics                                                         */
/****************************************************************************************/
void InverseKinematics(float x0, float y0, float z0, float x2, float y2, float z2, float L0,
                       float L1, float L2, float Phi, float *Theta) {
                        
  // Solve for Theta1
  Theta[0] = mechAtan2((y2 - y0), (x2 - x0));
  
  // Solve for Phi x,y, & z locations
  float xp  = x0 + L0*mechCos(Phi)*mechCos(Theta[0]);
  float yp  = y0 + L0*mechCos(Phi)*mechSin(Theta[0]);
  float zp  = z0 + L0*mechSin(Phi);
  
  float delx = x2-xp;
  float dely = y2-yp;
  float delz = z2-zp;

  float delxsq = delx*delx;
  float delysq = dely*dely;
  float delzsq = delz*delz;
  float delxyzsq = delxsq + delysq + delzsq;
  float delxyzsqrt = sqrt(delxyzsq);

  float L1sq = L1*L1;
  float L2sq = L2*L2;

  float Beta  = acos((L1sq+L2sq-delxyzsq)/(2*L1*L2));
  float Alpha = acos((delxyzsq+L1sq-L2sq)/(2*L1*delxyzsqrt)); 
  float Gamma = atan2(-delz,sqrt(delxsq+delysq));

  // Righty Solution
    Theta[1] = Gamma-Alpha;
    Theta[2] = PI-Beta;  
  
 //Lefty Solution
 //Theta[1] = Gamma+Alpha;
 //Theta[2] = Beta-pi;
 
}
/****************************************************************************************/
/* Calculate Kinematics                                                                 */
/****************************************************************************************/
void Kinematics(float x0, float y0, float z0, float L0, float L1, float L2, float Phi,
                float *X, float *Y, float *Z, float *Theta) {
                  
  float xp, yp, zp, x1, y1, z1, x2, y2, z2;
  float cPhi        = mechCos(Phi);
  float sPhi        = mechSin(Phi);
//  float cTheta1_0   = mechCos(Theta1[0]);
//  float sTheta1_0   = mechSin(Theta1[0]);
//  float sTheta2_0   = mechSin(Theta2[0]);
//  float cTheta2_0   = mechCos(Theta2[0]);
//  float cTheta23_0  = mechCos(Theta2[0] + Theta3[0]);
//  float sTheta23_0  = mechSin(Theta2[0] + Theta3[0]);
  float cTheta1_0   = mechCos(Theta[0]);
  float sTheta1_0   = mechSin(Theta[0]);
  float sTheta2_0   = mechSin(Theta[1]);
  float cTheta2_0   = mechCos(Theta[1]);
  float cTheta23_0  = mechCos(Theta[1] + Theta[2]);
  float sTheta23_0  = mechSin(Theta[1] + Theta[2]);
  
  xp  = x0 + L0 * cPhi * cTheta1_0;
  yp  = y0 + L0 * cPhi * sTheta1_0;
  zp  = z0 + L0 * sPhi;

  x1  = xp + L1 * cTheta2_0 * cTheta1_0;
  y1  = yp + L1 * cTheta2_0 * sTheta1_0;
  z1  = zp + L1 * sTheta2_0;

  x2  = x1 + L2 * cTheta23_0 * cTheta1_0;
  y2  = y1 + L2 * cTheta23_0 * sTheta1_0;
  z2  = z1 + L2 * sTheta23_0;

  X[0] = x0; X[1] = xp; X[2] = x1; X[3] = x2;
  Y[0] = y0; Y[1] = yp; Y[2] = y1; Y[3] = y2;
  Z[0] = z0; Z[1] = zp; Z[2] = z1; Z[3] = z2;
}
/****************************************************************************************/
/* End Mech Kineamtics Header                                                           */
/****************************************************************************************/
#endif
