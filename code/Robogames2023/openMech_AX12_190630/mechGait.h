#ifndef mechGait_h
#define mechGait_h

/****************************************************************************************/
/* Mech Gait Prototypes                                                                 */
/****************************************************************************************/
void  GaitTx();
void initX430();
/****************************************************************************************/
/* Mech Gait Calcultion Function                                                        */
/****************************************************************************************/
void Gait() {
  float fdelx1, fdelx2, fdelx3, fdelx4;
  float fdely1, fdely2, fdely3, fdely4;
  float fdelz1, fdelz2, fdelz3, fdelz4;

  fdelx1 = delxyLF_vec[mechGV.gaitIdx] * mechGV.gaitDelx;
  fdelx2 = delxyLR_vec[mechGV.gaitIdx] * mechGV.gaitDelx;
  fdelx3 = delxyRR_vec[mechGV.gaitIdx] * mechGV.gaitDelx;
  fdelx4 = delxyRF_vec[mechGV.gaitIdx] * mechGV.gaitDelx;

  fdely1 = delxyLF_vec[mechGV.gaitIdx] * mechGV.gaitDely;
  fdely2 = delxyLR_vec[mechGV.gaitIdx] * mechGV.gaitDely;
  fdely3 = delxyRR_vec[mechGV.gaitIdx] * mechGV.gaitDely;
  fdely4 = delxyRF_vec[mechGV.gaitIdx] * mechGV.gaitDely;

  fdelz1 =  delzLF_vec[mechGV.gaitIdx] * mechGV.gaitStepHeight;
  fdelz2 =  delzLR_vec[mechGV.gaitIdx] * mechGV.gaitStepHeight;
  fdelz3 =  delzRR_vec[mechGV.gaitIdx] * mechGV.gaitStepHeight;
  fdelz4 =  delzRF_vec[mechGV.gaitIdx] * mechGV.gaitStepHeight;

  //******************************************************************************************
  // Calculate Inverse Kinematics
  //******************************************************************************************
  // Foot Step Locations, Leg 1
  float Xft1[4] = {XFT[0] + fdelx1, XFT[1] + fdelx2, XFT[2] + fdelx3, XFT[3] + fdelx4};
  float Yft1[4] = {YFT[0] + fdely1, YFT[1] + fdely2, YFT[2] + fdely3, YFT[3] + fdely4};
  float Zft1[4] = {ZFT[0] + fdelz1, ZFT[1] + fdelz2, ZFT[2] + fdelz3, ZFT[3] + fdelz4};

  // Foot Step Locations, Leg 2
  float Xft2[4] = {XFT[0] + fdelx1, XFT[1] + fdelx2, XFT[2] + fdelx3, XFT[3] + fdelx4};
  float Yft2[4] = {YFT[0] + fdely1, YFT[1] + fdely2, YFT[2] + fdely3, YFT[3] + fdely4};
  float Zft2[4] = {ZFT[0] + fdelz1, ZFT[1] + fdelz2, ZFT[2] + fdelz3, ZFT[3] + fdelz4};

  // Foot Step Locations, Leg 3
  float Xft3[4] = {XFT[0] + fdelx1, XFT[1] + fdelx2, XFT[2] + fdelx3, XFT[3] + fdelx4};
  float Yft3[4] = {YFT[0] + fdely1, YFT[1] + fdely2, YFT[2] + fdely3, YFT[3] + fdely4};
  float Zft3[4] = {ZFT[0] + fdelz1, ZFT[1] + fdelz2, ZFT[2] + fdelz3, ZFT[3] + fdelz4};

  // Foot Step Locations, Leg 4
  float Xft4[4] = {XFT[0] + fdelx1, XFT[1] + fdelx2, XFT[2] + fdelx3, XFT[3] + fdelx4};
  float Yft4[4] = {YFT[0] + fdely1, YFT[1] + fdely2, YFT[2] + fdely3, YFT[3] + fdely4};
  float Zft4[4] = {ZFT[0] + fdelz1, ZFT[1] + fdelz2, ZFT[2] + fdelz3, ZFT[3] + fdelz4};

  SingleRotation(delyawLF_vec[mechGV.gaitIdx]*mechGV.gaitDelyaw, &Xft1[0], &Yft1[0], &Zft1[0]);
  SingleRotation(delyawLR_vec[mechGV.gaitIdx]*mechGV.gaitDelyaw, &Xft2[0], &Yft2[0], &Zft2[0]);
  SingleRotation(delyawRR_vec[mechGV.gaitIdx]*mechGV.gaitDelyaw, &Xft3[0], &Yft3[0], &Zft3[0]);
  SingleRotation(delyawRF_vec[mechGV.gaitIdx]*mechGV.gaitDelyaw, &Xft4[0], &Yft4[0], &Zft4[0]);

  // Update Foot Locations
  float Xft[4], Yft[4], Zft[4];
  Xft[0] = Xft1[0];
  Xft[1] = Xft2[1];
  Xft[2] = Xft3[2];
  Xft[3] = Xft4[3];

  Yft[0] = Yft1[0];
  Yft[1] = Yft2[1];
  Yft[2] = Yft3[2];
  Yft[3] = Yft4[3];

  Zft[0] = fdelz1;
  Zft[1] = fdelz2;
  Zft[2] = fdelz3;
  Zft[3] = fdelz4;

  // Shoulder Rotations
  float Xsh[4] = {XSH[0], XSH[1], XSH[2], XSH[3]};
  float Ysh[4] = {YSH[0], YSH[1], YSH[2], YSH[3]};
  float Zsh[4] = {ZSH[0], ZSH[1], ZSH[2], ZSH[3]};
  DoubleRotation(mechGV.bodyPitch, mechGV.bodyYaw, mechGV.bodyHeight + delzadj_vec[mechGV.gaitIdx]*mechGV.bodyHeightHop, &Xsh[0], &Ysh[0], &Zsh[0]);

  // Inverse Kinematic Angles
  InverseKinematics(Xsh[0], Ysh[0], Zsh[0], Xft[0], Yft[0], Zft[0], L0F, L1F, L2F, PhiF, &mechGV.fTheta0[0]);
  mechGV.uTheta0[0] = (unsigned int)(DxlNull + ((mechGV.fTheta0[0] * R2D + offsetT00) * D2C));
  mechGV.uTheta0[1] = (unsigned int)(DxlNull + ( mechGV.fTheta0[1] * R2D + offsetTF1 ) * D2C);
  mechGV.uTheta0[2] = (unsigned int)(DxlNull + ( mechGV.fTheta0[2] * R2D + offsetTF2 + offsetT02) * D2C);
  
//  uTheta0_AX12[0] = (unsigned int)(DxlNull_AX12 + ((mechGV.fTheta0[0] * R2D + offsetT00) * D2C_AX12));
//  uTheta0_AX12[1] = (unsigned int)(DxlNull_AX12 + ( mechGV.fTheta0[1] * R2D + offsetTF1 ) * D2C_AX12);
//  uTheta0_AX12[2] = (unsigned int)(DxlNull_AX12 + ( mechGV.fTheta0[2] * R2D + offsetTF2 + offsetT02) * D2C_AX12);
  
  // Check Angle Limits
  if      (mechGV.uTheta0[0] > Joint1CCWlimF) {
    mechGV.uTheta0[0] = Joint1CCWlimF;
  }
  else if (mechGV.uTheta0[0] < Joint1CWlimF) {
    mechGV.uTheta0[0]  = Joint1CWlimF;
  }
  if      (mechGV.uTheta0[1] > Joint2CCWlimF) {
    mechGV.uTheta0[1] = Joint2CCWlimF;
  }
  else if (mechGV.uTheta0[1] < Joint2CWlimF) {
    mechGV.uTheta0[1]  = Joint2CWlimF;
  }
  if      (mechGV.uTheta0[2] > Joint3CCWlimF) {
    mechGV.uTheta0[2] = Joint3CCWlimF;
  }
  else if (mechGV.uTheta0[2] < Joint3CWlimF) {
    mechGV.uTheta0[2]  = Joint3CWlimF;
  }

  InverseKinematics(Xsh[1], Ysh[1], Zsh[1], Xft[1], Yft[1], Zft[1], L0R, L1R, L2R, PhiR, &mechGV.fTheta1[0]);
  mechGV.uTheta1[0] = (unsigned int)(DxlNull + (mechGV.fTheta1[0] * R2D + offsetT10) * D2C);
  mechGV.uTheta1[1] = (unsigned int)(DxlNull + (mechGV.fTheta1[1] * R2D + offsetTR1 ) * D2C);
  mechGV.uTheta1[2] = (unsigned int)(DxlNull + (mechGV.fTheta1[2] * R2D + offsetTR2 + offsetT12) * D2C);

//  uTheta1_AX12[0] = (unsigned int)(DxlNull_AX12 + (mechGV.fTheta1[0] * R2D + offsetT10) * D2C_AX12);
//  uTheta1_AX12[1] = (unsigned int)(DxlNull_AX12 + (mechGV.fTheta1[1] * R2D + offsetTR1 ) * D2C_AX12);
//  uTheta1_AX12[2] = (unsigned int)(DxlNull_AX12 + (mechGV.fTheta1[2] * R2D + offsetTR2 + offsetT12) * D2C_AX12);
    
  // Check Angle Limits
  if      (mechGV.uTheta1[0] > Joint1CCWlimR) {
    mechGV.uTheta1[0] = Joint1CCWlimR;
  }
  else if (mechGV.uTheta1[0] < Joint1CWlimR) {
    mechGV.uTheta1[0]  = Joint1CWlimR;
  }
  if      (mechGV.uTheta1[1] > Joint2CCWlimR) {
    mechGV.uTheta1[1] = Joint2CCWlimR;
  }
  else if (mechGV.uTheta1[1] < Joint2CWlimR) {
    mechGV.uTheta1[1]  = Joint2CWlimR;
  }
  if      (mechGV.uTheta1[2] > Joint3CCWlimR) {
    mechGV.uTheta1[2] = Joint3CCWlimR;
  }
  else if (mechGV.uTheta1[2] < Joint3CWlimR) {
    mechGV.uTheta1[2]  = Joint3CWlimR;
  }

  InverseKinematics(Xsh[2], Ysh[2], Zsh[2], Xft[2], Yft[2], Zft[2], L0R, L1R, L2R, PhiR, &mechGV.fTheta2[0]);
  mechGV.uTheta2[0] = (unsigned int)(DxlNull + (mechGV.fTheta2[0] * R2D + offsetT20) * D2C);
  mechGV.uTheta2[1] = (unsigned int)(DxlNull + (mechGV.fTheta2[1] * R2D + offsetTR1 ) * D2C);
  mechGV.uTheta2[2] = (unsigned int)(DxlNull + (mechGV.fTheta2[2] * R2D + offsetTR2 + offsetT22) * D2C);

//  uTheta2_AX12[0] = (unsigned int)(DxlNull_AX12 + (mechGV.fTheta2[0] * R2D + offsetT20) * D2C_AX12);
//  uTheta2_AX12[1] = (unsigned int)(DxlNull_AX12 + (mechGV.fTheta2[1] * R2D + offsetTR1 ) * D2C_AX12);
//  uTheta2_AX12[2] = (unsigned int)(DxlNull_AX12 + (mechGV.fTheta2[2] * R2D + offsetTR2 + offsetT22) * D2C_AX12);

    
  // Check Angle Limits
  if      (mechGV.uTheta2[0] > Joint1CCWlimR) {
    mechGV.uTheta2[0] = Joint1CCWlimR;
  }
  else if (mechGV.uTheta2[0] < Joint1CWlimR) {
    mechGV.uTheta2[0]  = Joint1CWlimR;
  }
  if      (mechGV.uTheta2[1] > Joint2CCWlimR) {
    mechGV.uTheta2[1] = Joint2CCWlimR;
  }
  else if (mechGV.uTheta2[1] < Joint2CWlimR) {
    mechGV.uTheta2[1]  = Joint2CWlimR;
  }
  if      (mechGV.uTheta2[2] > Joint3CCWlimR) {
    mechGV.uTheta2[2] = Joint3CCWlimR;
  }
  else if (mechGV.uTheta2[2] < Joint3CWlimR) {
    mechGV.uTheta2[2]  = Joint3CWlimR;
  }

  InverseKinematics(Xsh[3], Ysh[3], Zsh[3], Xft[3], Yft[3], Zft[3], L0F, L1F, L2F, PhiF, &mechGV.fTheta3[0]);
  mechGV.uTheta3[0] = (unsigned)(DxlNull + (mechGV.fTheta3[0] * R2D + offsetT30) * D2C);
  mechGV.uTheta3[1] = (unsigned)(DxlNull + (mechGV.fTheta3[1] * R2D + offsetTF1 ) * D2C);
  mechGV.uTheta3[2] = (unsigned)(DxlNull + (mechGV.fTheta3[2] * R2D + offsetTF2 + offsetT32) * D2C);

//  uTheta3_AX12[0] = (unsigned)(DxlNull_AX12 + (mechGV.fTheta3[0] * R2D + offsetT30) * D2C_AX12);
//  uTheta3_AX12[1] = (unsigned)(DxlNull_AX12 + (mechGV.fTheta3[1] * R2D + offsetTF1 ) * D2C_AX12);
//  uTheta3_AX12[2] = (unsigned)(DxlNull_AX12 + (mechGV.fTheta3[2] * R2D + offsetTF2 + offsetT32) * D2C_AX12);
    
  // Check Angle Limits
  if      (mechGV.uTheta3[0] > Joint1CCWlimF) {
    mechGV.uTheta3[0] = Joint1CCWlimF;
  }
  else if (mechGV.uTheta3[0] < Joint1CWlimF) {
    mechGV.uTheta3[0]  = Joint1CWlimF;
  }
  if      (mechGV.uTheta3[1] > Joint2CCWlimF) {
    mechGV.uTheta3[1] = Joint2CCWlimF;
  }
  else if (mechGV.uTheta3[1] < Joint2CWlimF) {
    mechGV.uTheta3[1]  = Joint2CWlimF;
  }
  if      (mechGV.uTheta3[2] > Joint3CCWlimF) {
    mechGV.uTheta3[2] = Joint3CCWlimF;
  }
  else if (mechGV.uTheta3[2] < Joint3CWlimF) {
    mechGV.uTheta3[2]  = Joint3CWlimF;
  }

  if ( (mechGV.gaitIdx != 0) && (mechGV.gaitIdx != mechGV.gaitVectorSize / 2) ) {
  }
  else {
    if (mechGV.gaitDelx == 0 && mechGV.gaitDely == 0 && mechGV.gaitDelyaw == 0) {
      mechGV.gaitIdx = mechGV.gaitIdx - mechGV.gaitStepIncrement;
    }
  }

  mechGV.gaitIdx = mechGV.gaitIdx + mechGV.gaitStepIncrement;
  if (mechGV.gaitIdx > mechGV.gaitVectorSize - mechGV.gaitStepIncrement) {
    mechGV.gaitIdx = 0;
  }
}

/****************************************************************************************/
/* x430 Initialization Function                                                         */
/****************************************************************************************/
void initX430(){    

    X430.StatusReturnLevel(L11,1);
    X430.StatusReturnLevel(L12,1);
    X430.StatusReturnLevel(L13,1);
    X430.StatusReturnLevel(L21,1);
    X430.StatusReturnLevel(L22,1);
    X430.StatusReturnLevel(L23,1);
    X430.StatusReturnLevel(L31,1);
    X430.StatusReturnLevel(L32,1);
    X430.StatusReturnLevel(L33,1);
    X430.StatusReturnLevel(L41,1);
    X430.StatusReturnLevel(L42,1);
    X430.StatusReturnLevel(L43,1);
    
    delay(10);
    X430.TorqueEnable(L11,1);
    delay(10);
    X430.TorqueEnable(L12,1);
    delay(10);
    X430.TorqueEnable(L13,1);
    delay(10);
    X430.TorqueEnable(L21,1);
    delay(10);
    X430.TorqueEnable(L22,1);
    delay(10);
    X430.TorqueEnable(L23,1);
    delay(10);
    X430.TorqueEnable(L31,1);
    delay(10);
    X430.TorqueEnable(L32,1);
    delay(10);
    X430.TorqueEnable(L33,1);
    delay(10);
    X430.TorqueEnable(L41,1);
    delay(10);
    X430.TorqueEnable(L42,1);
    delay(10);
    X430.TorqueEnable(L43,1);
    delay(250);

    GaitTx();

    // Goal Velocity
//    X430.writeWord(L11,112,0);
//    X430.writeWord(L12,112,0);
//    X430.writeWord(L13,112,0);
//    X430.writeWord(L21,112,0);
//    X430.writeWord(L22,112,0);
//    X430.writeWord(L23,112,0);
//    X430.writeWord(L31,112,0);
//    X430.writeWord(L32,112,0);
//    X430.writeWord(L33,112,0);
//    X430.writeWord(L41,112,0);
//    X430.writeWord(L42,112,0);
//    X430.writeWord(L43,112,0);
//    
//    X430.writeWord(L11,114,5);
//    X430.writeWord(L12,114,5);
//    X430.writeWord(L13,114,5);
//    X430.writeWord(L21,114,5);
//    X430.writeWord(L22,114,5);
//    X430.writeWord(L23,114,5);
//    X430.writeWord(L31,114,5);
//    X430.writeWord(L32,114,5);
//    X430.writeWord(L33,114,5);
//    X430.writeWord(L41,114,5);
//    X430.writeWord(L42,114,5);
//    X430.writeWord(L43,114,5);
    
    X430.GoalPosition(L11,mechGV.uTheta0[0]);
    X430.GoalPosition(L21,mechGV.uTheta1[0]);
    X430.GoalPosition(L31,mechGV.uTheta2[0]);
    X430.GoalPosition(L41,mechGV.uTheta3[0]);
    delay(1000);

    X430.GoalPosition(L12,mechGV.uTheta0[1]);
    X430.GoalPosition(L22,mechGV.uTheta1[1]);
    X430.GoalPosition(L32,mechGV.uTheta2[1]);
    X430.GoalPosition(L42,mechGV.uTheta3[1]);
    delay(1000);

    X430.GoalPosition(L13,mechGV.uTheta0[2]);
    X430.GoalPosition(L23,mechGV.uTheta1[2]);
    X430.GoalPosition(L33,mechGV.uTheta2[2]);
    X430.GoalPosition(L43,mechGV.uTheta3[2]);
    delay(1000);
}
/****************************************************************************************/
/* End Mech Gait Header                                                                 */
/****************************************************************************************/
#endif
