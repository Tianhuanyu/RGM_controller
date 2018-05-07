//lines per round
const double RGM_LPR = 524288.0;
const double position_calibrations[6] = {RGM_LPR, RGM_LPR-43690, -142000+RGM_LPR, 217700, RGM_LPR-36410, 0}; // to_driver_count = phy_count + cali


int KDL::RGMctrl::pos_trans(const int& p1,const int& p2,const int& p3,
                const int& p4,const int& p5,const int& p6,JntArray& q_out) 
                {
                    if(q_out.rows()!=6)
                        return(error = E_SIZE_MISMATCH);

                    q_out(0) = fmod(((p1-position_calibrations[0])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;
                    q_out(1) = fmod(((p2-position_calibrations[1])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;
                    q_out(2) = fmod(((p3-position_calibrations[2])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;
                    q_out(3) = fmod(((p4-position_calibrations[3])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;
                    q_out(4) = fmod(((p5-position_calibrations[4])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;
                    q_out(5) = fmod(((p6-position_calibrations[5])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;

                    return 0;

                }
                //((value-position_calibrations[no])/lines_per_round*(2*math.pi)+math.pi)%(2*math.pi)-math.pi

int KDL::RGMctrl::pos_trans(const JntArray& q_in,int& p1,int& p2,int& p3,
                                                int& p4,int& p5,int& p6)
                {
                    if(q_in.rows()!=6)
                        return(error = E_SIZE_MISMATCH);

                    p1 = int( RGM_LPR*q_in(0)/(2*M_PI) + int(position_calibrations[0])%int(RGM_LPR));
                    p2 = int( RGM_LPR*q_in(1)/(2*M_PI) + int(position_calibrations[1])%int(RGM_LPR));
                    p3 = int( RGM_LPR*q_in(2)/(2*M_PI) + int(position_calibrations[2])%int(RGM_LPR));
                    p4 = int( RGM_LPR*q_in(3)/(2*M_PI) + int(position_calibrations[3])%int(RGM_LPR));
                    p5 = int( RGM_LPR*q_in(4)/(2*M_PI) + int(position_calibrations[4])%int(RGM_LPR));
                    p6 = int( RGM_LPR*q_in(5)/(2*M_PI) + int(position_calibrations[5])%int(RGM_LPR));
                   
                   return 0;
                        
                }

int KDL::RGMctrl::vel_trans(const JntArray& qdot_in,int& qd1,int& qd2,int& qd3,
                                                int& qd4,int& qd5,int& qd6)
                {
                    if(qdot_in.rows()!=6)
                        return(error = E_SIZE_MISMATCH);
                        
                        qd1 = qdot_in(0)*RGM_LPR;
                        qd2 = qdot_in(1)*RGM_LPR;
                        qd3 = qdot_in(2)*RGM_LPR;
                        qd4 = qdot_in(3)*RGM_LPR;
                        qd5 = qdot_in(4)*RGM_LPR;
                        qd6 = qdot_in(5)*RGM_LPR;    
                 
                   return 0;
                        
                }