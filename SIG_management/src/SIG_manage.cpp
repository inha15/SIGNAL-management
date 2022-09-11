#include <SIG_management/SIG_declare.h>

// Data Receive
void RCVD_SIG(std_msgs::Int32 SIGNAL)                                                   { SIG_MNG.set_SIGNAL((int)SIGNAL.data);}
void RCVD_fusion(const SIG_management::object_msg_arr fusion_DATA)                      { SIG_MNG.set_fusion_DATA(fusion_DATA);}
void RCVD_fusion_only_LiDAR(const SIG_management::object_msg_arr fusion_LiDAR_DATA)     { SIG_MNG.set_fusion_only_LiDAR(fusion_LiDAR_DATA); }
void RCVD_fusion_only_Camera(const std_msgs::String fusion_Camera_DATA)                 { SIG_MNG.set_fusion_only_Camera(fusion_Camera_DATA); }
void RCVD_fusion_TrafficSign(const std_msgs::String fusion_TFF_DATA)                    { SIG_MNG.set_fusion_TrafficSign(fusion_TFF_DATA); }
void RCVD_direct_LiDAR(const SIG_management::object_msg_arr LiDAR_DATA)                 { SIG_MNG.set_RCVD_direct_LiDAR(LiDAR_DATA); }

//Mission Filter
void AEB_Filter()                   { SIG_MNG.direct_LiDAR_ROI_ReSetting(AEB_Filter_ROI_min, AEB_Filter_ROI_max); }
void Parking_Filter()               { SIG_MNG.direct_LiDAR_ROI_ReSetting(Parking_Filter_ROI_min, Parking_Filter_ROI_max); }
void SmallStaticObstacle_Filter()   { SIG_MNG.direct_LiDAR_ROI_ReSetting(SmallStaticObstacle_Filter_ROI_min, SmallStaticObstacle_Filter_ROI_max); }
void LargeStaticObstacle_Filter()   { SIG_MNG.direct_LiDAR_ROI_ReSetting(LargeStaticObstacle_Filter_ROI_min, LargeStaticObstacle_Filter_ROI_max); }
void Deliv_Filter()                 { SIG_MNG.fusion_DATA_ROI_ReSetting(Deliv_Filter_ROI_min, Deliv_Filter_ROI_max); }
void Deliv_wall_remove()            { SIG_MNG.fusion_OBJ_remove(SignBoard_H, SignBoard_W); }
void Advanced_jiwonFilter_LargeStaticObstacle(){

}


void CheckMission(){
    int cur_SIG = SIG_MNG.get_SIGNAL();
    if(cur_SIG == TRI_DRIVING || cur_SIG == FIN_DRIVING)            return;
    if(cur_SIG == DIAGONAL_PARKING || cur_SIG == PARALLEL_PARKING)  Parking_Filter();
    if(cur_SIG == STATIC_OBSTACLE_SMALL)                            SmallStaticObstacle_Filter();
    if(cur_SIG == STATIC_OBSTACLE_LARGE)                            LargeStaticObstacle_Filter();
    if(cur_SIG == AEB)                                              AEB_Filter();
    if(cur_SIG == DELIV_FISRT || cur_SIG == DELIV_SECOND)           { Deliv_Filter(); Deliv_wall_remove(); }
}

void DeliveryMission_toLiDAR_ROI(){ 
    SIG_management::lidar_signal tmp_ROI;
    tmp_ROI.xMin = get<0>(Deliv_ROImin_toLiDAR);
    tmp_ROI.yMin = get<1>(Deliv_ROImin_toLiDAR);
    tmp_ROI.zMin = get<2>(Deliv_ROImin_toLiDAR);
    tmp_ROI.xMax = get<0>(Deliv_ROImax_toLiDAR);
    tmp_ROI.yMax = get<1>(Deliv_ROImax_toLiDAR);
    tmp_ROI.zMax = get<2>(Deliv_ROImax_toLiDAR);
    pub_toLiDAR_ROI.publish(tmp_ROI);
}

void Ordinary_toLiDAR_ROI(){
    SIG_management::lidar_signal tmp_ROI;
    tmp_ROI.xMin = get<0>(Ordinary_ROImin_toLiDAR);
    tmp_ROI.yMin = get<1>(Ordinary_ROImin_toLiDAR);
    tmp_ROI.zMin = get<2>(Ordinary_ROImin_toLiDAR);
    tmp_ROI.xMax = get<0>(Ordinary_ROImax_toLiDAR);
    tmp_ROI.yMax = get<1>(Ordinary_ROImax_toLiDAR);
    tmp_ROI.zMax = get<2>(Ordinary_ROImax_toLiDAR);
    pub_toLiDAR_ROI.publish(tmp_ROI);
}

void SEND_DATA(){
    if(SIG_state.fusion == 1)           pub_Fusion_DATA.publish(SIG_MNG.get_fusion_DATA());
    if(SIG_state.fusion_LiDAR == 1)     pub_Fusion_LiDAR_DATA.publish(SIG_MNG.get_fusion_LiDAR_DATA());
    if(SIG_state.fusion_Camera == 1)    pub_Fusion_Camera_DATA.publish(SIG_MNG.get_fusion_Camera_DATA());
    if(SIG_state.fusion_TFF == 1)       pub_Fusion_TFFsign_DATA.publish(SIG_MNG.get_fusion_TFF_DATA());
    if(SIG_state.direct_LiDAR == 1)     pub_LiDAR_DATA.publish(SIG_MNG.get_LiDAR_DATA());

    //pub mission name
    std_msgs::String tmp;
    tmp.data = SIG_MNG.get_mission_name();
    pub_Mission_name.publish(tmp); //mission name publish

    //pub LiDAR ROI
    if(SIG_MNG.get_SIGNAL() == DELIV_FISRT || SIG_MNG.get_SIGNAL() == DELIV_SECOND) DeliveryMission_toLiDAR_ROI();
    else Ordinary_toLiDAR_ROI();

}

void fin_process(){
    cout << "current mission : " << SIG_MNG.get_mission_name() << endl;
    SIG_MNG.set_SIG_state(SIG_MNG.get_SIGNAL());
    if(Filter_switch) CheckMission();  //check mission & apply mission filter
    SIG_state.print();
    SEND_DATA();
    //SIG_MNG.Print_Fusion_DATA();
    //SIG_MNG.clear();
}
