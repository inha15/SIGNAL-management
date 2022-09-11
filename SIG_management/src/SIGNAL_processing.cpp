#include <SIG_management/SIG_declare.h>



int main(int argc, char** argv){
    ros::init(argc, argv, "SIGNAL_MANAGEMENT_NODE"); //node name 
	ros::NodeHandle nh;         //nodehandle

    //FUSION param
    nh.getParam("SIGNAL_manage_node/Filter_switch",                         Filter_switch);

    nh.getParam("SIGNAL_manage_node/AEB_Filter_xmin",                       get<0>(AEB_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/AEB_Filter_ymin",                       get<1>(AEB_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/AEB_Filter_zmin",                       get<2>(AEB_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/AEB_Filter_xmax",                       get<0>(AEB_Filter_ROI_max));
    nh.getParam("SIGNAL_manage_node/AEB_Filter_ymax",                       get<1>(AEB_Filter_ROI_max));
    nh.getParam("SIGNAL_manage_node/AEB_Filter_zmax",                       get<2>(AEB_Filter_ROI_max));

    nh.getParam("SIGNAL_manage_node/Parking_Filter_xmin",                   get<0>(Parking_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/Parking_Filter_ymin",                   get<1>(Parking_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/Parking_Filter_zmin",                   get<2>(Parking_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/Parking_Filter_xmax",                   get<0>(Parking_Filter_ROI_max));
    nh.getParam("SIGNAL_manage_node/Parking_Filter_ymax",                   get<1>(Parking_Filter_ROI_max));
    nh.getParam("SIGNAL_manage_node/Parking_Filter_zmax",                   get<2>(Parking_Filter_ROI_max));

    nh.getParam("SIGNAL_manage_node/SmallStaticObstacle_Filter_xmin",       get<0>(SmallStaticObstacle_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/SmallStaticObstacle_Filter_ymin",       get<1>(SmallStaticObstacle_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/SmallStaticObstacle_Filter_zmin",       get<2>(SmallStaticObstacle_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/SmallStaticObstacle_Filter_xmax",       get<0>(SmallStaticObstacle_Filter_ROI_max));
    nh.getParam("SIGNAL_manage_node/SmallStaticObstacle_Filter_ymax",       get<1>(SmallStaticObstacle_Filter_ROI_max));
    nh.getParam("SIGNAL_manage_node/SmallStaticObstacle_Filter_zmax",       get<2>(SmallStaticObstacle_Filter_ROI_max));

    nh.getParam("SIGNAL_manage_node/LargeStaticObstacle_Filter_xmin",       get<0>(LargeStaticObstacle_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/LargeStaticObstacle_Filter_ymin",       get<1>(LargeStaticObstacle_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/LargeStaticObstacle_Filter_zmin",       get<2>(LargeStaticObstacle_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/LargeStaticObstacle_Filter_xmax",       get<0>(LargeStaticObstacle_Filter_ROI_max));
    nh.getParam("SIGNAL_manage_node/LargeStaticObstacle_Filter_ymax",       get<1>(LargeStaticObstacle_Filter_ROI_max));
    nh.getParam("SIGNAL_manage_node/LargeStaticObstacle_Filter_zmax",       get<2>(LargeStaticObstacle_Filter_ROI_max));

    nh.getParam("SIGNAL_manage_node/DelivFilter_xmin",                      get<0>(Deliv_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/DelivFilter_ymin",                      get<1>(Deliv_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/DelivFilter_zmin",                      get<2>(Deliv_Filter_ROI_min));
    nh.getParam("SIGNAL_manage_node/DelivFilter_xmax",                      get<0>(Deliv_Filter_ROI_max));
    nh.getParam("SIGNAL_manage_node/DelivFilter_ymax",                      get<1>(Deliv_Filter_ROI_max));
    nh.getParam("SIGNAL_manage_node/DelivFilter_zmax",                      get<2>(Deliv_Filter_ROI_max));

    nh.getParam("SIGNAL_manage_node/Deliv_raw_LiDAR_xmin",                  get<0>(Deliv_ROImin_toLiDAR));
    nh.getParam("SIGNAL_manage_node/Deliv_raw_LiDAR_ymin",                  get<1>(Deliv_ROImin_toLiDAR));
    nh.getParam("SIGNAL_manage_node/Deliv_raw_LiDAR_zmin",                  get<2>(Deliv_ROImin_toLiDAR));
    nh.getParam("SIGNAL_manage_node/Deliv_raw_LiDAR_xmax",                  get<0>(Deliv_ROImax_toLiDAR));
    nh.getParam("SIGNAL_manage_node/Deliv_raw_LiDAR_ymax",                  get<1>(Deliv_ROImax_toLiDAR));
    nh.getParam("SIGNAL_manage_node/Deliv_raw_LiDAR_zmax",                  get<2>(Deliv_ROImax_toLiDAR));

    nh.getParam("SIGNAL_manage_node/Sign_Board_Heigh",                      SignBoard_H);
    nh.getParam("SIGNAL_manage_node/Sign_Board_Width",                      SignBoard_W);

    ros::Subscriber sub_SIG                     = nh.subscribe<std_msgs::Int32>                      ("/indexFromCtrl", 1, RCVD_SIG);
    ros::Subscriber sub_fusion                  = nh.subscribe<SIG_management::object_msg_arr>       ("/Fusion_msg", 1, RCVD_fusion);
    ros::Subscriber sub_fusion_only_LiDAR       = nh.subscribe<SIG_management::object_msg_arr>       ("/LiDAR_only_msg", 1, RCVD_fusion_only_LiDAR);
    ros::Subscriber sub_fusion_only_Camera      = nh.subscribe<std_msgs::String>                     ("/Camera_only_msg", 1, RCVD_fusion_only_Camera);
    ros::Subscriber sub_fusion_TrafficSign      = nh.subscribe<std_msgs::String>                     ("/Trafficsign_msg", 1, RCVD_fusion_TrafficSign);
    ros::Subscriber sub_direct_LiDAR            = nh.subscribe<SIG_management::object_msg_arr>       ("/Lidar_object", 1, RCVD_direct_LiDAR);

    pub_Fusion_DATA                             = nh.advertise<SIG_management::object_msg_arr>      ("/SIG_Fusion_object",  1);
    pub_Fusion_LiDAR_DATA                       = nh.advertise<SIG_management::object_msg_arr>      ("/SIG_Fusion_Lidar_object",  1);
    pub_Fusion_Camera_DATA                      = nh.advertise<std_msgs::String>                    ("/SIG_Fusion_Camera_object",  1);
    pub_Fusion_TFFsign_DATA                     = nh.advertise<std_msgs::String>                    ("/SIG_Fusion_TFFsign_object",  1);
    pub_LiDAR_DATA                              = nh.advertise<SIG_management::object_msg_arr>      ("/SIG_Lidar_object",  1);
    pub_Mission_name                            = nh.advertise<std_msgs::String>                    ("/SIG_Mission_name",  1);
    pub_toLiDAR_ROI                             = nh.advertise<SIG_management::lidar_signal>        ("/SIG_Delivery_Mission_ROI",  1);

    ros::Rate r(17);
    while(ros::ok()){
        fin_process();
        ros::spinOnce();
        r.sleep();
    }

}