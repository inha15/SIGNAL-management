#ifndef SIG_DECLARE
#define SIG_DECLARE

//header
#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <set>
#include <utility>
#include <algorithm>
#include <string>
#include <time.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Dense>

//MSG
#include "SIG_management/object_msg.h"
#include "SIG_management/object_msg_arr.h"
#include "SIG_management/lidar_signal.h"

//final
#define FIN_DRIVING 60    //include TFF
#define FIRST_CROSSWALK_STOP 61
#define SECOND_CROSSWALK_STOP 63 //
#define DELIV_FISRT 20
#define DELIV_PICK_UP 21
#define DELIV_FISRT_FIN 22 //
#define DELIV_SECOND 40
#define DELIV_DROP_OFF 41
#define DELIV_SECOND_FIN 42 //
#define PARALLEL_PARKING 50
#define PARALLEL_PARKING_STOP 51
#define STATIC_OBSTACLE_LARGE 10

//trial
#define TRI_DRIVING 300
#define DIAGONAL_PARKING 100
#define DIAGONAL_PARKING_STOP 101
#define STATIC_OBSTACLE_SMALL 200
#define RIGHT_TURN_STOP 501
#define TRI_TFFSIGN 400
#define AEB 500


using namespace std;

ros::Publisher pub_Fusion_DATA;
ros::Publisher pub_Fusion_LiDAR_DATA;
ros::Publisher pub_Fusion_Camera_DATA;
ros::Publisher pub_Fusion_TFFsign_DATA;
ros::Publisher pub_LiDAR_DATA;
ros::Publisher pub_Mission_name;
ros::Publisher pub_toLiDAR_ROI;

//Filter Param
bool Filter_switch = 0;
tuple<float,float,float> AEB_Filter_ROI_min = make_tuple(0, 0, 0);
tuple<float,float,float> AEB_Filter_ROI_max = make_tuple(0, 0, 0);
tuple<float,float,float> Parking_Filter_ROI_min = make_tuple(0, 0, 0);
tuple<float,float,float> Parking_Filter_ROI_max = make_tuple(0, 0, 0);
tuple<float,float,float> SmallStaticObstacle_Filter_ROI_min = make_tuple(0, 0, 0);
tuple<float,float,float> SmallStaticObstacle_Filter_ROI_max = make_tuple(0, 0, 0);
tuple<float,float,float> LargeStaticObstacle_Filter_ROI_min = make_tuple(0, 0, 0);
tuple<float,float,float> LargeStaticObstacle_Filter_ROI_max = make_tuple(0, 0, 0);
tuple<float,float,float> Deliv_Filter_ROI_min = make_tuple(0, 0, 0);
tuple<float,float,float> Deliv_Filter_ROI_max = make_tuple(0, 0, 0);
tuple<float,float,float> Deliv_ROImin_toLiDAR = make_tuple(0, 0, 0); //for delivery mission, manipulation
tuple<float,float,float> Deliv_ROImax_toLiDAR = make_tuple(0, 0, 0);
tuple<float,float,float> Ordinary_ROImin_toLiDAR = make_tuple(-50, -50, -50);
tuple<float,float,float> Ordinary_ROImax_toLiDAR = make_tuple(50, 50, 50);
float SignBoard_H = 0;
float SignBoard_W = 0;

//func prototype
void RCVD_SIG(const std_msgs::Int32);
void RCVD_fusion(const SIG_management::object_msg_arr);
void RCVD_fusion_only_LiDAR(const SIG_management::object_msg_arr);
void RCVD_fusion_only_Camera(std_msgs::String);
void RCVD_fusion_TrafficSign(std_msgs::String);
void RCVD_direct_LiDAR(const SIG_management::object_msg_arr);

void Parking_Filter();
void LargeStaticObstacle_Filter();
void SmallStaticObstacle_Filter();
void Deliv_Filter();
void Deliv_wall_remove();
void AEB_Filter();
void Advanced_jiwonFilter_LargeStaticObstacle();

void CheckMission();
void DeliveryMission_toLiDAR_ROI();
void Ordinary_toLiDAR_ROI();
void SEND_DATA();
void fin_process();


struct SeneorData_SIG{
    bool fusion = 0;
    bool fusion_LiDAR = 0;
    bool fusion_Camera = 0;
    bool fusion_TFF = 0;
    bool direct_LiDAR = 0;
    void clear(){
        fusion = fusion_LiDAR = fusion_Camera = fusion_TFF = direct_LiDAR = 0;
    }
    void print(){
        cout<<"Fusion  :  ";
        if(fusion) cout<<"On";
        else cout<<"Off";
        cout<<"    /    LiDAR  :  ";
        if(direct_LiDAR) cout<<"On";
        else cout<<"Off";
        cout<<"    /    TFF_Sign  :  ";
        if(fusion_TFF) cout<<"On";
        else cout<<"Off";
        cout<<endl;
    }
};
SeneorData_SIG SIG_state;

namespace SIG{
class SigManage{
public:
    SigManage(){ 
        this -> clear();
        this -> mission_name = "Drive"; 
    }

    void clear(){
        mission_SIGNAL = 0;
        mission_name.clear();
        fusion_DATA.object_msg_arr.clear();
        fusion_LiDAR_DATA.object_msg_arr.clear();
        fusion_Camera_DATA.data.clear();
        fusion_TFF_DATA.data.clear();
        LiDAR_DATA.object_msg_arr.clear();
    }

    void set_SIGNAL(int SIG)                                                        { this -> mission_SIGNAL = SIG; set_mission_name(SIG); }
    void set_fusion_DATA(SIG_management::object_msg_arr fusion_DATA)                { this -> fusion_DATA = fusion_DATA; }
    void set_fusion_only_LiDAR(SIG_management::object_msg_arr fusion_LiDAR_DATA)    { this -> fusion_LiDAR_DATA = fusion_LiDAR_DATA; }
    void set_fusion_only_Camera(std_msgs::String fusion_Camera_DATA)                { this -> fusion_Camera_DATA = fusion_Camera_DATA;}
    void set_fusion_TrafficSign(std_msgs::String fusion_TFF_DATA)                   { this -> fusion_TFF_DATA = fusion_TFF_DATA;}
    void set_RCVD_direct_LiDAR(SIG_management::object_msg_arr LiDAR_DATA)           { this -> LiDAR_DATA = LiDAR_DATA;}
    void set_mission_name(int SIG){
        switch(SIG) {
        case DELIV_FISRT:
            this -> mission_name = "First Delivery Start";
            break;

        case DELIV_PICK_UP:
            this -> mission_name = "Delivery Pick Up";
            break;

        case DELIV_SECOND:
            this -> mission_name = "Second Delivery Start";
            break;

        case DELIV_DROP_OFF:
            this -> mission_name = "Delivery Drop Off";
            break;

        case DIAGONAL_PARKING:
            this -> mission_name = "Diagonal Parking Start";
            break;

        case PARALLEL_PARKING:
            this -> mission_name = "Parallel Parking Start";
            break;

        case DIAGONAL_PARKING_STOP:
            this -> mission_name = "Diagonal Parking Stop";
            break;

        case PARALLEL_PARKING_STOP:
            this -> mission_name = "Parallel Parking Stop";
            break;            

        case STATIC_OBSTACLE_SMALL:
            this -> mission_name = "Small Static Obstacle avoidance";
            break;

        case STATIC_OBSTACLE_LARGE:
            this -> mission_name = "Large Static Obstacle avoidance";
            break;

        case AEB:
            this -> mission_name = "AEB Start";
            break;

        case RIGHT_TURN_STOP:
            this -> mission_name = "Right Turn Stop";
            break;

        case TRI_TFFSIGN:
            this -> mission_name = "Perceive Traffic Sign for Driving";
            break;

        case FIRST_CROSSWALK_STOP:
            this -> mission_name = "First Cross Walk Stop";
            break;

        case SECOND_CROSSWALK_STOP:
            this -> mission_name = "Second Cross Walk Stop";
            break;

        case DELIV_FISRT_FIN:
            this -> mission_name = "First Delivery Finish";
            break;

        case DELIV_SECOND_FIN:
            this -> mission_name = "Second Delivery Finish";
            break;

        default:
            this -> mission_name = "Drive"; 
            break;
        }
    }
    void set_SIG_state(int SIG){
        SIG_state.clear();
        switch(SIG) {
        case DELIV_FISRT:
        case DELIV_SECOND:
            SIG_state.fusion = 1;
            SIG_state.fusion_LiDAR = 0;
            SIG_state.fusion_Camera = 0;
            SIG_state.fusion_TFF = 0;
            SIG_state.direct_LiDAR = 0;
            break;

        case DELIV_PICK_UP:
        case DELIV_DROP_OFF:
        case RIGHT_TURN_STOP:
        case DIAGONAL_PARKING_STOP:
        case PARALLEL_PARKING_STOP:
            SIG_state.fusion = 0;
            SIG_state.fusion_LiDAR = 0;
            SIG_state.fusion_Camera = 0;
            SIG_state.fusion_TFF = 0;
            SIG_state.direct_LiDAR = 0;
            break;

        case DIAGONAL_PARKING:
        case PARALLEL_PARKING:
        case STATIC_OBSTACLE_SMALL:
        case STATIC_OBSTACLE_LARGE:
        case AEB:
            SIG_state.fusion = 0;
            SIG_state.fusion_LiDAR = 0;
            SIG_state.fusion_Camera = 0;
            SIG_state.fusion_TFF = 0;
            SIG_state.direct_LiDAR = 1;
            break;     

        default:
            SIG_state.fusion = 0;
            SIG_state.fusion_LiDAR = 0;
            SIG_state.fusion_Camera = 0;
            SIG_state.fusion_TFF = 1;
            SIG_state.direct_LiDAR = 0;
            break;
        }
    }

    int get_SIGNAL()                                            {return this -> mission_SIGNAL;}
    string get_mission_name()                                   {return this -> mission_name;}
    SIG_management::object_msg_arr get_fusion_DATA()            {return this -> fusion_DATA;}
    SIG_management::object_msg_arr get_fusion_LiDAR_DATA()      {return this -> fusion_LiDAR_DATA;}
    std_msgs::String get_fusion_Camera_DATA()                   {return this -> fusion_Camera_DATA;}
    std_msgs::String get_fusion_TFF_DATA()                      {return this -> fusion_TFF_DATA;}
    SIG_management::object_msg_arr get_LiDAR_DATA()             {return this -> LiDAR_DATA;}

    bool OutOfRange(SIG_management::object_msg tmp, tuple<float,float,float> min_ROI, tuple<float,float,float> max_ROI){
        if(tmp.x > get<0>(max_ROI) || tmp.x < get<0>(min_ROI)) return 1;
        if(tmp.y > get<1>(max_ROI) || tmp.y < get<1>(min_ROI)) return 1;
        if(tmp.z > get<2>(max_ROI) || tmp.z < get<2>(min_ROI)) return 1;
    }

    void direct_LiDAR_ROI_ReSetting(tuple<float,float,float> min_ROI, tuple<float,float,float> max_ROI){
        SIG_management::object_msg_arr tmp;
        for(int i = 0; i < LiDAR_DATA.object_msg_arr.size(); i++){
            if(OutOfRange(LiDAR_DATA.object_msg_arr[i],min_ROI,max_ROI)) continue;
            tmp.object_msg_arr.push_back(LiDAR_DATA.object_msg_arr[i]);
        }
        LiDAR_DATA = tmp;
    }

    void fusion_DATA_ROI_ReSetting(tuple<float,float,float> min_ROI, tuple<float,float,float> max_ROI){
        SIG_management::object_msg_arr tmp;
        for(int i = 0; i < fusion_DATA.object_msg_arr.size(); i++){
            if(OutOfRange(fusion_DATA.object_msg_arr[i],min_ROI,max_ROI)) continue;
            tmp.object_msg_arr.push_back(fusion_DATA.object_msg_arr[i]);
        }
        fusion_DATA = tmp;
    }

    bool check_In_minmax(SIG_management::object_msg tmp, float heigh, float width){
        if(abs(tmp.zMax - tmp.zMin) > heigh) return 0;
        if(abs(tmp.yMax - tmp.yMin) > width) return 0;
        return 1;
    }

    void fusion_OBJ_remove(float heigh, float width){
        SIG_management::object_msg_arr tmp;
        for(int i = 0; i < fusion_DATA.object_msg_arr.size(); i++){
            if(!check_In_minmax(fusion_DATA.object_msg_arr[i],heigh,width)) continue;
            tmp.object_msg_arr.push_back(fusion_DATA.object_msg_arr[i]);
        }
        fusion_DATA = tmp;
    }

    void Print_Fusion_DATA(){
        for(int i = 0; i < fusion_DATA.object_msg_arr.size(); i++){
            cout << fusion_DATA.object_msg_arr[i].classes << "  ----  x : " <<fusion_DATA.object_msg_arr[i].x << "       y : " <<fusion_DATA.object_msg_arr[i].y<<endl;
        }
    }

private:
    int                             mission_SIGNAL;
    string                          mission_name;
    SIG_management::object_msg_arr  fusion_DATA;
    SIG_management::object_msg_arr  fusion_LiDAR_DATA;
    std_msgs::String                fusion_Camera_DATA;
    std_msgs::String                fusion_TFF_DATA;
    SIG_management::object_msg_arr  LiDAR_DATA;
};
}
SIG::SigManage SIG_MNG;


#endif
