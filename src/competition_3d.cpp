# include "api_3d.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "asnav_node");
    ros::NodeHandle nh_;

    ASNAV uav(nh_);
    ros::Rate rate(50);
     
    // static constexpr float fly_height = 0.5f, descend_z = 0.3f;

    if (!uav.takeoff(uav.fly_height))
    {
        ROS_INFO("起飞失败，任务终止");
        return 1;
    }

    int mission_num = 0; // 任务阶段计数器
    bool is_hovering = false;        // 是否处于悬停制动状态
    ros::Time hover_start_time;      // 悬停开始的时间戳
    double hover_duration = 2.0;     // 自定义悬停时间（秒），你可以随时修改这个值

    while (ros::ok())
    {
        switch (mission_num)
        {
        case 0:  // 阶段0：起飞
            ROS_INFO("阶段0：起飞成功，准备开始下一任务");
            // if (uav.positionSmooth(0, 0, uav.fly_height, 0.2f, 1.0f))
            // {
            //     ROS_INFO("悬停稳定完成，开始穿框");
            //     uav.reset_target();
            //     mission_num = 1;
            // }
            mission_num = 1;
            break;
        case 1:  
            if (uav.navigationSuper(-1.9f, 0.0f, uav.fly_height, NAN, 0.2f))
            {
                ROS_WARN("[Super] 测试完成");
                mission_num = 2;
            }
            break;

        case 2:  
            if (uav.navigationSuper(-2.5f, -2.7f, uav.fly_height, NAN, 0.2f))
            {
                ROS_WARN("[Super] 测试完成");
                mission_num = 3;
            }
            break;

        case 3:
            if (uav.putShootSimple(-0.4f, -2.1f, uav.fly_height, 0.0f, 0.2f))
            {
                ROS_WARN("任务三完成！！！");
                mission_num = 4;
            }
            break;

        case 4:  
            if (uav.navigationSuper(-2.5f, -2.7f, uav.fly_height, NAN, 0.2f))
            {
                ROS_WARN("[Super] 测试完成");
                mission_num = 5;
            }
            break;

        case 5:  
            if (uav.navigationSuper(-1.9f, 0.0f, uav.fly_height, NAN, 0.2f))
            {
                ROS_WARN("[Super] 测试完成");
                mission_num = 6;
            }
            break;

        case 6:  
            if (uav.navigationSuper(0.0f, 0.0f, uav.fly_height, NAN, 0.2f, true))
            {
                ROS_WARN("[Super] 测试完成");
                mission_num = 14;
            }
            break;
/////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@super任务航点一
        // case 1:  
        //     if (uav.navigationSuper(-2.55f, 0.0f, uav.fly_height, NAN, 0.2f))
        //     {
        //         ROS_WARN("[Super] 测试完成");
        //         mission_num = 2;
        //     }
        //     break;

        // case 2:  
        //     if (uav.navigationSuper(-1.8f, -2.7f, uav.fly_height, NAN, 0.2f))
        //     {
        //         ROS_WARN("[Super] 测试完成");
        //         mission_num = 3;
        //     }
        //     break;

        // case 3:
        //     if (uav.putShootSimple(-0.3f, -2.1f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("任务三完成！！！");
        //         mission_num = 4;
        //     }
        //     break;

        // case 4:  
        //     if (uav.navigationSuper(-1.8f, -2.7f, uav.fly_height, NAN, 0.2f))
        //     {
        //         ROS_WARN("[Super] 测试完成");
        //         mission_num = 5;
        //     }
        //     break;

        // case 5:  
        //     if (uav.navigationSuper(-2.6f, 0.0f, uav.fly_height, NAN, 0.2f))
        //     {
        //         ROS_WARN("[Super] 测试完成");
        //         mission_num = 6;
        //     }
        //     break;

        // case 6:  
        //     if (uav.navigationSuper(0.0f, 0.0f, uav.fly_height, NAN, 0.2f, true))
        //     {
        //         ROS_WARN("[Super] 测试完成");
        //         mission_num = 14;
        //     }
        //     break;
////@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@super任务航点一


        // case 2:
        //     if (uav.arTrackLanding(0.0f, uav.fly_height, 0.1f, 0.15f, 0.0f, 0.0f))
        //     {
        //         ROS_INFO("AR码跟踪降落完成，准备前往下一个任务");
        //         uav.reset_target();
        //         mission_num = 15;
        //     }
        //     break;

        
//睿抗###################################################
        // case 1:
            // if (uav.navigationZplus(-2.85f, 0.0f, uav.fly_height, 0.0f, 0.2f))
            // {
            //     ROS_WARN("完成！！！");
            //     mission_num = 2;
            // }
            // break;

        // case 2:
        //     if (uav.navigationZplus(-2.85f, -2.8f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("完成！！！");
        //         mission_num = 3;
        //     }
        //     break;

        // case 3:
        //     if (uav.putShootSimple(-0.30, -2.1, uav.fly_height, 0.0f, 0.35f))
        //     {
        //         ROS_WARN("任务三完成！！！");
        //         mission_num = 4;
        //     }
        //     break;

        // case 4:
        //     if (uav.positionSmooth(-0.45f, -2.7f, uav.fly_height, 0.2f, 0.0f))
        //     {
        //         ROS_INFO("返航，穿柱中");
        //         mission_num = 5;
        //     }
        //     break;

        // // case 4:
        // // if (uav.navigationZplus(-0.30f, -2.8f, uav.fly_height, 0.0f, 0.2f))
        // //     {
        // //         ROS_INFO("返航，穿柱中");
        // //         mission_num = 5;
        // //     }
        // //     break;

        // case 5:
        //     if (uav.navigationZplus(-2.85f, -2.7f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_INFO("返航，穿柱中");
        //         mission_num = 6;
        //     }
        //     break;

        // case 6:
        //     if (uav.navigationZplus(-2.85f, 0.0f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("完成！！！");
        //         mission_num = 7;
        //     }
        //     break;

        // case 7:
        //     if (uav.navigationZplus(-0.3f, 0.0f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("完成！！！");
        //         mission_num = 14;
        //     }
        //     break;
//睿抗###################################################
        

            

//！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        // case 1:
        //     if (uav.navigationZpro(-2.52f, 0.0f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("穿框完成！！！");
        //         uav.reset_target();
        //         mission_num = 2;
        //     }
        //     break;

        // case 2:
        //     if (uav.navigationZpro(-2.35f, -1.2f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("任务一完成！！！");
        //         uav.reset_target();
        //         mission_num = 3;
        //     }
        //     break;

        // case 3:
        //     if (uav.navigationZpro(-1.63f, -2.6f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("任务二完成！！！");
        //         uav.reset_target();
        //         mission_num = 4;
        //     }
        //     break;

        // case 4:
        //     if (uav.putShoot(-0.32, -2.2, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("任务三完成！！！");
        //         uav.reset_target();
        //         mission_num = 5;
        //     }
        //     break;
        
        // case 5:
        //     if (uav.positionSmooth(-0.4f, -2.6f, uav.fly_height, 0.2f, 0.2f))
        //     {
        //         ROS_INFO("返航，穿柱中");
        //         uav.reset_target();
        //         mission_num = 6;
        //     }
        //     break;

        // case 6:
        //     if (uav.navigationZpro(-2.35f, -1.2f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("返航，穿柱中");
        //         uav.reset_target();
        //         mission_num = 7;
        //     }
        //     break;
        
        // case 7:
        //     if (uav.navigationZpro(-2.52f, 0.0f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("穿框回起点");
        //         uav.reset_target();
        //         mission_num = 8;
        //     }
        //     break;

        // case 8:
        //     if (uav.navigationZpro(0.0f, 0.0f, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("穿框回起点");
        //         uav.reset_target();
        //         mission_num = 14;
        //     }
        //     break;

////！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
       
        // case 2:
        //     if (uav.arTrackLanding(0.0f, uav.fly_height, 0.1f, 0.15f, 0.0f, 0.08f))
        //     {
        //         ROS_INFO("AR码跟踪降落完成，准备前往下一个任务");
        //         uav.reset_target();
        //         mission_num = 15;
        //     }
        //     break;

        // case 21:
        //     if (uav.trackYoloForward(0.01f, 0.005f, 0.005f, 80.0, 20, 20))
        //     {
        //         uav.reset_target();
        //         ROS_INFO("阶段2: 穿框完成，开始 %f 秒悬停制动...", hover_duration);
        //         mission_num = 30;
        //     }
        //     break;               

        // case 8:
        //     if (uav.flyDown(uav.descend_z))
        //     {
        //         ROS_WARN("准备开始调整yaw");
        //         uav.reset_target();
        //         mission_num = 9;
        //     }
        //     break;

        // case 9:
        //     if (uav.controlYaw(0.3f, -1.2f, uav.descend_z, -M_PI/2, 0.2f))
        //     {
        //         uav.reset_target();
        //         ROS_INFO("调整yaw结束,准备开始扎气球");
        //         mission_num = 10;
        //     }
        //     break;
                 
        case 14: 
            if(uav.autoLand())
            {
                ROS_INFO("降落成功，任务完成");
                mission_num = 15;
            }
            break;
                        
        case 15:  
            ROS_INFO_THROTTLE(5, "任务已完成");
            ros::shutdown();
            return 0;
            break;
        
        default:
            ROS_WARN("未知任务阶段：%d", mission_num);
            break;
        }

        uav.setpointPublish();
        ros::spinOnce();
        rate.sleep();  
    }   
    return 0;
}
