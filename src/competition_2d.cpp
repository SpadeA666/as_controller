# include "api_2d.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "asnav_node");
    ros::NodeHandle nh_;

    ASNAV uav(nh_);
    ros::Rate rate(20);
     
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
            mission_num = 1;
            break;

        // case 1:
        //     if (!is_hovering) 
        //     {
        //         // 1. 正常导航到目标点
        //         if (uav.navigation(1.86f, -1.27f, uav.fly_height, 0.0f, 0.2f))
        //         {
        //             ROS_WARN("刚进入目标点1容差范围，开始 %f 秒悬停制动...", hover_duration);
        //             is_hovering = true;                  // 触发悬停状态
        //             hover_start_time = ros::Time::now(); // 记录当前时间
        //         }
        //     }
        //     else 
        //     {
        //         // 2. 悬停状态下，强制发送该点的静态位置指令，让飞控急刹车
        //         uav.position(1.7f, -1.0f, uav.fly_height, 0.0f, 0.2f);

        //         // 3. 检查悬停时间是否满足
        //         if ((ros::Time::now() - hover_start_time).toSec() >= hover_duration)
        //         {
        //             uav.reset_target();
        //             ROS_INFO("机身已稳定。进入下一任务。");
        //             is_hovering = false;  // 重置标志位，为了以后其他的 case 能继续用
        //             mission_num = 2;      // 真正进入下一阶段
        //         }
        //     }
        //     break;

        case 1:
            if (uav.navigation(1.86f, -1.1f, uav.fly_height, 0.0f, 0.2f))
            {
                ROS_WARN("到达", hover_duration);
                is_hovering = true;                  // 触发悬停状态
                hover_start_time = ros::Time::now(); // 记录当前时间
                mission_num = 3;
            }
            break;

        // case 2:
        //     if (uav.controlYaw(2.0f, -0.9f, uav.fly_height, M_PI/2, 2.0f))
        //     {
        //         uav.reset_target();
        //         ROS_INFO("阶段2: 调整yaw结束");
        //         mission_num = 3;
        //     }
        //     break;
            

        // case 2:
        //     if (uav.trackYoloForward(0.01f, 0.005f, 0.005f, 80.0, 20, 20))
        //     {
        //         uav.reset_target();
        //         ROS_INFO("阶段2: 穿框完成，开始 %f 秒悬停制动...", hover_duration);
        //         mission_num = 3;
        //     }
        //     break;        
        
        case 2:
            if (uav.positionSmooth(3.0f, -1.0f, uav.fly_height, 0.2f, 1.0f))
            {
                uav.reset_target();
                ROS_INFO("悬停结束，机身已稳定。进入下一任务。");
                mission_num = 3;
            }
            break;

        // case 2:
        //     if (uav.positionSmooth(1.2f, 1.2f, uav.fly_height, 0.2f, 2.0f))
        //     {
        //         uav.reset_target();
        //         ROS_INFO("阶段2: 悬停结束，机身已稳定。进入下一任务。");
        //         mission_num = 3;
        //     }
        //     break;
            
                  
        case 3: 
            if(uav.autoLand())
            {
                ROS_INFO("降落成功，任务完成");
                mission_num = 4;
            }
            break;
                        
        case 4:  // 阶段4：任务完成，保持等待
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
