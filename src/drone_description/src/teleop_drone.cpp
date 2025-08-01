#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <map>
#include <array>
#include <vector>

// key to delta speeds
static std::map<char, std::array<double,4>> bindings = {
    {'w', { 10,  10,  10,  10}},
    {'s', {-10, -10, -10, -10}},
    {'i', { 10, -10, -10,  10}},
    {'k', {-10,  10,  10, -10}},
    {'j', {-10, -10,  10,  10}},
    {'l', { 10,  10, -10, -10}},
    {'a', { 10, -10,  10, -10}},
    {'d', {-10,  10, -10,  10}}
};

// Read one character without waiting for Enter
int getch(){
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "drone_teleop_cpp");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/motor_speed", 1);

    std::vector<double> speeds(4, 0.0);
    ROS_INFO("\nControls:\n  w/s : collective throttle\n  i/k : pitch forward/back\n  j/l : roll left/right\n  a/d : yaw left/right\nCTRL-C to quit\n");

    ros::Rate rate(20);
    while(ros::ok()){
        // wait up to 10 ms for a key
        struct timeval timeout;
        timeout.tv_sec  = 0;
        timeout.tv_usec = 10000;

        if(::isatty(STDIN_FILENO) &&
           ::select(STDIN_FILENO+1, nullptr, nullptr, nullptr, &timeout) > 0)
        {
            char c = getch();
            if(c=='\x03') break;  // Ctrl-C
            auto it = bindings.find(c);
            if(it != bindings.end()){
                auto &d = it->second;
                for(int i=0; i<4; ++i){
                    speeds[i] = std::max(0.0, speeds[i] + d[i]);
                }
            }
        }

        std_msgs::Float64MultiArray msg;
        msg.data = speeds;
        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    // stop on exit
    std_msgs::Float64MultiArray stop_msg;
    stop_msg.data = {0,0,0,0};
    pub.publish(stop_msg);

    return 0;
}
