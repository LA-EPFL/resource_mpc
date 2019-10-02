#include "ros/ros.h"
#include "resource_mpc/plant_model.hpp"
#include "resource_mpc/rmpc_state.h"
#include "resource_mpc/rmpc_control.h"
#include <memory>

using namespace casadi;

class Plant
{
public:
    Plant(const ros::NodeHandle &_nh);
    ~Plant(){}

    DM state;
    DM control;

private:
    void read_power();
    void control_callback(const resource_mpc::rmpc_control::ConstPtr &msg){}

    std::shared_ptr<ros::NodeHandle> nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    std::string ip_address;
    uint port;
    double current_power;
};

Plant::Plant(const ros::NodeHandle &_nh)
{
    /** initialize ROS part */
    nh = std::make_shared<ros::NodeHandle>(_nh);
    sub = nh->subscribe("/rmpc_control", 100, &Plant::control_callback, this);
    pub = nh->advertise<resource_mpc::rmpc_state>("/rmpc_state", 100);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmpc_plant_node");
    ros::NodeHandle n;

    Plant plant(n);

    return 0;
}