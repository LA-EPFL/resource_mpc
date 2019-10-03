#include "ros/ros.h"
#include "resource_mpc/plant_model.hpp"
#include "resource_mpc/rmpc_state.h"
#include "resource_mpc/rmpc_control.h"

#include <memory>
#include <thread>
#include <mutex>

#include "nmpc.hpp"
#include "resource_mpc/plant_model.hpp"

class Controller
{
public:
    Controller(const ros::NodeHandle &_nh);
    ~Controller(){}

    void compute_control();
    void publish();

private:
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Publisher pub;
    ros::Subscriber sub;
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmpc_controller_node");
    ros::NodeHandle n;

    return 0;
}