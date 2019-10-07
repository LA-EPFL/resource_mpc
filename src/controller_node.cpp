#include "ros/ros.h"
#include "resource_mpc/plant_model.hpp"
#include "resource_mpc/rmpc_state.h"
#include "resource_mpc/rmpc_control.h"

#include <memory>
#include <thread>
#include <mutex>

#include "nmpc.hpp"
#include "resource_mpc/plant_model.hpp"

using namespace casadi;

#define DIMX 3
#define DIMU 3
#define NUMSEGMENTS 6
#define POLYORDER 3

using nmpc = polympc::nmpc<System, DIMX, DIMU, NUMSEGMENTS, POLYORDER>;

class Controller
{
public:
    Controller(const ros::NodeHandle &_nh);
    ~Controller(){}

    void compute_control();
    void publish();

    DM control;
    DM state;
    double D;

    bool initialised(){return m_initialised;};

private:
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    std::shared_ptr<nmpc> mpc;

    void estimator_callback(const resource_mpc::rmpc_state::ConstPtr &msg);
    bool m_initialised;
};

void Controller::estimator_callback(const resource_mpc::rmpc_state::ConstPtr &msg)
{
    state(0) = msg->x1;
    state(1) = msg->x2;
    state(2) = msg->e;

    mpc->setReference(DM(msg->ref_y));

    if(!m_initialised)
        m_initialised = true;
}

Controller::Controller(const ros::NodeHandle &_nh)
{
    /** intialise ROS part*/
    nh = std::make_shared<ros::NodeHandle>(_nh);
    sub = nh->subscribe("/rmpc_state", 100, &Controller::estimator_callback, this);
    pub = nh->advertise<resource_mpc::rmpc_control>("/rmpc_control", 100);

    /** NMPC part */
    double tf = 3.0;
    casadi::DM y_ref = casadi::DM(1.0);

    casadi::DMDict mpc_props;
    mpc_props["mpc.Q"] = casadi::DM(100);
    mpc_props["mpc.R"] = casadi::DM::diag(casadi::DM(std::vector<double>{0.5,0,0}));
    mpc_props["mpc.P"] = casadi::DM(100);

    mpc = std::make_shared<nmpc>(y_ref, tf, mpc_props);

    /** set state and control constraints */
    DM lbu = DM(std::vector<double>{-10, 0.5, 0.25});
    DM ubu = DM(std::vector<double>{10, 2, 0.5});
    mpc->setLBU(lbu);
    mpc->setUBU(ubu);

    const double max_resource = 1.0;
    const double e0 = max_resource / 2;

    DM lbx = DM::vertcat({-DM::inf(), -5, 0});
    DM ubx = DM::vertcat({ DM::inf(),  5, max_resource});
    mpc->setLBX(lbx);
    mpc->setUBX(ubx);

    state = DM::vertcat({0, 0, e0});
    control = DM::vertcat({0,1.0,0.25});

    m_initialised = false;
}

void Controller::publish()
{
    resource_mpc::rmpc_control msg;
    std::vector<double> control_vec = control.nonzeros();
    msg.header.stamp = ros::Time::now();
    msg.u  = control_vec[0];
    msg.D  = control_vec[1];
    msg.v  = control_vec[2];

    D = control_vec[1];

    pub.publish(msg);
}

void Controller::compute_control()
{
    mpc->computeControl(state);
    DM opt_ctl = mpc->getOptimalControl();

    /** set the control to send */
    control = opt_ctl(Slice(0,3), opt_ctl.size2() - 1);
}

void c_sleep(const double &milliseconds)
{
    if(milliseconds > 0.0)
        usleep(milliseconds * 1000);
    else
        return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmpc_controller_node");
    ros::NodeHandle n;

    Controller controller(n);
    ros::Rate rate(20);

    while(ros::ok())
    {
        ros::spinOnce();
        if(controller.initialised())
        {
            double start = ros::Time::now().toSec();
            controller.compute_control();
            double finish = ros::Time::now().toSec();
            controller.publish();
            double comp_time_ms = (finish - start) * 1000;
            std::cout << "Controller computation time: " << comp_time_ms << " [ms] \n";
            c_sleep((controller.D * 0.25)*1000 - comp_time_ms - 40);
            std::cout << "Controller sleeping for: " << (controller.D * 0.25)*1000 - comp_time_ms << " [ms] \n";
            //rate.sleep();
        } else {
            rate.sleep();
            std::cout << "Controller is not initialised \n";
        }
    }

    return 0;
}