#include "ros/ros.h"
#include "resource_mpc/plant_model.hpp"
#include "resource_mpc/rmpc_state.h"
#include "resource_mpc/rmpc_control.h"
#include <memory>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/wait.h>

using namespace casadi;

#define PORT 65001

class Plant
{
public:
    Plant(const ros::NodeHandle &_nh);
    Plant(){};
    ~Plant()
    {
        if(pwr_thread.joinable()) pwr_thread.join();
        close(pwr_socket);
    }

    DM state;
    DM control;

private:
    void read_power();
    void control_callback(const resource_mpc::rmpc_control::ConstPtr &msg){}

    std::shared_ptr<ros::NodeHandle> nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    std::string ip_address;
    uint32_t port;
    int pwr_socket;
    struct sockaddr_in pwr_addr;
    struct hostent *he;
    int numbytes;
    char buf[100];
    unsigned addr_len;
    double current_power;

    std::thread pwr_thread;
    std::mutex pwr_mutex;
};

Plant::Plant(const ros::NodeHandle &_nh)
{
    /** initialize ROS part */
    nh = std::make_shared<ros::NodeHandle>(_nh);
    sub = nh->subscribe("/rmpc_control", 100, &Plant::control_callback, this);
    pub = nh->advertise<resource_mpc::rmpc_state>("/rmpc_state", 100);

    /** initialise UDP port */
    ip_address = "127.0.0.1";
    port = 65002;

    if((pwr_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        std::cerr << "plant_node: failed to open power date socket \n";
        exit(2);
    }
    else
        std::cout << "plant_node: successfully opened socket \n";

    if((he = gethostbyname(ip_address.c_str())) == NULL)
    {
        std::cerr << "plant_node: could not get host by name \n";
        exit(1);
    } else
    {
        std::cout << "Got host by name: " << ip_address << " : type:  " << he->h_addrtype << "\n";
    }

    memset((char *)&pwr_addr, 0, sizeof(pwr_addr));

    pwr_addr.sin_family = AF_INET;
    pwr_addr.sin_port = htons(PORT);
    pwr_addr.sin_addr.s_addr = INADDR_ANY;
    //pwr_addr.sin_addr = *((struct in_addr *)he->h_addr);

    //int broadcast = 1;
    //setsockopt(pwr_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    if (setsockopt (pwr_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
        std::cerr << "setsockopt failed \n";

    if( ::bind(pwr_socket, (struct sockaddr *)&pwr_addr, sizeof(pwr_addr)) == -1 )
    {
        std::cerr << "plant_node: power socket failed to bind \n";
        exit(2);
    } else
    {
        std::cout << "plant_node: successfully binded to " << ip_address << " : " << port << "\n";
    }


    /** start power reading thread */
    pwr_thread = std::thread(&Plant::read_power, this);
}

void Plant::read_power()
{
    double temp[4];
    addr_len = sizeof(temp);

    std::cout << "Started power reading thread \n";

    while(ros::ok())
    {
        int numbytes = 0;
        if( numbytes = recvfrom(pwr_socket, (void*)&temp, sizeof(temp), 0, (struct sockaddr *)&pwr_addr, &addr_len) == -1)
        {
            std::cerr << "plant_node: failed to receive power measurements \n";
            //exit(1);
        }
        else
        {
            std::cout << "plant_node: received " << numbytes << " bytes length: " << addr_len << "\n";
        }

        //double power = static_cast<double>(temp);
        //std::cout << "plant_node: current power consumption: " << power << " Watts \n";

        usleep(10 * 1000);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmpc_plant_node");
    ros::NodeHandle n;

    Plant plant(n);


    while(ros::ok())
    {
        std::cout << "plant mode is running \n";
        usleep(100 * 1000);
    }

    return 0;
}