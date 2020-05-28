#include <kuka_control/iiwa_manipulation.h>

int main(int argc, char ** argv){
    ros::init(argc, argv, "pnp_node");
    ros::NodeHandle nh;

    iwtros::iiwaMove mover(nh, "iiwa_arm");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    mover.run();
    spinner.stop();
     // Hello
    return 0;
}
