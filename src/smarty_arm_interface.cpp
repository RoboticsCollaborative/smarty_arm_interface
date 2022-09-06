#include "smarty_arm_interface/smarty_arm_interface.h"

using namespace std;

/* RDDNode constructor */
SMARTY_ARM_Node::SMARTY_ARM_Node(ros::NodeHandle &node, Rdda *rddaptr, std::string type): node_type(type) {
    nh_ = node;
    rdda = rddaptr;

    if (node_type == "master") {
        smarty_arm_packet_sub = nh_.subscribe("/pti_slave_output", 1, &SMARTY_ARM_Node::eepacket_callback, this, ros::TransportHints().udp());
        smarty_arm_packet_pub = nh_.advertise<smarty_arm_interface::EEPacket>("/smarty_arm_output", 1);
    }

    ROS_INFO("Node initialized");
}

SMARTY_ARM_Node::~SMARTY_ARM_Node() = default;


/* Publish rdda packet through ROS */
void SMARTY_ARM_Node::publish_eepacket() {

    smarty_arm_interface::EEPacket packet_msg;

    packet_msg.wave.resize(3);

    mutex_lock(&rdda->mutex);
    for (int i = 0; i < 3; i ++) {
        packet_msg.wave[i] = rdda->arm[0].eePacket[i].wave_out;
    }
    packet_msg.position.x = rdda->arm[0].ee[0].pos;
    packet_msg.position.y = rdda->arm[0].ee[1].pos;
    packet_msg.position.z = rdda->arm[0].ee[2].pos;
    packet_msg.angle.x = rdda->arm[0].ee[3].pos;
    packet_msg.angle.y = rdda->arm[0].ee[4].pos;
    packet_msg.angle.z = rdda->arm[0].ee[5].pos;
    packet_msg.twist.linear.x = rdda->arm[0].ee[0].vel;
    packet_msg.twist.linear.y = rdda->arm[0].ee[1].vel;
    packet_msg.twist.linear.z = rdda->arm[0].ee[2].vel;
    packet_msg.twist.angular.x = 0.0;
    packet_msg.twist.angular.y = 0.0;
    packet_msg.twist.angular.z = 0.0;
    mutex_unlock(&rdda->mutex);

    // if (smarty_arm_packet_pub.getNumSubscribers() == 0) {
    //     if (node_type == "master") {
    //         ROS_ERROR_STREAM("Connection lost, trying to reconnect...");
    //         smarty_arm_packet_pub.shutdown();
    //         smarty_arm_packet_pub = nh_.advertise<smarty_arm_interface::EEPacket>("/smarty_arm_output", 1);
    //     }
    // }

    smarty_arm_packet_pub.publish(packet_msg);

    // ROS_INFO("Published RDDARead message");
}

/* Subscriber callback */
/* Comment out callback for remote test */
void SMARTY_ARM_Node::eepacket_callback(const smarty_arm_interface::EEPacket::ConstPtr &packet_msg) {

    mutex_lock(&rdda->mutex);
    for (int i = 0; i < 3; i++) {
        rdda->arm[0].eePacket[i].wave_in = packet_msg->wave[i];
    }

    mutex_unlock(&rdda->mutex);
    ROS_INFO_THROTTLE(1, "Write into smarty arm memory");

}


/* Run loop */
void SMARTY_ARM_Node::run() {
    ros::Rate loop_rate(500);
    while (ros::ok()) {
	/* Publisher (wrap) */
    publish_eepacket();
	/* Subscriber callback loop */
	ros::spinOnce();
	loop_rate.sleep();
    }
}

int main(int argc, char** argv) {

    /* Instanciate input-output data varibles */
    Rdda *rdda;

    /* Map data structs to shared memory */
    /* Open and obtain shared memory pointers for master-input data */
    rdda = initRdda();
    if (rdda == nullptr) {
        fprintf(stderr, "Init rdda failed.\n");
        printf("shm_open error, errno(%d): %s\n", errno, strerror(errno));
        exit(1);
    }

    /* Initialise ROS node */
    ros::init(argc, argv, "smarty_arm_interface");
    printf("Launch ros interface\n");

    ros::NodeHandle node("~");

    // initialize a node with "master" or "slave" setting
    SMARTY_ARM_Node smarty_arm(node, rdda, std::string(argv[1]));
    ROS_INFO("Node starts running");
    smarty_arm.run();
}
