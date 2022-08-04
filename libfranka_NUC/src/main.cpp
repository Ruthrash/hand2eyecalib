#include <franka/exception.h>

#include <iostream>
#include <array>
#include <string>

#include <zmq.hpp>

#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>

int main(int argc, char** argv) {

    try {

    franka::Robot robot("192.168.1.107");// should be robot's ip
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REP);
    socket.bind ("tcp://127.0.0.1:2000");//change IP if communicating between two machines

        while(true) {
            std::cout<<"Waiting for EE frame request"<<std::endl;
            zmq::message_t request;
            socket.recv (&request);
            franka::RobotState robot_state = robot.readOnce();
	    zmq::message_t ee_frame_reply(robot_state.O_T_EE);
	    socket.send (ee_frame_reply);
	    std::cout<<"Sent current EE frame \n";
        }

    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
}
