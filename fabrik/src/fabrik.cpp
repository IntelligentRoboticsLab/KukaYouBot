

#include "components/Arm.cpp"

const double KUKA_YOUBOT_ARM_MEASUREMENTS[] = { 147, 155, 135, 218 };
const double KUKA_YOUBOT_JOINT_OFFSETS[] = { 169, 65, -146, 102.5 };
const double KUKA_YOUBOT_JOINT_INVERT[] = { 1, -1, 1 };
const double MAX_DISTANCE = 1;
const int MAX_ITERATIONS = 30;

const int KUKA_YOUBOT_JOINT_VERTICAL_PLANE = 1;

class Fabrik {

public:

    Arm theArm;

    Fabrik() {

        theArm.initiateArm( KUKA_YOUBOT_ARM_MEASUREMENTS,
                            KUKA_YOUBOT_JOINT_OFFSETS,
                            KUKA_YOUBOT_JOINT_INVERT );

    }


};
