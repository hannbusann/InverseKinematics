#include <iostream>
#include "InverseKinematics.h"
#include "../include/Utility/dmotion_math.hpp"

using namespace std;
using namespace dmotion;

int main(int argc, char **argv) {
    /**Initialize a vector to profile robot's one foot of right/left leg.
     * Here the vector contains x y z r p y of one foot
     */



    InvKin right(true);
    vector<double> l1 = {0, -27.5, -12.5, 0, 0, -30};
    PrintVector(right.LegInvKin(l1));

    InvKin left(false);
    vector<double> l2 = {0,27.5, -12.5, 0 , 0 , 30};
    PrintVector(left.LegInvKin(l2));


    /**弧度制和角度制互相转换的演示**/
    vector<double> yy = {180, 90, 0, -90, -270};
    PrintVector(Deg2Rad(yy));
    PrintVector(Rad2Deg(yy));

    return 0;
}
