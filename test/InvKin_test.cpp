#include <iostream>
#include "InverseKinematics.h"
#include "../include/Utility/dmotion_math.hpp"

using namespace std;
using namespace dmotion;

int main(int argc, char **argv) {
    /**Initialize a vector to profile robot's one foot of right/left leg.
     * Here the vector contains x y z r p y of one foot
     */



    InvKin left(false);
    for (int i = 0; i < 1000; i++) {
        vector<double> xx = {13.423, 12.25, -20.1, 0, -90, 30};
        left.LegInvKin(xx);
    }
    vector<double> xx = {13.423, 12.25, -20.1, 0, -90, 30};
    PrintVector(left.LegInvKin(xx));


    /**弧度制和角度制互相转换的演示**/
    vector<double> yy = {180, 90, 0, -90, -270};
    PrintVector(Deg2Rad(yy));
    PrintVector(Rad2Deg(yy));

    return 0;
}
