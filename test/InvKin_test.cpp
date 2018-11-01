#include <iostream>
#include "InverseKinematics.h"
#include "../include/Utility/dmotion_math.hpp"

using namespace std;
using namespace dmotion;
int main(int argc,char **argv)
{
    vector<double> xx = {0,4.5,-35,0,0,0};
    PrintVector(xx);
//    cout << InvKin::upper_leg_length <<endl;
    InvKin left(false);
    PrintVector(left.LegInvKin(xx));

    return 0;
}
