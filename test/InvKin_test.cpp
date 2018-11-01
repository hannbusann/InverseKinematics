#include <iostream>
#include "../include/InverseKinematics/InverseKinematics.h"
#include "../include/Utility/dmotion_math.hpp"

using namespace std;
int main(int argc,char **argv)
{
    vector<double> xx = {1,6,-24,0,0,0};
    dmotion::PrintVector(xx);
    dmotion::InvKin left(false);

    return 0;
}
