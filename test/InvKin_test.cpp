#include <iostream>
#include "InverseKinematics.h"
#include "../include/Utility/dmotion_math.hpp"

using namespace std;
using namespace dmotion;

int main(int argc, char **argv) {
    /**Initialize a vector to profile robot's one foot of right/left leg.
     * Here the vector contains x y z r p y of one foot
     */


//
//    InvKin right(true);
//    vector<double> l1 = {0, -4.5, -26, 0, 30, 0};
//    cout << "第一个" << endl;
//    PrintVector(right.LegInvKin(l1));

    InvKin left(false);
    vector<double> l2 = {3.5, 4.5, -31, 0, -91, 0};
    cout << "左腿" << endl;
    PrintVector(left.LegInvKin(l2));

    cout << "制度测试" << endl;
    /**弧度制和角度制互相转换的演示**/
    vector<double> yy = {180, 90, 0, -90, -270};
    PrintVector(Deg2Rad(yy));
    PrintVector(Rad2Deg(yy));

    cout << "全身" << endl;
    WholeBodyIK twoleg(9.0);
    vector<double> mass = {31, 0, 3.5, 0, 90, 0};
    PrintVector(twoleg.GetIKResult(mass));

    cout << "双变量反正切函数测试" << endl;
    cout << dmotion::Atan(1,1) << endl;

    return 0;
}
