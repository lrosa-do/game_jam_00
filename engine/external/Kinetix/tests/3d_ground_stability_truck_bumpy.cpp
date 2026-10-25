#include "check.h"
#include "ground_rig.h"

using namespace ground_test;

int main()
{
    truckStabilityMinute(true, 60.0f);
    truckStabilityMinute(true, 120.0f);
    return TEST_RESULT();
}
