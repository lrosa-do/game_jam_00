#include "check.h"
#include "ground_rig.h"

using namespace ground_test;

int main()
{
    truckStabilityMinute(false, 60.0f);
    truckStabilityMinute(false, 120.0f);
    return TEST_RESULT();
}
