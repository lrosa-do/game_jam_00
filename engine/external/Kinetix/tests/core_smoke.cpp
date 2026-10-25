#include "check.h"

#include <kinetix/core/Math.h>

int main()
{
    CHECK_NEAR(kinetix::Math::dot(kinetix::Math::vec3(1, 2, 3), kinetix::Math::vec3(4, 5, 6)), 32.0, 1e-6);
    return TEST_RESULT();
}
