#include "prisma3d/main/MainLoop.h"

namespace prisma3d
{

MainLoop::~MainLoop()
{
}

void MainLoop::initialize()
{
}

void MainLoop::iterationPrepare()
{
}

bool MainLoop::physicsProcess(double)
{
    return false;
}

void MainLoop::iterationEnd()
{
}

bool MainLoop::process(double)
{
    return false;
}

void MainLoop::finalize()
{
}

void MainLoop::notification(int)
{
}

} // namespace prisma3d
