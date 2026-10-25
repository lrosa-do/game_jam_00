#pragma once

namespace prisma3d
{

class MainLoop
{
public:
    enum Notification : int
    {
        kNotificationOsMemoryWarning = 2009,
        kNotificationWmAbout = 2011,
        kNotificationCrash = 2012,
        kNotificationApplicationResumed = 2014,
        kNotificationApplicationPaused = 2015,
        kNotificationApplicationFocusIn = 2016,
        kNotificationApplicationFocusOut = 2017
    };

    virtual ~MainLoop();

    virtual void initialize();
    virtual void iterationPrepare();
    virtual bool physicsProcess(double time);
    virtual void iterationEnd();
    virtual bool process(double time);
    virtual void finalize();
    virtual void notification(int what);
};

} // namespace prisma3d
