#include "RobotContext.h"


RobotContext::RobotContext()
{
    limelight = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}
