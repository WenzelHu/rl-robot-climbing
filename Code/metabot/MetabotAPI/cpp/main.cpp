#include <iostream>
#include <unistd.h>
#include <MetabotV2.h>

using namespace Metabot;

int main()
{
    MetabotV2 metabot("/dev/rfcomm0");

    // Enable monitoring
    metabot.monitor(10);

    // Wait for robot info
    metabot.waitUpdate();

    // Print voltage
    std::cout << "Metabot V2, " << metabot.voltage << "V" << std::endl;
    
    // Starting the motors
    std::cout << std::endl;
    std::cout << "> Starting motors" << std::endl;
    metabot.start();
    metabot.waitUpdate();
    sleep(3);

    // Moving
    std::cout << std::endl;
    std::cout << "> Walking forward" << std::endl;
    metabot.control(100, 0, 0);
    sleep(3);
    std::cout << "> Turning" << std::endl;
    metabot.control(0, 0, 50);
    sleep(3);
    metabot.control(0, 0, 0);

    // Sound
    std::cout << std::endl;
    std::cout << "> Beep" << std::endl;
    metabot.beep(440, 500);
    sleep(3);

    // Changing colors
    std::cout << std::endl;
    std::cout << "> Yellow!" << std::endl;
    metabot.setLeds("yellow");
    sleep(3);
    std::cout << "> Cyan!" << std::endl;
    metabot.setLeds("cyan");
    sleep(3);
    std::cout << "> Off!" << std::endl;
    metabot.setLeds("off");
    sleep(3);
 
    // Stopping the robot
    std::cout << std::endl;
    std::cout << "> Reset" << std::endl;
    metabot.reset();
    sleep(3);

    std::cout << "> Stopping" << std::endl;
    metabot.stop();

    return 0;
}
