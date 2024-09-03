#include <iostream>
#include <string>
#include <memory>
#include <thread>

#include "System.h"

//using namespace std;

std::string sData_path = "data";
std::string sConfig_path = "config";

std::shared_ptr<System> pSystem;

void PubImuData()
{
    
}

void PubImageData()
{
}

int main(int argc, char ** argv)
{
    if(!argc != 3)
    {
        std::cout << "start error" << std::endl;
        return -1;
    }

    sData_path = argv[1];
    sConfig_path = argv[2];

    pSystem.reset(new System(sConfig_path));
    std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);

    std::thread thd_PubImuData(PubImuData);
    std::thread thd_PubImageData(PubImageData);

    thd_PubImuData.join();
    thd_PubImageData.join();

    std::cout << "finish" << std::endl;

    return 0;
}