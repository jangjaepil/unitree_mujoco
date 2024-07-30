#include<controller.hpp>

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }
    
    // std::cin.get();
    controller controller;
    controller.Init(); // communication init
    controller.run();
    
    return 0;
}
