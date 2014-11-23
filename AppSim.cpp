#include "AppSim.h"
#include "GVManager.h"
#include <iostream>
#include <cstdlib>
#include <cassert>

using namespace std;

//======================================================================
void AppSim::init(int argc, char** argv, bool output)
{
    AppMates::init(argc, argv, false);
    // 出力の抑制
    GVManager::resetFlag("FLAG_OUTPUT_TIMELINE", false);
    GVManager::resetFlag("FLAG_OUTPUT_TRIP_INFO", false);
    GVManager::resetFlag("FLAG_OUTPUT_MONITOR_D", true);
    GVManager::resetFlag("FLAG_OUTPUT_MONITOR_S", true);
    GVManager::resetFlag("FLAG_OUTPUT_GEN_COUNTER", false);
    
    assert(_simulator);
    _vis.reset(new Visualizer());
}

//======================================================================
void AppSim::parseArgument(int argc, char** argv)
{
    /*
     * AppSim独自のオプションを使用する場合はここに記述する．
     * これはApp::initから呼ばれる．
     */
}

//======================================================================
void AppSim::printUsage()
{
    cout <<
        "Usage  : ./advmates-sim [Option list] \n"
         << endl;
    AppMates::printUsage();
    exit(EXIT_SUCCESS);
}

//======================================================================
int AppSim::run()
{
    if (_vis.get())
    {
        _vis->setSimulator(_simulator);
        _vis->visualize();
	cout << "kokomade" << endl;
	if(GVManager::getFlag("FLAG_AUTO_START"))
	    _vis->autoStart(); 
        return EXIT_SUCCESS;
    }
    else
    {
        cerr << "Visualizer not found." << endl;
        return EXIT_FAILURE;
    }
}
