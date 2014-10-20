#include "AppCalc.h"
#include "Simulator.h"
#include "TimeManager.h"
#include "GVManager.h"
#include "VehicleIO.h"
#include "ErrorController.h"
#include <iostream>
#include <cctype>
#include <cstdlib>
#include <unistd.h>
#ifndef USE_MINGW
#include <getopt.h>
#endif

using namespace std;

//======================================================================
AppCalc::AppCalc()
{
    _maxTime = DEFAULT_MAX_TIME;
}

//======================================================================
void AppCalc::init(int argc, char** argv, bool output)
{
    // オプションの処理とグローバル変数の準備，simulatorの作成
    AppMates::init(argc, argv, output);

    if (output
        && GVManager::getFlag("FLAG_VERBOSE"))
    {
        cout << "time to stop calculation    : " << _maxTime << endl;
        cout << "output timeline data        : "
             << (GVManager::getFlag("FLAG_OUTPUT_TIMELINE")?"true":"false")
             << "\noutput monitoring detail data      : "
             << (GVManager::getFlag("FLAG_OUTPUT_MONITOR_D")?"true":"false")
             << "\noutput monitoring statistic data   : "
             << (GVManager::getFlag("FLAG_OUTPUT_MONITOR_S")?"true":"false")
             << "\noutput generate counter data       : "
             << (GVManager::getFlag("FLAG_OUTPUT_GEN_COUNTER")?"true":"false")
             << "\noutput trip distance and trip time : "
             << (GVManager::getFlag("FLAG_OUTPUT_TRIP_INFO")?"true":"false")
             << endl;
    }
}

//======================================================================
void AppCalc::parseArgument(int argc, char** argv)
{
    int opt;
#ifdef USE_MINGW
    while ((opt = getopt(argc, argv,
                         App::shortOptions.c_str())) != -1)
#else //USE_MINGW
    while ((opt = getopt_long(argc, argv,
                              AppMates::shortOptions.c_str(),
                              AppMates::longOptions,
                              &AppMates::optionIndex)) != -1)
#endif //USE_MINGW
    {
        switch (opt)
        {
        case 'T':
        case 't':
            _maxTime = static_cast<unsigned long>(atoi(optarg));
            break;
        case 'S': // 時系列データ出力をonに(calcデフォルトでon)
            GVManager::resetFlag("FLAG_OUTPUT_TIMELINE", true);
            break;
        case 's': // 時系列データ出力をoffに
            GVManager::resetFlag("FLAG_OUTPUT_TIMELINE", false);
            break;
        case 'L': // 走行距離，時間の出力をonに(calcデフォルトでon)
            GVManager::resetFlag("FLAG_OUTPUT_TRIP_INFO", true);
            break;
        case 'l': // 走行距離，時間の出力をoffに
            GVManager::resetFlag("FLAG_OUTPUT_TRIP_INFO", false);
            break;
        case 'M': // 路側器データ出力をonに(calcデフォルトでon)
            GVManager::resetFlag("FLAG_OUTPUT_MONITOR_D", true);
            GVManager::resetFlag("FLAG_OUTPUT_MONITOR_S", true);
            break;
        case 'm': // 路側器データ出力をoffに
            GVManager::resetFlag("FLAG_OUTPUT_MONITOR_D", false);
            GVManager::resetFlag("FLAG_OUTPUT_MONITOR_S", false);
            break;
#ifndef USE_MINGW
        case 20: // 路側器データのうち詳細データの出力をoffに
            GVManager::resetFlag("FLAG_OUTPUT_MONITOR_D", false);
            break;
        case 21: // 路側器データのうち統計データの出力をoffに
            GVManager::resetFlag("FLAG_OUTPUT_MONITOR_S", false);
            break;
#endif //USE_MINGW
        case 'G': // 発生データ出力をonに(matesデフォルトでon)
            GVManager::resetFlag("FLAG_OUTPUT_GEN_COUNTER", true);
            break;
        case 'g': // 発生データ出力をoffに
            GVManager::resetFlag("FLAG_OUTPUT_GEN_COUNTER", false);
            break;
        default:
            break;
        }
    }
}

//======================================================================
void AppCalc::printUsage()
{
    cout <<
        "Usage  : ./advmates-calc [Option list] \n"
         << endl;
    AppMates::printUsage();
    cout <<
        " -t                : time to stop calculation\n"
        "                     This must be multiple number of time step.\n"
        "                     (Time step default is 100 mili second.)\n"
        "                     If this isn't given, simulator will use default value.\n"
        "                     (default: "
         << DEFAULT_MAX_TIME
         <<
        "[mili second]).\n"
        " -s                : do not output timeline data\n"
        " -m                : do not output monitoring data\n"
#ifndef USE_MINGW
        " --no-output-monitor-d\n"
        "                   : do not output monitoring data(detail)\n"
        " --no-output-monitor-s\n"
        "                   : do not output monitoring data(statistic)\n"
#endif //USE_MINGW
        " -g                : do not output generate counter data\n"
        " -l                : do not output trip info\n"
         << endl;
    exit(EXIT_SUCCESS);
}

//======================================================================
int AppCalc::batchRun()
{
    ulint maxTime = _maxTime;
    ulint mt = GVManager::getMaxTime();
    if (mt>100)
        maxTime = mt;
#ifdef ERROR_MODE
maxTime = ErrorController::maxTime();
#endif

    cout << "*** Run advmates-calc: max time=" << maxTime
         << " (dt=" << TimeManager::unit() << ") ***" << endl;
    _simulator->run(maxTime);

    if (GVManager::getFlag("FLAG_OUTPUT_TRIP_INFO"))
    {
        VehicleIO::instance().writeAllVehiclesDistanceData();
    }

    return EXIT_SUCCESS;
}
