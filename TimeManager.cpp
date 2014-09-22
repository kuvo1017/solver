#include "TimeManager.h"
#include "Clocker.h"
#include <iostream>
#ifdef _OPENMP
#include <omp.h>
#endif //_OPENMP

using namespace std;

// メンバ変数の定義
ulint TimeManager::_time = 0;
ulint TimeManager::_unit = 100;
ulint TimeManager::_step = 0;

MAPCLK TimeManager::_clockers;

//======================================================================
ulint TimeManager::time()
{
    return _time;
}

//======================================================================
void TimeManager::setTime(ulint presentTime)
{
    _time = presentTime;
}

//======================================================================
ulint TimeManager::unit()
{
    return _unit;
}

//======================================================================
void TimeManager::setUnit(ulint unit)
{
    _unit = unit;
}

//======================================================================
ulint TimeManager::step()
{
    return _step;
}

//======================================================================
void TimeManager::setStep(ulint step)
{
    _step = step;
}

//======================================================================
void TimeManager::increment()
{
    _step++;
    _time += _unit;
}

//======================================================================
bool TimeManager::startClock(const std::string clockName)
{
    // マルチスレッド処理の中では計時しない
    /*
     * 計時の中にマルチスレッド処理が負組まれるのは構わない
     */

#ifdef _OPENMP
    if (omp_in_parallel())
    {
        return false;
    }
#endif //_OPENMP    

    // 該当する時計の検索
    Clocker* clocker;
    ITRMAPCLK iclk = _clockers.find(clockName);
    if (iclk==_clockers.end())
    {
        // 見つからなければ新たに作成する
        clocker = new Clocker();
        _clockers[clockName] = clocker;
        cout << "clock " << clockName << " created." << endl;
    }
    else
    {
        clocker = (*iclk).second;
    }

    bool result = clocker->startClock();
    if (!result)
    {
        cerr << "clock " << clockName << " has already started" << endl;
        return false;
    }
    return true;
}

//======================================================================
bool TimeManager::stopClock(const std::string clockName)
{
    // マルチスレッド処理の中では計時しない
    /*
     * 計時の中にマルチスレッド処理が負組まれるのは構わない
     */

#ifdef _OPENMP
    if (omp_in_parallel())
    {
        return false;
    }
#endif //_OPENMP    

    // 該当する時計の検索
    Clocker* clocker;
    ITRMAPCLK iclk = _clockers.find(clockName);
    if (iclk==_clockers.end())
    {
        // 見つからない場合
        cerr << "clock " << clockName << " is not found" << endl;
        return false;
    }
    clocker = (*iclk).second;

    bool result = clocker->stopClock();
    if (!result)
    {
        cerr << "clock " << clockName << " has not started" << endl;
        return false;
    }
    return true;
}

//======================================================================
void TimeManager::printAllClockers()
{
    cout << "clock name/ total cpu time[sec], total wallclock time[sec]"
         << endl;
    CITRMAPCLK ciclk = _clockers.begin();
    while (ciclk != _clockers.end())
    {
        cout << (*ciclk).first << "/ "
             << (*ciclk).second->totalProcessorTime() << ", "
             << (*ciclk).second->totalTime() << endl;
        ciclk++;
    }
}

//======================================================================
void TimeManager::deleteAllClockers()
{
    ITRMAPCLK iclk = _clockers.begin();
    while (iclk != _clockers.end())
    {
        delete (*iclk).second;
        iclk++;
    }
    _clockers.erase(_clockers.begin(), _clockers.end());
}
