#include "Simulator.h"
#include "GVManager.h"
#include "GVInitializer.h"
#include "RoadMapBuilder.h"
#include "ObjManager.h"
#include "FileManager.h"
#include "LaneBundle.h"
#include "Intersection.h"
#include "ODNode.h"
#include "Section.h"
#include "Lane.h"
#include "RoadEntity.h"
#include "Vehicle.h"
#include "VehicleFamily.h"
#include "VehicleIO.h"
#include "SignalIO.h"
#include "Router.h"
#include "Random.h"
#include "DetectorIO.h"
#include "DetectorUnit.h"
#include "GenerateVehicleIO.h"
#include "GenerateVehicleController.h"
#include "VehicleFamilyManager.h"
#include "VehicleFamilyIO.h"
#include "AmuStringOperator.h"
#include "AmuConverter.h"
#include "Conf.h"
#include "ErrorController.h"
#include <iostream>
#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <functional>
#include <fstream>
#include <cmath>
#include <map>
#ifdef _OPENMP
#include <omp.h>
#endif //_OPENMP

using namespace std;

//======================================================================
Simulator::Simulator()
{
  _roadMap = 0;
  _checkLaneError = false;

  _genVehicleController = &GenerateVehicleController::instance();

  _vehicleIO = &VehicleIO::instance();
  _signalIO = &SignalIO::instance();
}

//======================================================================
Simulator::~Simulator()
{
  TimeManager::printAllClockers();

  // Managerで管理するオブジェクトの開放
  TimeManager::deleteAllClockers();
  FileManager::deleteAllOFStreams();
  ObjManager::deleteAll();
 
  if (_roadMap!=NULL)
  {
    delete _roadMap;
  }
}

//======================================================================
bool Simulator::hasInit() const
{
  if (_roadMap)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//======================================================================
bool Simulator::getReadyRoadMap()
{

  RoadMapBuilder builder;

  // 道路ネットワークの作成
  builder.buildRoadMap();

  // 制限速度の設定
  builder.setSpeedLimit();

  if (GVManager::getFlag("FLAG_INPUT_SIGNAL"))
  {
    // 信号の作成
    builder.buildSignals();
  }
  else
  {
    // 全青信号の作成
    builder.buildSignalsAllBlue();
  }

  _roadMap = builder.roadMap();
  _signalIO->setRoadMap(_roadMap);

  // mapInfo.txtの作成
  if (_roadMap)
  {
    _roadMap->writeMapInfo();
  }

  // コンソールへ地図情報を表示する
  if (GVManager::getFlag("FLAG_VERBOSE"))
  {
    _roadMap->dispIntersections();
  }
  // signal_count.txtの出力
  string fSignalCount;
  GVManager::getVariable("RESULT_SIGNAL_COUNT_FILE", &fSignalCount);
  ofstream ofs(fSignalCount.c_str(), ios::out);

  CITRMAPSI its;
  // 信号の総数 (信号ID=ノードIDの数とは一致しないので注意)
  int totalNumberOfSignals=0;
  for (its=_roadMap->signals()->begin();
      its!=_roadMap->signals()->end();
      its++)
  {
    totalNumberOfSignals += (*its).second->numDirections();
  } 

  ofs << _roadMap->intersections()->size() << "\n" // 交差点の総数
    << totalNumberOfSignals;		     // 信号機の総数

#ifdef BARRIER
  _roadMap->setBarriers();
#endif
  return _roadMap;
}

//======================================================================
bool Simulator::getReadySampleScenario(double xmin, double xmax,
    double ymin, double ymax,
    double xsp,  double ysp,
    unsigned int numVehicles,
    double headway)
{
  RoadMapBuilder builder;

  // 道路ネットワークの作成
  builder.buildGridRoadMap(xmin, xmax, ymin, ymax, xsp, ysp);

  // 全青信号の作成
  builder.buildSignalsAllBlue();

  _roadMap = builder.roadMap();
  _signalIO->setRoadMap(_roadMap);

  // 車両発生に関する設定
  getReadyVehicles();


  // 車両配置
  if (numVehicles)
  {
    _genVehicleController
      ->generateSampleVehicles(numVehicles, headway);
  }

  return _roadMap;
}

//======================================================================
void Simulator::startSampleScenario(double xmin, double xmax,
    double ymin, double ymax,
    double xsp,  double ysp,
    unsigned int numVehicles,
    double headway)
{
  GVInitializer::init("./");
  getReadySampleScenario(xmin, xmax, ymin, ymax, xsp, ysp,
      numVehicles, headway);
}

//======================================================================
bool Simulator::getReadyRoadsideUnit()
{
  assert(_roadMap);

  // 車両感知器設定ファイルの読み込み
  DetectorIO::getReadyDetectors(_roadMap);

  // 感知器データ出力ファイルの準備
  vector<DetectorUnit*>* detectorUnits = ObjManager::detectorUnits();
  DetectorIO::getReadyOutputFiles(detectorUnits);
  if (GVManager::getFlag("FLAG_VERBOSE"))
  {
    DetectorIO::print();
  }

  // 車両発生カウンタの設定ファイル読み込みと準備
  GenerateVehicleIO::getReadyCounters(_roadMap);

  return true;
}

//======================================================================
bool Simulator::getReadyVehicles()
{
  assert(_roadMap);

  _genVehicleController->setRoadMap(_roadMap);
  _genVehicleController->getReadyGeneration();

  return true;
}

//======================================================================
void Simulator::checkLane()
{
  // レーンチェック、エラー時は表示確認のため run のみ止める
  _checkLaneError = !_roadMap->checkIntersectionLane();
}

//======================================================================
bool Simulator::checkLaneError()
{
  return _checkLaneError;
}

//======================================================================
bool Simulator::run(ulint time)
{
  // レーンチェックエラー、表示確認のため run のみ止める
  if (_checkLaneError)
  {
    return false;
  }
  if (time>TimeManager::time())
  {
    TimeManager::startClock("ERROR_MODE");
    TimeManager::startClock("TOTALRUN");
    while (time>TimeManager::time()  
#ifdef OACIS
    &&!ErrorController::stopRun()
#endif
    )
    //&& TimeManager::time() < 2*1000)
    {
      timeIncrement();
    }
    TimeManager::stopClock("TOTALRUN");
    return true;
  }
  else
  {
    return false;
  }
}

//======================================================================
bool Simulator::timeIncrement()
{
  // 時刻の更新
  TimeManager::increment();
  if (GVManager::getFlag("FLAG_VERBOSE"))
  {
    if (TimeManager::time()%1000==0)
    {
      cout << "Time: "
	<< TimeManager::time()/1000 << "[sec]" << endl;
    }
  }

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // エージェント列の更新
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // レーン上のエージェントを更新
  TimeManager::startClock("RENEW_AGENT");
  _roadMap->renewAgentLine();
  TimeManager::stopClock("RENEW_AGENT");

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // モニタリング
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if (GVManager::getFlag("FLAG_OUTPUT_MONITOR_D")
      || GVManager::getFlag("FLAG_OUTPUT_MONITOR_S"))
  {
    vector<DetectorUnit*>* detectorUnits = ObjManager::detectorUnits();
    for_each(detectorUnits->begin(),
	detectorUnits->end(),
	mem_fun(&DetectorUnit::monitorLanes));
    DetectorIO::writeTrafficData(detectorUnits);
  }

#ifdef OACIS
  // 事故の記録
  if(TimeManager::time() % 100000 == 0)
    ErrorController::checkStatData();
#endif

  // エージェントの消去
  TimeManager::startClock("DELETE_AGENT");
#ifdef ERROR_MODE
 deleteAccidentVehicle();
#endif
  _roadMap->deleteArrivedAgents();
  TimeManager::stopClock("DELETE_AGENT");

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // エージェントの発生
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 車両の発生
#ifndef EXCLUDE_VEHICLES
  TimeManager::startClock("GENERATE");
  _genVehicleController->generateVehicle();
  TimeManager::stopClock("GENERATE");
#endif //EXCLUDE_VEHICLES

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 認知
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef EXCLUDE_VEHICLES
  vector<Vehicle*>* vehicles = ObjManager::vehicles();
#ifdef BARRIER
  _roadMap->checkVisible(vehicles);
#endif //BARRIER

#ifndef _OPENMP
  TimeManager::startClock("RECOGNIZE");
  for_each(vehicles->begin(), vehicles->end(),
      mem_fun(&Vehicle::recognize));
  TimeManager::stopClock("RECOGNIZE");
#else //_OPENMP
  int vehiclesSize = vehicles->size();
  Random::multiStockReady(vehiclesSize);
  TimeManager::startClock("RECOGNIZE");
#pragma omp parallel for schedule (dynamic)
  for (int i = 0; i < vehiclesSize; i++)
  {
    Random::multiStockBeginMulti(i);
    (*vehicles)[i]->recognize();
  }
  TimeManager::stopClock("RECOGNIZE");
  Random::multiStockEndMulti();
#endif //_OPENMP

#endif //EXCLUDE_VEHICLES

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 意志決定
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef EXCLUDE_VEHICLES

#ifndef _OPENMP
  TimeManager::startClock("READYTORUN");
  for_each(vehicles->begin(), vehicles->end(),
      mem_fun(&Vehicle::determineAcceleration));
  TimeManager::stopClock("READYTORUN");
#else //_OPENMP
  Random::multiStockReady(vehiclesSize);
  TimeManager::startClock("READYTORUN");
#pragma omp parallel for schedule (dynamic)
  for (int i = 0; i < vehiclesSize; i++)
  {
    Random::multiStockBeginMulti(i);
    (*vehicles)[i]->determineAcceleration();
  }
  TimeManager::stopClock("READYTORUN");
  Random::multiStockEndMulti();
#endif //_OPENMP

#endif //EXCLUDE_VEHICLES

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 行動
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef EXCLUDE_VEHICLES

#ifndef _OPENMP
  TimeManager::startClock("RUN");
  for_each(vehicles->begin(), vehicles->end(),
      mem_fun(&Vehicle::run));
  TimeManager::stopClock("RUN");
#else //_OPENMP
  Random::multiStockReady(vehiclesSize);
  TimeManager::startClock("RUN");
#pragma omp parallel for schedule (dynamic)
  for (int i = 0; i < vehiclesSize; i++)
  {
    Random::multiStockBeginMulti(i);
    (*vehicles)[i]->run();
  }
  TimeManager::stopClock("RUN");
  Random::multiStockEndMulti();
#endif //_OPENMP

#endif //EXCLUDE_VEHICLES

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 時系列データ出力
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  writeRunInfo();
  if (GVManager::getFlag("FLAG_OUTPUT_TIMELINE"))
  {
    writeResult();
  }

  return true;
}

//======================================================================
void Simulator::writeResult() const
{
  const RMAPSI* signals
    = _roadMap->signals();
  _signalIO
    ->writeSignalsDynamicData(TimeManager::time(), signals);

  vector<Vehicle*>* vehicles = ObjManager::vehicles();
  _vehicleIO
    ->writeVehiclesDynamicData(TimeManager::time(), vehicles);
}

//======================================================================
void Simulator::writeRunInfo() const
{
  string fRunInfo;
  GVManager::getVariable("RESULT_RUN_INFO_FILE", &fRunInfo);
  ofstream ofs(fRunInfo.c_str(), ios::trunc);

  /**
   * @page run_info result/runInfo.txt
   *
   * @section シミュレーション実行情報
   * 上段に総ステップ数，下段に1ステップあたりの時間[msec]を出力する．
   */

  if (!ofs.fail())
  {
    ofs << TimeManager::step() << "\n"
      << TimeManager::unit();
    ofs.close();
  }
}

//======================================================================
RoadMap* Simulator::roadMap()
{
  return _roadMap;
}

//======================================================================
GenerateVehicleController* Simulator::generateVehicleController()
{
  return _genVehicleController;
}

//====================================================================== 
void Simulator::deleteAccidentVehicle(){
  std::vector<Vehicle*>* vehicles = ObjManager::vehicles();
  std::vector<Vehicle*>::iterator it = vehicles->begin();
  for(int i=0;i<vehicles->size();i++)
  {
    if(!((*it)->errorController()->accidentCheck()))
    {      
      ObjManager::deleteVehicle(vehicles->at(i),true);
    }
    else
    {      

      vehicles->at(i)->errorController()->errorCheck();

    }
    it++;
  }
}
