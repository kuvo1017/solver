#include "GenerateVehicleController.h"
#include "RoadMap.h"
#include "Intersection.h"
#include "ODNode.h"
#include "Section.h"
#include "Lane.h"
#include "Vehicle.h"
#include "Random.h"
#include "TimeManager.h"
#include "ObjManager.h"
#include "GVManager.h"
#include "VehicleFamilyManager.h"
#include "VehicleFamilyIO.h"
#include "VehicleFamily.h"
#include "AmuConverter.h"
#include "AmuStringOperator.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <climits>
#include <cmath>

using namespace std;

//======================================================================
GenerateVehicleController::GenerateVehicleController()
{
  _generateVehicleQueue.clear();
}

//======================================================================
GenerateVehicleController::~GenerateVehicleController()
{
}

//======================================================================
GenerateVehicleController& GenerateVehicleController::instance()
{
  static GenerateVehicleController instance;
  return instance;
}

//======================================================================
void GenerateVehicleController::setRoadMap(RoadMap* roadMap)
{
  _roadMap = roadMap;
}

//======================================================================
bool GenerateVehicleController::getReadyGeneration()
{
  //------------------------------------------------------------------
  // ODノードのレベル分け
  vector<ODNode*> vec = _roadMap->odNodes();
  for (unsigned int i=0; i<vec.size(); i++)
  {
    if (0<=_odNodeStartLevel(vec[i])
	&& _odNodeStartLevel(vec[i])<3)
    {
      _startLevel[_odNodeStartLevel(vec[i])].push_back(vec[i]);
    }
    if (0<=_odNodeGoalLevel(vec[i])
	&& _odNodeGoalLevel(vec[i])<3)
    {
      _goalLevel[_odNodeGoalLevel(vec[i])].push_back(vec[i]);
    }
  }

  //------------------------------------------------------------------
  // 標準的な交通量(単路ごとの台/時) : 基本交通容量の10%
  /*
   * "レーンごと"ではない
   */
  _defaultTrafficVolume[0]
    = static_cast<int>(
	GVManager::getNumeric("DEFAULT_TRAFFIC_VOLUME_WIDE"));
  _defaultTrafficVolume[1]
    = static_cast<int>(
	GVManager::getNumeric("DEFAULT_TRAFFIC_VOLUME_NORMAL"));
  _defaultTrafficVolume[2]
    = static_cast<int>(
	GVManager::getNumeric("DEFAULT_TRAFFIC_VOLUME_NARROW"));

  //------------------------------------------------------------------
  // 車種情報の設定
  // デフォルトで20(普通車), 50(大型車)を作成する
  double length, width, height;
  double weight = 0.0;
  double accel, decel;
  double r, g, b;

  length = GVManager::getNumeric("VEHICLE_LENGTH_PASSENGER");
  width  = GVManager::getNumeric("VEHICLE_WIDTH_PASSENGER");
  height = GVManager::getNumeric("VEHICLE_HEIGHT_PASSENGER");
  accel  = GVManager::getNumeric("MAX_ACCELERATION_PASSENGER");
  decel  = GVManager::getNumeric("MAX_DECELERATION_PASSENGER");
  r = 0.0;
  g = 0.0;
  b = 1.0;
  VFAttribute passenger(VehicleFamily::passenger(),
      length, width, height, weight,
      accel, decel, r, g, b);
  VehicleFamilyManager::addVehicleFamily(passenger);

  length = GVManager::getNumeric("VEHICLE_LENGTH_TRUCK");
  width  = GVManager::getNumeric("VEHICLE_WIDTH_TRUCK");
  height = GVManager::getNumeric("VEHICLE_HEIGHT_TRUCK");
  accel  = GVManager::getNumeric("MAX_ACCELERATION_TRUCK");
  decel  = GVManager::getNumeric("MAX_DECELERATION_TRUCK");
  r = 0.3;
  g = 0.7;
  b = 1.0;
  VFAttribute truck(VehicleFamily::truck(),
      length, width, height, weight,
      accel, decel, r, g, b);
  VehicleFamilyManager::addVehicleFamily(truck);

  // 車両情報をファイルから読み込む
  VehicleFamilyIO::getReadyVehicleFamily();
  VehicleFamilyIO::print();

  //------------------------------------------------------------------
  // 車両発生定義ファイルの読み込み
  if (GVManager::getFlag("FLAG_INPUT_VEHICLE"))
  {
    string fGenerateTable, fDefaultGenerateTable;
    GVManager::getVariable("GENERATE_TABLE",
	&fGenerateTable);
    GVManager::getVariable("DEFAULT_GENERATE_TABLE",
	&fDefaultGenerateTable);

    if (!fGenerateTable.empty())
    {
      _table.init(fGenerateTable);
    } 
    if (!fDefaultGenerateTable.empty())
    {
      _defaultTable.init(fDefaultGenerateTable);
    }
  }

  // 車両発生が定義されていない交差点/時間帯のテーブルを作成する
  if (GVManager::getFlag("FLAG_GEN_RAND_VEHICLE"))
  {
    _createRandomTable();
  }

  // 最初の車両発生時刻を決定する
  _determineFirstGenerationTime();

  //------------------------------------------------------------------
  // 経路探索用のパラメータの設定
  _readRouteParameter();

  return true;
}

//======================================================================
void GenerateVehicleController::generateVehicle()
{
  // 車両を確実に発生させる（バス等）場合はここに処理を追加する

  // 車両を確率的に発生させる
  _generateVehicleRandom();

  vector<ODNode*> tmpODNodes;
  for (unsigned int i=0; i<_waitingODNodes.size(); i++)
  {
    _waitingODNodes[i]->pushVehicleToReal(_roadMap);
    if (_waitingODNodes[i]->hasWaitingVehicles())
    {
      // 次のタイムステップでpushVeicleを試みる
      tmpODNodes.push_back(_waitingODNodes[i]);
    }
    else
    {
      // 次にaddWaitingVehicleされるまで何もしない
      _waitingODNodes[i]->setWaitingToPushVehicle(false);
    }
  }
  _waitingODNodes.swap(tmpODNodes);
  /*
     vector<ODNode*> odNodes = _roadMap->odNodes();
     for (unsigned int i=0; i<odNodes.size(); i++)
     {
     if (odNodes[i]->hasWaitingVehicles())
     {
     odNodes[i]->pushVehicleToReal(_roadMap);
     }
     }
   */
}

//======================================================================
void GenerateVehicleController::generateVehicleManual(
    const std::string& startId,
    const std::string& goalId,
    std::vector<std::string> stopPoints,
    VehicleType vehicleType,
    std::vector<double> params)
{
  ODNode* start
    = dynamic_cast<ODNode*>(_roadMap->intersection(startId));
  if (start==NULL)
  {
    cout << "start:" << startId << " is not a OD node." << endl;
    return;
  }

  ODNode* goal = NULL;
  if (goalId!="******")
  {
    goal = dynamic_cast<ODNode*>(_roadMap->intersection(goalId));
  }
  if (goal==NULL)
  {
    goal = _decideGoalRandomly(start);
  }
  if (goal==NULL)
  {
    cout << "cannot decide goal." << endl;
    return;
  }

  assert(start && goal);

  OD od;
  od.setValue(start->id(), goal->id(), stopPoints);
  Vehicle* newVehicle
    = _createVehicle(start,
	goal,
	start,
	start->nextSection(0),
	&od,
	vehicleType,
	params);

  // startの_waitingVehiclesの先頭に加える
  /*
   * 登場は_generateVehicleRandomの中で一括して行う
   */
  start->addWaitingVehicleFront(newVehicle);

  if (!(start->isWaitingToPushVehicle()))
  {
#ifdef _OPENMP
#pragma omp critical (pushWaitingODNode)
    {
#endif //_OPENMP
      _waitingODNodes.push_back(start);
#ifdef _OPENMP
    }
#endif
    start->setWaitingToPushVehicle(true);
  }

  newVehicle->route()->print(cout);
}

//======================================================================
void GenerateVehicleController::generateSampleVehicles(
    unsigned int numVehicles,
    double headway)
{
  assert(_roadMap);

  // ODノード
  vector<ODNode*> nodes = _roadMap->odNodes();

  // レーン1本あたりの車両台数を求める
  /*
   * 単路には上下1本ずつのレーンが含まれ、
   * ODノードに直接接続するレーンには車両を配置しないため、
   * 車両を配置すべきレーン総数は
   * (セクション総数)x2-（ODノード総数）
   */
  int ave = ceil(static_cast<double>(numVehicles)
      / (_roadMap->sections()->size()*2-nodes.size()));
  int id = 0;

  CITRMAPS its = _roadMap->sections()->begin();
  while (its != _roadMap->sections()->end())
  {
    CITRMAPLAN itl = (*its).second->lanes()->begin();
    while (itl != (*its).second->lanes()->end())
    {
      double length = (*itl).second->length();

      // レーン上流にある交差点
      Intersection* prev
	= (*its).second->intersection
	(!((*its).second->isUp((*itl).second)));
      // レーン下流にある交差点
      Intersection* next
	= (*its).second->intersection
	((*its).second->isUp((*itl).second));

      // 下流がODノードであるレーンには発生させない
      if (dynamic_cast<ODNode*>(next))
      {
	itl++;
	continue;
      }

      // ここまでで車両を配置すべき単路とレーンが決定された
      for (int i=0;
	  i<ave && static_cast<unsigned int>(id)<numVehicles;
	  i++)
      {
	// 車両生成処理を行う

	// 起点(ダミーの場合もある)の指定
	ODNode* start = NULL;
	if (dynamic_cast<ODNode*>(prev))
	{
	  start = dynamic_cast<ODNode*>(prev);
	}
	else
	{
	  start = nodes[Random::uniform(0,nodes.size())];
	}

	// 終点(ランダム)の指定
	ODNode* goal = _decideGoalRandomly(start);

	// 経由地情報の設定
	OD od;
	if (dynamic_cast<ODNode*>(prev))
	{
	  od.setValue(start->id(), goal->id());
	}
	else
	{
	  vector<string> stopPoints;
	  stopPoints.push_back(prev->id());
	  stopPoints.push_back(next->id());
	  od.setValue(start->id(), goal->id(),stopPoints);
	  od.setLastPassedStopPoint(prev->id());
	}

	// 車両生成
	Vehicle* tmpVehicle
	  = _createVehicle(start,
	      goal,
	      prev,
	      (*its).second,
	      &od,
	      VehicleFamily::passenger());

	// 車両の配置
	length -= tmpVehicle->bodyLength()*0.5;

	tmpVehicle->addToSection(
	    _roadMap, (*its).second, (*itl).second,
	    (*itl).second->length()
	    -(headway+tmpVehicle->bodyLength())*i
	    -tmpVehicle->bodyLength()*0.5);
	const_cast<Route*>(tmpVehicle->route())
	  ->setLastPassedIntersection(prev);

	bool result
	  = ObjManager::addVehicleToReal(tmpVehicle);
	assert(result);

	length -= (headway + tmpVehicle->bodyLength()*0.5); 
	if (length<tmpVehicle->bodyLength())
	  break;

	id++;
      }
      itl++;
    }
    its++;
  }
}

//======================================================================
void GenerateVehicleController::_createRandomTable()
{
  vector<ODNode*> odNodes = _roadMap->odNodes();

  for (unsigned int i=0; i<odNodes.size(); i++)
  {
    if (_odNodeStartLevel(odNodes[i])<0)
    {
      // ODノードからの流出点がない = Destinationにしかなり得ない
      continue;
    }
    int volume
      = _defaultTrafficVolume[_odNodeStartLevel(odNodes[i])];

    vector<const GTCell*> validGTCells;
    _table.getValidGTCells(odNodes[i]->id(), &validGTCells);
    _defaultTable.getValidGTCells(odNodes[i]->id(), &validGTCells);

    if (validGTCells.empty())
    {
      // 有効なGTCellが見つからなかった場合
      /*
       * デフォルトの交通量を適用する
       */
      /**
       * @todo シミュレーションの最大時間をGVManagerで管理すべき
       */
#ifdef OACIS
      _randomTable.createGTCell(0, GVManager::getNumeric("MAX_TIME"), volume,
	  odNodes[i]->id(), "******");
#else
      _randomTable.createGTCell(0, 86400000, volume,
	  odNodes[i]->id(), "******");
#endif
    }
    else
    {
      // 有効なGTCellが見つかった場合
      /*
       * 指定時間外はデフォルト交通量を適用
       */
      /**
       * @todo 厳密に区間の計算をする必要があるか
       */
      ulint minStart = ULONG_MAX;
      ulint maxEnd = 0;
      for (unsigned int j=0; j < validGTCells.size(); j++)
      {
	if (validGTCells[j]->begin() < minStart)
	{
	  minStart = validGTCells[j]->begin();
	}
	if (validGTCells[j]->end() > maxEnd)
	{
	  maxEnd = validGTCells[j]->end();
	}
      }
      if (minStart > 0)
      {
	_randomTable.createGTCell(0, minStart, volume,
	    odNodes[i]->id(), "******");
      }
#ifdef OACIS
      if (maxEnd < GVManager::getNumeric("MAX_TIME"))
      {
	_randomTable.createGTCell(maxEnd, GVManager::getNumeric("MAX_TIME"), volume,
	    odNodes[i]->id(), "******");
      }
#else
      if (maxEnd < 86400000)
      {
	_randomTable.createGTCell(maxEnd, 86400000, volume,
	    odNodes[i]->id(), "******");
      }
#endif
    }
  }
}

//======================================================================
void GenerateVehicleController::_determineFirstGenerationTime()
{    
  vector<ODNode*> odNodes = _roadMap->odNodes();

  for (unsigned int i=0; i<odNodes.size(); i++)
  {
    if (_odNodeStartLevel(odNodes[i])<0)
    {
      // ODノードからの流出点がない = Destinationにしかなり得ない
      continue;
    }

    vector<const GTCell*> validGTCells;
    _table.getValidGTCells(odNodes[i]->id(), &validGTCells);
    _defaultTable.getValidGTCells(odNodes[i]->id(), &validGTCells);
    _randomTable.getValidGTCells(odNodes[i]->id(), &validGTCells);

    if (!(validGTCells.empty()))
    {
      // 有効なGTCellが見つかった場合
      for (unsigned int j=0; j<validGTCells.size(); j++)
      {
	// 車両発生時刻を決定する
	if (validGTCells[j]->volume()!=0)
	{
	  _addNextGenerateTime(0, validGTCells[j]);
	}
      }
    }
  }
}

//======================================================================
bool GenerateVehicleController::_addNextGenerateTime(
    ulint startTime,
    const GTCell* cell)
{
  // 平均時間間隔[msec]
  /*
   * 交通量から算出される
   */
  double meanInterval = (60.0*60.0*1000.0)/cell->volume();

  // 時間間隔
  ulint interval = ceil(-meanInterval*log(1-Random::uniform()));
  interval += TimeManager::unit()-interval%TimeManager::unit();

  ulint nextTime = startTime + interval;

  if (nextTime <= cell->end())
  {

#ifdef _OPENMP
#pragma omp critical (addNextGenerateTime)
    {
#endif //_OPENMP

      _generateVehicleQueue.insert(
	  pair<unsigned long, const GTCell*>(nextTime, cell));

#ifdef _OPENMP
    }
#endif //_OPENMP

    return true;
  }
  else
  {
    return false;
  }
}

//======================================================================
void GenerateVehicleController::_generateVehicleRandom()
{
  // 現在のタイムステップで車両を発生させるGTCell
  vector<const GTCell*> activeCells;

  while(true)
  {
    if (_generateVehicleQueue.empty())
    {
      break;
    }
    if ((*(_generateVehicleQueue.begin())).first
	> TimeManager::time())
    {
      break;
    }
    activeCells.push_back((*(_generateVehicleQueue.begin())).second);
    _generateVehicleQueue.erase(_generateVehicleQueue.begin());
  }

  if (activeCells.empty())
  {
    return;
  }
#ifndef _OPENMP
  for (unsigned int i=0; i<activeCells.size(); i++)
  {
#else //_OPENMP
    unsigned int cellSize = activeCells.size();    
    Random::multiStockReady(cellSize);
#pragma omp parallel for schedule (dynamic)
    for (unsigned int i=0; i<cellSize; i++)
    {
      Random::multiStockBeginMulti(i);
#endif //_OPENMP
      // 出発地の取得
      string startId = activeCells[i]->start();
      ODNode* start
	= dynamic_cast<ODNode*>(_roadMap->intersection(startId));
      assert(start);

      // 目的地の取得
      string goalId  = activeCells[i]->goal();
      ODNode* goal = NULL;
      if (goalId == "******")
      {
	goal = _decideGoalRandomly(start);
      }
      else
      {
	goal
	  = dynamic_cast<ODNode*>(_roadMap->intersection(startId)); 
      }
      assert(goal);

      // 経由地の取得
      OD od = *(activeCells[i]->od());
      if (goalId == "******")
      {
	// 目的地の再設定
	const OD* pOD = activeCells[i]->od();
	od.setValue(pOD->start(), goal->id(), *(pOD->stopPoints()));
      }

      // 車種の取得
      VehicleType type = activeCells[i]->vehicleType();

      // 車両の生成
      Vehicle* newVehicle
	= _createVehicle(start, goal, start, start->nextSection(0),
	    &od, type);

      // 出発地のwaitingVehiclesに追加
      start->addWaitingVehicle(newVehicle);        

      if (!(start->isWaitingToPushVehicle()))
      {
	start->setWaitingToPushVehicle(true);
#ifdef _OPENMP
#pragma omp critical (pushWaitingODNode)
	{
#endif //_OPENMP
	  _waitingODNodes.push_back(start);
#ifdef _OPENMP
	}
#endif
      }

      // 次の出発時刻を予約
      _addNextGenerateTime(TimeManager::time(), activeCells[i]);
    }
#ifdef _OPENMP
    Random::multiStockEndMulti();
#endif //_OPENMP
  }

  //======================================================================
  void GenerateVehicleController::_generateVehicleRandom2()
  {
    // 車両発生テーブルに現在時刻を通知する
    _table.setTime(TimeManager::time());
    _defaultTable.setTime(TimeManager::time());

    vector<ODNode*> odNodes = _roadMap->odNodes();

#ifndef _OPENMP
    for (unsigned int i=0; i<odNodes.size(); i++)
    {
#else //_OPENMP
      int nodeSize = odNodes.size();
      Random::multiStockReady(nodeSize);
#pragma omp parallel for schedule (dynamic)
      for (unsigned int i=0; i<odNodes.size(); i++)
      {
	Random::multiStockBeginMulti(i);
#endif //_OPENMP
	// 各ODノードの処理
	// 車両発生はGenerate Table, Default Generate Tableで指定された
	// 1つのCellごとに，1[veh./step]まで発生させることができる．
	ODNode* start = odNodes[i];
	ODNode* goal = NULL;

	if (_odNodeStartLevel(start)<0)
	{
	  // ODノードからの流出点がない = Destinationにしかなり得ない
	  continue;
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// ODの決定と車両の生成・登録
	// isEnoughSpaceはAddToRealの段階で判断

	// ファイルにより交通量が指定されているか
	// 指定されていない場合にはdefaultTrafficVolumeに従ってランダム生成
	bool hasActiveGTCells = false;

	OD od;
	int volume;
	VehicleType vehicleType;
	vector<const GTCell*> cells;
	// Generate Tableに基づく車両発生
	_table.getActiveGTCells(start->id(), &cells);
	if (cells.size()!=0)
	{
	  hasActiveGTCells = true;

	  for (unsigned int i=0; i<cells.size(); i++)
	  {
	    od = *cells[i]->od();
	    volume = cells[i]->volume();
	    vehicleType = cells[i]->vehicleType();
	    if (volume!=0)
	    {
	      if (Random::uniform()
		  <volume/60.0/60.0/1000.0*TimeManager::unit())
	      {
		goal
		  = dynamic_cast<ODNode*>(
		      _roadMap->intersection(od.goal()));
		assert(goal);
		od = *cells[i]->od();
		Vehicle* tmpVehicle
		  = _createVehicle(start,
		      goal,
		      start,
		      start->nextSection(0),
		      &od,
		      vehicleType);
		start->addWaitingVehicle(tmpVehicle);
	      }
	    }
	  }
	}

	// Default Generate Tableに基づく車両発生
	_defaultTable.getActiveGTCells(start->id(), &cells);
	if (cells.size()!=0)
	{
	  hasActiveGTCells = true;

	  for (unsigned int i=0; i<cells.size(); i++)
	  {
	    volume = cells[i]->volume();
	    vehicleType = cells[i]->vehicleType();
	    if (volume!=0)
	    {
	      if (Random::uniform()
		  <volume/60.0/60.0/1000.0*TimeManager::unit())
	      {
		goal = _decideGoalRandomly(start);
		assert(goal);
		const OD* pOD = cells[i]->od();
		od.setValue(pOD->start(), goal->id(),
		    *pOD->stopPoints());
		Vehicle* tmpVehicle
		  = _createVehicle(start,
		      goal,
		      start,
		      start->nextSection(0),
		      &od,
		      vehicleType);
		start->addWaitingVehicle(tmpVehicle);
	      }
	    }
	  }
	}

	// Generate Table, Default Generate Tableの両方で指定されていない場合
	// 用意されたdefaultTrafficVolumeの情報を用いる
	/*
	 * Generate Table, Default Generate Tableにおいて
	 * volume=0が指定されている場合には車両は発生しない
	 * ファイルで指定しない場合にはdefaultの交通量に従って発生するため
	 * 車両を発生させたくないODノードがある場合には注意．
	 */
	if (hasActiveGTCells==false
	    && GVManager::getFlag("FLAG_GEN_RAND_VEHICLE"))
	{
	  if (Random::uniform()
	      <_defaultTrafficVolume[_odNodeStartLevel(start)]
	      /60.0/60.0/1000.0*TimeManager::unit())
	  {
	    goal = _decideGoalRandomly(start);
	    if(Random::uniform() < 0.7)
	    {
	      vehicleType = VehicleFamily::passenger();
	    }
	    else
	    {
	      vehicleType = VehicleFamily::truck();
	    }
	    assert(goal);
	    od.setValue(start->id(), goal->id());
	    Vehicle* tmpVehicle
	      = _createVehicle(start,
		  goal,
		  start,
		  start->nextSection(0),
		  &od,
		  vehicleType);
	    start->addWaitingVehicle(tmpVehicle);
	  }
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// 車両の登場
	/*
	 * 渋滞などの理由により生成した車両が即座に登場できない場合があるので
	 * ODNodeの_waitingVehiclesが空でないステップでは登場を試みる
	 * 並列時は後で非並列でまとめて行う
	 */
	if (start->hasWaitingVehicles())
	{
	  start->pushVehicleToReal(_roadMap);
	}
      }
#ifdef _OPENMP
      Random::multiStockEndMulti();
#endif //_OPENMP
    }

    //======================================================================
    Vehicle* GenerateVehicleController::_createVehicle(
	ODNode* start,
	ODNode* goal,
	Intersection* past,
	Section* section,
	OD* od,
	VehicleType vehicleType)
    {
      assert(start!=NULL && goal!=NULL);

      // 車両の生成 
      Vehicle* tmpVehicle = ObjManager::createVehicle();

      //車両属性の設定
      tmpVehicle->setType(vehicleType);
      _setVehicleStatus(tmpVehicle);

      // 経路選択用のパラメータの設定
      tmpVehicle->router()->setTrip(od, _roadMap);
      tmpVehicle->router()
	->setParam(_vehicleRoutingParams
	    [Random::uniform(0,_vehicleRoutingParams.size())]);

      // 経路選択は関数の外に出す
      tmpVehicle->reroute(section, past);

      return tmpVehicle;
    }

    //======================================================================
    Vehicle* GenerateVehicleController::_createVehicle(
	ODNode* start,
	ODNode* goal,
	Intersection* past,
	Section* section,
	OD* od,
	VehicleType vehicleType,
	std::vector<double> params)
    {
      assert(start!=NULL && goal!=NULL);

      // 車両の生成 
      Vehicle* tmpVehicle = ObjManager::createVehicle();

      //車両属性の設定
      tmpVehicle->setType(vehicleType);
      _setVehicleStatus(tmpVehicle);

      // 経路選択用のパラメータの設定
      vector<double> p;
      tmpVehicle->router()->setTrip(od, _roadMap);
      for (int i=0; i<VEHICLE_ROUTING_PARAMETER_SIZE; i++)
      {
	p.push_back(_vehicleRoutingParams[0][i]);
      }
      for (unsigned int i=0; i<params.size()&&i<p.size(); i++)
      {
	p[i] = params[i];
      }
      tmpVehicle->router()->setParam(p);

      // 経路選択
      tmpVehicle->reroute(section, past);

      return tmpVehicle;
    }

    //======================================================================
    void GenerateVehicleController::_setVehicleStatus(Vehicle* vehicle)
    {
      double length, width, height;
      double accel, decel;
      double r, g, b;

      // 車種に対応付けられた属性を取得する
      /*
       * ファイルから読み込んだ属性を適用するように変更
       * 2014/6/8 by H.Fujii
       */
      VFAttribute* vfa
	= VehicleFamilyManager::vehicleFamilyAttribute(vehicle->type());
      if (!vfa)
      {
	if (VehicleFamily::isTruck(vehicle->type()))
	{
	  vfa = VehicleFamilyManager::vehicleFamilyAttribute
	    (VehicleFamily::truck());
	}
	else
	{
	  vfa = VehicleFamilyManager::vehicleFamilyAttribute
	    (VehicleFamily::passenger());
	}
      }
      vfa->getSize(&length, &width, &height);
      vfa->getPerformance(&accel, &decel);
      vfa->getBodyColor(&r, &g, &b);

      vehicle->setBodySize(length, width, height);
      vehicle->setPerformance(accel, decel);
      r = 0.0;
       g = 0.0;
       b = 1.0;
       
      vehicle->setBodyColor(r, g, b);
    }

    //======================================================================
    ODNode* GenerateVehicleController::_decideGoalRandomly(ODNode* start)
    {
      ODNode* result = NULL;

      // start以外のODノードを_goalLevelからコピーする
      // 複数ネットワークを許す場合には，startから到達可能かどうかもチェックする
      vector<ODNode*> goals[3];
      for (int i=0; i<3; i++)
      {
	for (unsigned int j=0; j<_goalLevel[i].size(); j++)
	{
#ifdef UNIQUE_NETWORK
	  if (start!=_goalLevel[i][j])
	  {
	    goals[i].push_back(_goalLevel[i][j]);
	  }
#else  //UNIQUE_NETWORK
	  if (start!=_goalLevel[i][j]
	      && start->isNetworked(start,_goalLevel[i][j]))
	  {
	    goals[i].push_back(_goalLevel[i][j]);
	  }
#endif //UNIQUE_NETWORK
	}
      }
      int level[3];
      for(int i=0; i<3; i++)
      {
	level[i] = _defaultTrafficVolume[i]; 
	if (goals[i].size()==0)
	{
	  level[i] = 0;
	}
      }

      int total = level[0] + level[1] + level[2];
      int r = Random::uniform(RAND_MAX) % total;
      for (int i=0; i<3; i++)
      {
	if (r<level[i])
	{
	  result = goals[i][Random::uniform(goals[i].size())];
	} 
	else
	{
	  r -= level[i];
	}
      }
      assert(result);
      return result;
    }

    //======================================================================
    int GenerateVehicleController::_odNodeStartLevel(ODNode* node) const
    {
      int result = -1;
      // ODノードから見たnumOut
      if (node->border(0)->numOut()>=3)
      {
	result = 0;
      } else if (node->border(0)->numOut()==2)
      {
	result = 1;
      } else if (node->border(0)->numOut()==1)
      {
	result = 2;
      }
      return result;
    }

    //======================================================================
    int GenerateVehicleController::_odNodeGoalLevel(ODNode* node) const
    {
      // ODノードから見たnumIn
      int result = -1;
      if (node->border(0)->numIn()>=3)
      {
	result = 0;
      }
      else if (node->border(0)->numIn()==2)
      {
	result = 1;
      }
      else if (node->border(0)->numIn()==1)
      {
	result = 2;
      }
      return result;
    }

    //======================================================================
    void GenerateVehicleController::_readRouteParameter()
    {
      /*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       * 経路探索用のパラメータの設定
       * 0番目は距離に関するコスト（これが大きいと距離が短い経路が高効用）
       * 1番目は時間に関するコスト（時間が短い経路が高効用）
       * 2番目は交差点での直進に関するコスト（直進が少ない経路が高効用）
       * 3番目は交差点での左折に関するコスト（左折が少ない経路が高効用）
       * 4番目は交差点での右折に関するコスト（右折が少ない経路が高効用）
       * 5番目は道路の広さに関するコスト（道路幅が広い経路が高効用）
       *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
       */

      _vehicleRoutingParams.resize(1);
      _vehicleRoutingParams[0].resize(VEHICLE_ROUTING_PARAMETER_SIZE);

      _vehicleRoutingParams[0][0] = 1;
      _vehicleRoutingParams[0][1] = 0;
      _vehicleRoutingParams[0][2] = 0;
      _vehicleRoutingParams[0][3] = 0;
      _vehicleRoutingParams[0][4] = 0;
      _vehicleRoutingParams[0][5] = 0;

      string fParamFile;
      GVManager::getVariable("VEHICLE_ROUTE_PARAM_FILE",
	  &fParamFile);

      ifstream inParamFile(fParamFile.c_str(), ios::in);
      if (inParamFile.good())
      {
	string str;
	int index = 0;
	while (inParamFile.good())
	{
	  getline(inParamFile, str);
	  AmuStringOperator::getAdjustString(&str);
	  if (!str.empty())
	  {
	    vector<string> tokens;
	    AmuStringOperator::getTokens(&tokens, str, ',');
	    if (tokens.size()==VEHICLE_ROUTING_PARAMETER_SIZE)
	    {
	      // パラメータ指定が有効な行
	      _vehicleRoutingParams.resize(index+1);
	      _vehicleRoutingParams[index]
		.resize(VEHICLE_ROUTING_PARAMETER_SIZE);

	      for (unsigned int i=0; i<tokens.size(); i++)
	      {
		_vehicleRoutingParams[index][i]
		  = AmuConverter::strtod(tokens[i]);
	      }
	      index++;
	    }
	  }
	}
      }
      else
      {
	if (GVManager::getFlag("FLAG_INPUT_VEHICLE"))
	{
	  // 入力ファイルが存在しない場合
	  cout << "no vehicle routing parameter file: "
	    << fParamFile << endl;
	}
	_vehicleRoutingParams.resize(3);
	for (unsigned int i=1; i<_vehicleRoutingParams.size(); i++)
	{
	  _vehicleRoutingParams[i]
	    .resize(VEHICLE_ROUTING_PARAMETER_SIZE);
	}

	_vehicleRoutingParams[1][0] = 0;
	_vehicleRoutingParams[1][1] = 1;
	_vehicleRoutingParams[1][2] = 0;
	_vehicleRoutingParams[1][3] = 0;
	_vehicleRoutingParams[1][4] = 0;
	_vehicleRoutingParams[1][5] = 0;

	_vehicleRoutingParams[2][0] = 1;
	_vehicleRoutingParams[2][1] = 1;
	_vehicleRoutingParams[2][2] = 0;
	_vehicleRoutingParams[2][3] = 0;
	_vehicleRoutingParams[2][4] = 0;
	_vehicleRoutingParams[2][5] = 0;
      }

      if (GVManager::getFlag("FLAG_VERBOSE"))
      {
	cout << endl << "*** Vehicle Routing Parameters ***" << endl;
	cout << "NumParams: " << _vehicleRoutingParams.size() << ", "
	  << _vehicleRoutingParams[0].size() << endl;
	for (unsigned int i=0; i<_vehicleRoutingParams.size(); i++)
	{
	  cout << i << ":";
	  for (unsigned int j=0; j<_vehicleRoutingParams[i].size(); j++)
	  {
	    cout << _vehicleRoutingParams[i][j] << ",";
	  }
	  cout << endl;
	}
	cout << endl;
      }
    }
