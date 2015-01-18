#include "Vehicle.h"
#include "RoadMap.h"
#include "LaneBundle.h"
#include "Intersection.h"
#include "ODNode.h"
#include "Section.h"
#include "Lane.h"
#include "RoadEntity.h"
#include "TimeManager.h"
#include "Random.h"
#include "Route.h"
#include "ARouter.h"
#include "Router.h"
#include "LocalLaneRoute.h"
#include "LocalLaneRouter.h"
#include "Signal.h"
#include "SignalColor.h"
#include "VirtualLeader.h"
#include "GVManager.h"
#include "AmuPoint.h"
#include "ErrorController.h"
#include <cassert>
#include <cmath>
#include <algorithm>
#include <math.h>

using namespace std;

//======================================================================
Vehicle::Vehicle():_id()
{

  _bodyLength = 4.400;
  _bodyWidth  = 1.830;
  _bodyDiagnoalXY = sqrt(_bodyLength * _bodyLength + _bodyWidth * _bodyWidth);
  _bodyHeight = 1.315;
  _bodyColorR = 0.0;
  _bodyColorG = 0.0;
  _bodyColorB = 1.0;

  _roadMap      = NULL;
  _intersection = NULL;
  _prevIntersection = NULL;
  _section      = NULL;
  _lane         = NULL;
  _nextLane     = NULL;
  _prevLane     = NULL;

  _length      = 0;
  _oldLength   = -10;
  _totalLength = 0;
  _tripLength  = 0;
  _error       = 0;
  _velocity      = 0;
  _velocityHistory.clear();
  _errorVelocity = 0;
  _accel         = 0;
  _vMax = 60.0/60.0/60.0;
  _startTime = 0;

  _blinker.setNone();

  _laneShifter.setVehicle(this);

  _strictCollisionCheck
    = (GVManager::getNumeric("STRICT_COLLISION_CHECK") == 1);
  _entryTime     = 0;
  _isNotifying   = false;
  _hasPaused     = false;
  _sleepTime     = 0;
  _isNotifySet   = false;
  _isUnnotifySet = false;

  _route  = new Route();
  _router = new Router();
  _localRouter.setLocalRoute(&_localRoute);
  _localRoute.clear();
  _errorController = new ErrorController(this);
}

//======================================================================
void Vehicle::setId(const string& id)
{
  _id = id;
}

//======================================================================
Vehicle::~Vehicle()
{
  if(_router) delete _router;
  if(_route) delete _route;

  if(_intersection) _intersection->eraseWatchedVehicle(this);
  if(_section) _section->eraseWatchedVehicle(this);

  for (int i=0; i<static_cast<signed int>(_leaders.size()); i++)
  {
    delete _leaders[i];
  }
  _leaders.clear();
}

//======================================================================
const string&  Vehicle::id() const
{
  return _id;
}

//======================================================================
VehicleType Vehicle::type() const
{
  return _type;
}

//======================================================================
void Vehicle::setType(VehicleType type)
{
  _type = type;
}

//======================================================================
double Vehicle::bodyWidth() const
{
  return _bodyWidth;
}

//======================================================================
double Vehicle::bodyLength() const
{
  return _bodyLength;
}
//======================================================================
double Vehicle::bodyDiagnoalXY() const
{
  return _bodyDiagnoalXY;
}
 
//======================================================================
double Vehicle::bodyHeight() const
{
  return _bodyHeight;
}

//======================================================================
void Vehicle::setBodySize(double length, double width, double height) 
{
  _bodyLength = length;
  _bodyWidth = width;
  _bodyHeight = height;
}

//======================================================================
void Vehicle::setPerformance(double accel, double brake)
{
  assert(accel>0 && brake<0);
  _maxAcceleration = accel*1.0e-6;
  _maxDeceleration = brake*1.0e-6;
}

//======================================================================
void Vehicle::setBodyColor(double r, double g, double b)
{
  _bodyColorR = r;
  _bodyColorG = g;
  _bodyColorB = b;
}
//======================================================================
void Vehicle::getBodyColor(double* result_r,
    double* result_g,
    double* result_b) const
{
  *result_r = _bodyColorR;
  *result_g = _bodyColorG;
  *result_b = _bodyColorB;
}

//======================================================================
double Vehicle::error() const
{
return _error;
}
//======================================================================
double Vehicle::errorVelocity() const
{
return _error;
}
 

//======================================================================
double Vehicle::length() const
{
  return _length;
}

//======================================================================
double Vehicle::oldLength() const
{
  return _oldLength;
}

//======================================================================
double Vehicle::totalLength() const
{
  return _totalLength;
}

//======================================================================
double Vehicle::tripLength() const
{
  return _tripLength;
}

//======================================================================
double Vehicle::x() const
{
  AmuVector pv(_lane->beginConnector()->point(),
      _lane->endConnector()->point());
  pv.normalize();

  double x = _lane->beginConnector()->point().x()+ _length*pv.x();

  pv.revoltXY(M_PI_2);
  x += _error * pv.x();

  return x;
}


//======================================================================
double Vehicle::y() const
{
  AmuVector pv(_lane->beginConnector()->point(),
      _lane->endConnector()->point());
  pv.normalize();

  double y = _lane->beginConnector()->point().y()+ _length*pv.y();

  pv.revoltXY(M_PI_2);
  y += _error * pv.y();

  return y;
}

//======================================================================
double Vehicle::z() const
{
  AmuVector pv(_lane->beginConnector()->point(),
      _lane->endConnector()->point());
  pv.normalize();

  double z = _lane->beginConnector()->point().z()+ _length*pv.z();

  pv.revoltXY(M_PI_2);
  z += _error * pv.z();

  return z;
}

//======================================================================
LaneBundle* Vehicle::laneBundle() const
{
  assert((!_section && _intersection)
      || (_section && !_intersection));

  if (_section)
  {
    return _section;
  }
  else if (_intersection)
  {
    return _intersection;
  }
  else
  {
    return NULL;
  }
}

//======================================================================
Section* Vehicle::section() const
{
  return _section;
}

//======================================================================
Intersection* Vehicle::intersection() const
{
  return _intersection;
}

//======================================================================
Lane* Vehicle::lane() const
{
  return _lane;
}
//====================================================================== 
LocalLaneRoute Vehicle::localRoute() const 
{
  return _localRoute;
}
//======================================================================
bool Vehicle::isAwayFromOriginNode() const
{
  bool check = true;

  if (_route->lastPassedIntersectionIndex()==0
      && _route->start()==_router->start()
      && _totalLength
      < GVManager::getNumeric("NO_OUTPUT_LENGTH_FROM_ORIGIN_NODE"))
  {
    check = false;
  }
  return check;
}

//======================================================================
double Vehicle::velocity() const
{
  return _velocity;
}

//======================================================================
double Vehicle::aveVelocity() const
{
  if (_velocityHistory.empty())
  {
    return 1.0;
  }

  double sum = 0.0;
  for (unsigned int i=0; i<_velocityHistory.size(); i++)
  {
    sum += _velocityHistory[i];
  }
  return sum / _velocityHistory.size();
}

//======================================================================
double Vehicle::accel() const
{
  return _accel;
}

//======================================================================
const AmuVector Vehicle::directionVector() const
{
  assert(_lane!=NULL);
  return _lane->directionVector();
}

//======================================================================
void Vehicle::notify()
{
  if (_section)
  {
    _section->addWatchedVehicle(this);
  }
  else
  {
    _intersection->addWatchedVehicle(this);
  }
  _isNotifying = true;
}

//======================================================================
void Vehicle::unnotify()
{
  if (_section)
  {
    _section->eraseWatchedVehicle(this);
  }
  else
  {
    _intersection->eraseWatchedVehicle(this);
  }
  _isNotifying = false; 
}

//======================================================================
Blinker Vehicle::blinker() const
{
  return _blinker;
}

//======================================================================
int Vehicle::directionFrom() const
{
  Intersection* inter;
  if (_intersection)
  {
    inter = _intersection;
  }
  else
  {
    inter = _section->intersection(_section->isUp(_lane));
  }
  if (inter)
  {
    const vector<Lane*>* liInter = _localRoute.lanesInIntersection();
    assert(inter->isMyLane((*liInter)[0]));
    return inter->direction((*liInter)[0]->beginConnector());
  }
  else 
  {
    return -1;
  }
}

//======================================================================
int Vehicle::directionTo() const
{
  Intersection* inter;
  if (_intersection)
  {
    inter = _intersection;
  }
  else
  {
    inter = _section->intersection(_section->isUp(_lane));
  }
  if (inter)
  {
    const vector<Lane*>* liInter = _localRoute.lanesInIntersection();
    assert(inter->isMyLane((*liInter)[liInter->size()-1]));
    return inter
      ->direction((*liInter)[liInter->size()-1]->endConnector());
  }
  else
  { 
    return -1;
  }
}

//======================================================================
VehicleLaneShifter& Vehicle::laneShifter()
{
  return _laneShifter;
}

//======================================================================
bool Vehicle::isSleep() const
{
  return (_sleepTime>0);
}

//======================================================================
void Vehicle::setStartTime(ulint startTime)
{
  _startTime = startTime;
}

//======================================================================
ulint Vehicle::startTime() const
{
  return _startTime;
}

//======================================================================
ulint Vehicle::startStep() const
{
  return _startTime / TimeManager::unit();
}

//======================================================================
const vector<VirtualLeader*>* Vehicle::virtualLeaders() const
{
  return &_leaders;
}

//======================================================================
ErrorController* Vehicle::errorController()
{
  return _errorController;
} 
//======================================================================
const Route* Vehicle::route() const
{
  return _route;
}

//======================================================================
ARouter* Vehicle::router()
{
  return _router;
}

//======================================================================
/*
 * この関数は以前は全ての再探索で使用されていたが，
 * 現在は車両発生時にしか呼ばれなくなった．
 * ("reroute"という関数名は正しくない)
 */
bool Vehicle::reroute(const Intersection* start)
{
  bool succeed = true;
  int _routerIStep = 10000;
  Route* new_route = NULL;

  _router->search(start, _routerIStep, new_route);

  if(new_route == NULL)
  {
    succeed = false;
  }
  else if(new_route->size() == 0)
  {
    succeed = false;
  }

  if(_route != NULL)
  {
    delete _route;
  }

  if(succeed)
  {
    _route = new_route;
    _localRouter.setRoute(_route);
  }
  else
  {
    if(new_route != NULL)
    {
      delete new_route;
      new_route = NULL;
    }
    // ルート探索失敗した場合は仕方が無いのでスタート地点とゴール地点を
    // 直接結ぶ経路を設定する (ほとんどの場合、このような経路はない。)
    // 落ちるのを防ぐための苦肉の策
    _route = new Route();
    _route->push(const_cast<Intersection*>(_router->start()));
    _route->push(const_cast<Intersection*>(_router->goal()));
    _localRouter.setRoute(_route);
  }

  return succeed;
}

//======================================================================
bool Vehicle::reroute(const Section* section,
    const Intersection* start)
{
  bool succeed = true;
  int _routerIStep = 10000;
  Route* new_route = NULL;

  // cout << "vehicle:" << _id << " reroute." << endl;
  _router->search(section, start, _routerIStep, new_route);

  if(new_route == NULL)
  {
    succeed = false;
  }
  else if(new_route->size() == 0)
  {
    succeed = false;
  }

  // 古い経路を削除
  if(_route != NULL)
  {
    delete _route;
  }

  // 新しい経路を設定
  if(succeed)
  {
    _route = new_route;
    _localRouter.setRoute(_route);
  }
  else
  {
    if(new_route != NULL)
    {
      delete new_route;
      new_route = NULL;
    }
    // ルート探索失敗した場合は仕方が無いのでスタート地点とゴール地点を
    // 直接結ぶ経路を設定する(ほとんどの場合、このような経路はない。)
    // 落ちるのを防ぐための苦肉の策
    _route = new Route();
    _route->push(const_cast<Intersection*>(_router->start()));
    _route->push(const_cast<Intersection*>(_router->goal()));
    _localRouter.setRoute(_route);
  }

  return succeed;
}

//======================================================================
const vector<Lane*>* Vehicle::lanesInIntersection() const
{
  return _localRoute.lanesInIntersection();
}

//====================================================================== 
void Vehicle::returnError()
{
  _error = 0.0;
  _errorVelocity = 0.0;
}

//====================================================================== 
void Vehicle::stopByAccident()
{
  _velocity = 0.0;
}
//======================================================================
void Vehicle::print() const
{
  cout << "--- Vehicle Information ---" << endl;
  cout << "ID: " << _id << ", Type: " << _type << endl;

  // 位置と速度に関するもの
  if (_section!=NULL)
  {
    cout << "Section ID, Lane ID: " << _section->id();
  }
  else
  {
    cout << "Intersection ID, Lane ID: " << _intersection->id();
  }
  cout << ", " << _lane->id() << endl;
  cout << "Length, Error: " << _length << ", " << _error << endl;
  cout << "Velocity, ErrorVelocity: "
    << _velocity <<", "<< _errorVelocity << endl;
  cout << "(x,y,z)=("
    << x() << ", " << y() << ", " << z() << ")"<< endl;

  // 経路に関するもの
  _router->printParam();
  _route->print(cout);
  _localRoute.print();

  // 交錯レーンに関するもの
  if (_section!=NULL)
  {
    Intersection* nextInter
      = _section->intersection(_section->isUp(_lane));
    if (nextInter)
    {
      vector<Lane*> cli;
      vector<Lane*> cls;
      nextInter->collisionLanes(_localRoute.lanesInIntersection(),
	  &cli, &cls);
      cout << "Collision Lanes in Intersection:" << endl;
      for (unsigned int i=0; i<cli.size(); i++)
      {
	cout << "\t" << cli[i]->id() << endl;
      }
      cout << "Collision Lanes in Section:" << endl;
      for (unsigned int i=0; i<cls.size(); i++)
      {
	cout << "\t" << cls[i]->id() << " of section "
	  << nextInter->nextSection
	  (nextInter->direction
	   (cls[i]->endConnector()))->id() 
	  << endl;
      }
    }
  }

  // 車線変更に関するもの
  if (_laneShifter.isActive())
  {
    cout << "Shift Lane:" << endl;
    cout << "  Target Lane  : " << _laneShifter.laneTo()->id() << endl;
    cout << "  Target Length: " << _laneShifter.lengthTo() << endl;
  }

#ifdef VL_DEBUG
  cout << "Virtual Leaders:" << endl;
  for (unsigned int i=0; i<_leaders.size(); i++)
  {
    _leaders[i]->print();
  }
#endif //VL_DEBUG

  cout << endl;
}
