#include <vector>
#include "ErrorController.h"
#include "Lane.h"
#include "Random.h"
#include "TimeManager.h"
#include "Vehicle.h"
#include "VehicleIO.h"
#include "VirtualLeader.h"
#include "CollisionJudge.h"
#include "ObjManager.h"

using namespace std;
ErrorController::ErrorController(Vehicle* vehicle){
  _vehicle = vehicle;
  _isWall = false;
  _rearErrorVelocity = 0;
  _rearErrorTime = 0;
  _rearError = false;
  _rearErrorLength = 0;
  _rearErrorVelocity = 0;
  _rearErrorTime = 0;
  _slideErrorTime=0;
  _isPassingError = false;
  _isArrogance = false;
  _isLRError = false;
  _isShiftError = false;
  _isHeadError = false;
  _isShiftEnd = false;
  _velocityDifference = 0;
  _rearId = "";
  _accidentOccur = false;
  _isAccident = false;
  _accidentTime = 0;
  _isSlow=false;
  _shiftTime=0;
  _accidentTime = 0;
}
//======================================================================
VirtualLeader* ErrorController::rearError(VirtualLeader* resultLeader){
  RoadOccupant* front = _vehicle->lane()->frontAgent(dynamic_cast<RoadOccupant*>(_vehicle));
  if (front){
    Vehicle* frontVehicle = dynamic_cast<Vehicle *>(front);
    if(CollisionJudge::isCollid(_vehicle,frontVehicle)){
      accidentOccur();
      frontVehicle->errorController()->accidentOccur();
    }
    //多くの対称を認知したときに先行者の速度等を認知するかわりに予測する処理
    //追突事故の再現用
    //rearErrorをonにする
    if(!_rearError&&(_vehicle->velocity()>5.0/60.0/60.0)){
      //確率をあげるポイント
      int p=1;
      const std::vector<VirtualLeader *>* leaders = _vehicle->virtualLeaders(); 
      for(int i=0;i<leaders->size();i++){
	string type = leaders->at(i)->getType();
	if(type=="SHIFTFRONT_CAR")
	  p+=5;
	else if(type=="MERGE_CAR")
	  p+=10;
	else if(type=="RED_SIGNAL")
	  p+=1;
      }
      int x = Random::uniform(0,10000);
      if(x*p>9000){
	/*
	   cout <<"==============================" <<endl;
	   cout << "Don't watch Error is occuring" <<endl;
	   cout<<"==============================" << endl;
	 */
	errorOccur("rear-end");
	_rearError=true;
      }
    }

    if(_rearError){
      //　予測エラーの処理
      if(_rearErrorTime==0){ 
	_rearErrorLength=front->length()-front->bodyLength()/2-_vehicle->length()-_vehicle->bodyLength()/2;
	_rearErrorVelocity=front->velocity();
      }
      if(_rearErrorTime<3000){
	_rearErrorTime += TimeManager::unit();
#ifndef VL_DEBUG
	resultLeader= new VirtualLeader(_rearErrorLength, _rearErrorVelocity);
#else
	resultLeader = new VirtualLeader(_rearErrorLength, _rearErrorVelocity, resultLeader->id());
#endif

      }else{
	_rearError=false;
	_rearErrorLength=0;
	_rearErrorVelocity=0;
      }
    }
  }
  return resultLeader;
}

//======================================================================
bool ErrorController::isRearError() const{
  return _rearError;
}
//======================================================================
bool ErrorController::isLRError() const{
  return _isLRError;
}
//======================================================================
bool ErrorController::isShiftError() const{
  return _isShiftError;
}
//======================================================================
bool ErrorController::isHeadError() const{
  return _isHeadError;
}
//======================================================================
bool ErrorController::isShiftEnd() const{
  return _isShiftEnd;
}
/*
//======================================================================
void ErrorController::shifEnd() const{

_isShiftEnd=true;
}
 */
//======================================================================
bool ErrorController::isPassingError() const{
  return _isPassingError;
}
//======================================================================
bool ErrorController::isAccident() const{
  return _isAccident;
}
//======================================================================
int ErrorController::rearErrorTime() const{
  return _rearErrorTime;
} 
//======================================================================
int ErrorController::accidentTime() const{
  return _accidentTime;
}
//======================================================================
void ErrorController::accidentOccur(){
  /*
  cout << "=================================" <<endl;
  cout << "Accident occured: car id is " <<  _vehicle->id() << endl;
  cout << "=================================" <<endl;
  */
  _isAccident = true;
  _rearError=false;
  _isPassingError = false;
  _vehicle->setBodyColor(0,0,0); 
  //writeAccident();
  VehicleIO::instance().writeVehicleAccidentData(TimeManager::time(),_vehicle);
  _vehicle->stopByAccident();
}
//======================================================================
void ErrorController::errorOccur(string type){
  /*
  cout << "=================================" <<endl;
  cout << "Error occured: car id is " <<  _vehicle->id() << endl;
  cout << "=================================" <<endl;
  */
  _vehicle->setBodyColor(0.5,0.5,0);
  VehicleIO::instance().writeVehicleErrorData(TimeManager::time(),_vehicle,type);
}

//======================================================================
void ErrorController::recogWall(){
  _isWall = true;
}
//====================================================================== 
bool ErrorController::accidentCheck(){
  if(_isAccident){
    if(_accidentTime<7000){
      _accidentTime+=TimeManager::unit();
    return true;
    }else{
      return false;
    }
  }else{
  return true;
  }
}    

