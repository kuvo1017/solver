#include <vector>
#include <sstream>
#include <time.h>
#include "ErrorController.h"
#include "Lane.h"
#include "Random.h"
#include "TimeManager.h"
#include "Vehicle.h"
#include "VehicleIO.h"
#include "VirtualLeader.h"
#include "CollisionJudge.h"
#include "ObjManager.h"
#include "DetectorUnit.h"
#include "GVManager.h"
#include "picojson.h"
#include "FileManager.h"
#include "VehicleFamilyManager.h"

int ErrorController::_stopNAccident = 100;
bool ErrorController::_stopRun = false;
bool ErrorController::_initWrite = true;
time_t ErrorController::_startTime = time(NULL);
 
using namespace std;

ErrorController::ErrorController(Vehicle* vehicle){
  _vehicle = vehicle;
  _rearErrorVelocity = 0;
  _rearErrorTime = 0;
  _isRearError = false;
  _rearErrorLength = 0;
  _rearErrorVelocity = 0;
  _rearErrorTime = 0;
  _headErrorTime=0;
  _isPassingError = false;
  _isArroganceLR = false;
  _isLRError = false;
  _isShiftError = false;
  _isHeadError = false;
  _rearId = "";
  _accidentOccur = false;
  _isAccident = false;
  _accidentTime = 0;
  _shiftTime=0;
  _accidentTime = 0;
  _errorType = "NOT_ERROR";
  _errorVelocity = 0.0;
  std::vector<Vehicle*> _invisibleVehicles;

}
//======================================================================
void ErrorController::chooseError()
{
  int nCand = _candidates.size();
  if(nCand>1)
  {
    double rand = Random::uniform()*nCand;
    for(int i=0;i<nCand;i++)
    {
      if(i <= rand && i+1 > rand)
      {
	if(_candidates[i] == "REAR")
	{
	  _isRearError = true;
	  _errorType = "REAR";
	}else if(_candidates[i] == "SHIFT")
	{
	  _isShiftError = true;
	  _errorType = "SHIFT";
	}else if(_candidates[i] == "HEAD")
	{
	  _isHeadError = true;
	  _errorType = "HEAD";
	}
      }
    }
    _errorOccur();
  }else if(nCand == 1)
  {
    if(_candidates[0] == "REAR"){
      _isRearError = true;
      _errorType = "REAR";
    }else if(_candidates[0] == "SHIFT")
    {
      _isShiftError = true;
      _errorType = "SHIFT";
    }else if(_candidates[0] == "HEAD")
    {
      _isHeadError = true;
      _errorType = "HEAD";
    }
    _errorOccur();
  }
}
//======================================================================
VirtualLeader* ErrorController::rearError(VirtualLeader* resultLeader){
 RoadOccupant* front = _vehicle->lane()->frontAgent(dynamic_cast<RoadOccupant*>(_vehicle));
  if (front){
    Vehicle* frontVehicle = dynamic_cast<Vehicle *>(front);
    if(GVManager::getNumeric("REAR_ERROR_RATE") == 0 || _isAccident)
      return resultLeader;
    // 事故が起きているかを判断
    if(_isRearError)
    {
      CollisionJudge::isFrontCollid(_vehicle,frontVehicle);
    }
    //多くの対称を認知したときに先行者の速度等を認知するかわりに予測する処理
    //rearErrorをonにする
    if(!_isRearError&&(_vehicle->velocity() > 5.0/60.0/60.0)){
      if(_objectPoint() < GVManager::getNumeric("REAR_ERROR_RATE")){
        _candidates.push_back("REAR");
      }
    }
    if(_isRearError){
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
        _isRearError=false;
        _rearErrorLength=0;
        _rearErrorVelocity=0;
	_rearErrorTime = 0;
        _errorEnd();
      }
    }
  }
  return resultLeader;
}
//======================================================================
double ErrorController::_objectPoint() 
{
  double point = 1;
  double x = Random::uniform();
  const std::vector<VirtualLeader *>* leaders = _vehicle->virtualLeaders(); 
  for(int i=0;i<leaders->size();i++){
    string type = leaders->at(i)->getType();
    if(type=="SHIFTFRONT_CAR")
      point+=5.0;
    else if(type=="MERGE_CAR")
      point+=10.0;
    else if(type=="RED_SIGNAL")
      point+=1.0;
  }
  return point*x;
} 

//======================================================================
void ErrorController::setInvisibleVehicle(Vehicle* vehicle)
{
//  cout << "set" <<endl;
  _invisibleVehicles.push_back(vehicle);
}

//======================================================================
void ErrorController::LRError(Vehicle* thatV,double thisTti,double thatTti) {
  if(!_isArroganceLR)
  {
    return;
  }
 // 現在位置から交差点を通過仕切るまでの時間[秒]
  double thisTtp,thatTtp;
  // 現在いるレーンの次のレーン、つまり交差点内のレーン
  Lane* thisNextLane = _vehicle->localRoute().next(_vehicle->lane());
  Lane* thatNextLane = thatV->localRoute().next(thatV->lane()); 
  if(thisNextLane!=NULL)
    thisTtp = thisTti/1000 + thisNextLane->length()/GVManager::getNumeric("VELOCITY_AT_TURNING_LEFT")*3.6;
  if(thatNextLane!=NULL)
    thatTtp = thatTti/1000 + thatNextLane->length()/GVManager::getNumeric("VELOCITY_AT_TURNING_LEFT")*3.6;
  // 誤差時間
  double mistakeTime = 3.0;
  if(thisTtp<thatTtp+mistakeTime)
  {
    /*
    RelativeDirection turning = _vehicle->localRoute().turning(); 
    switch(turning){
    case 2:
      errorOccur("RightError");
      break;
    case 4:
      errorOccur("StraightError");
      break;
    case 8:
      errorOccur("LeftError");
      break;
    default:
      break;
    }
    */
    _isLRError = true;
    _errorOccur();
  }
  else
  {
    _isLRError = false;
  }
}
//======================================================================
bool ErrorController::headError(){
 if(_isHeadError)
  {   
    if(_checkHeadAccident()){
      return false;
    }
    return true;
  }
 // 速度があまりに小さい時にはハンドルを
 // 切り間違えても対向車線に進入することはないので
 if(
      GVManager::getNumeric("HEAD_ERROR_RATE") == 0
      || (_onComingLane()==nullptr)
      || _isAccident
      || _vehicle->velocity() < 20.0/3600.0 )
  {
    return false;
  }
  if(_objectPoint() < GVManager::getNumeric("HEAD_ERROR_RATE")){
    _candidates.push_back("HEAD");
  }
}
//======================================================================
double ErrorController::errorVelocity() 
{
  if(_isAccident)
  {
    return 0.0;
  }
  double error = _vehicle->error();
  if(_isHeadError ){
    if(error < -5.0 )
    {
       _errorVelocity =  5.0/60.0/60.0;
    _headErrorTime+=TimeManager::unit();
    }
    else if(_headErrorTime > 100 && (error > 0))
    {
      _vehicle->returnError();
      _errorVelocity =  0.0;
      _headErrorTime = 0;
      _isHeadError = false;
      _errorEnd();

    }
    else if(_headErrorTime ==0)
    {
      _errorVelocity = -5.0/60.0/60.0;
      _headErrorTime+=TimeManager::unit();
    }
  }
  return _errorVelocity;
}

//======================================================================
bool ErrorController::_checkHeadAccident()
{
  Lane* onComingLane = _onComingLane();  
  if(onComingLane!=NULL){
    Vehicle* frontSideVehicle = onComingLane->followingVehicle(_vehicle->lane()->length()-_vehicle->length());
    if(frontSideVehicle!=NULL){
      if(CollisionJudge::isHeadCollid(_vehicle,frontSideVehicle)){
        _isHeadError=false;
        return true;
      }
    }
  }
  return false;
}
//====================================================================== 
Lane* ErrorController::_onComingLane()
{
  Lane* onComingLane=NULL;
  Section* section = _vehicle->section();
  if(!_isAccident 
      && section != NULL)
  {
    // 自車が最も右側の車線にいるかをチェック
    // もしいなければ、正面衝突が起こらないため
    Lane* rightLane = NULL;
    double rightLaneLength =0.0;
    Lane* myLane = _vehicle->lane();
    section->getRightSideLane(myLane,myLane->length(),&rightLane,&rightLaneLength);
    if(rightLaneLength != 0)
      return onComingLane;
    const map<string, Lane*, less<string> >* lanes = _vehicle->laneBundle()->lanes();
    map<string, Lane*, less<string> >::const_iterator  ite = lanes->begin();
    // 自分の方向
    int myDirection = _vehicle->section()->isUp(_vehicle->lane());
    //自分の対向車線
    while (ite != lanes->end()) {
      Lane* lane = ite->second;
      if(section->isUp(lane)!=myDirection){
        // 右車線
        Lane* rl = NULL;
        // 右車線の長さ
        double rll;
        section->getRightSideLane(lane,lane->length(),&rl,&rll);
        if(rl==NULL){
          onComingLane = lane;
        }
      }
      ite++;
    }
  } 
  return onComingLane;
}

//====================================================================== 
bool ErrorController::shiftError()
{
  if(_vehicle->velocity() < 20.0/60.0/60.0)
  {
    return false;
  }
  if(_objectPoint() < GVManager::getNumeric("SHIFT_ERROR_RATE"))
  {
    _candidates.push_back("SHIFT");
    return true;
  }else{
    return false;
  }
}
//====================================================================== 
void ErrorController::endShiftError()
{
  if(_isShiftError)
  {
    _isShiftError = false;
    _errorEnd();
  }
}

//======================================================================
bool ErrorController::isRearError() const{
  return _isRearError;
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
const std::vector<Vehicle*>* ErrorController::invisibleVehicles() const{
  return &_invisibleVehicles;
} 
//======================================================================
int ErrorController::accidentTime() const{
  return _accidentTime;
}
//======================================================================
void ErrorController::errorInIntersection(Intersection* next)
{
  /**
  * 最初に2つのエラーが発生するかどうかを別々に計算する
  * （エラー率による影響を正確に反映させるため）
  * 仮に両方のエラーが同時に発生したときにはどちらか片方を選ぶ
  */
  _isPassingError = false;
  _isLRError = false;
  /**
   * 出会い頭のエラーが発生するか計算
   * 次に進入する交差点に障害物がない場合は計算しない。
   * より計算負荷を減らすには
   * 交差点に遮蔽物があるかを問い合わせるのではなく、
   * 自車が見通しの悪い単ろにいるかを問い合わせるべき
   */
  if(next->barriers().size() != 0)
  {
    if(GVManager::getNumeric("LR_ERROR_RATE") != 0)
    {
      if( Random::uniform() < GVManager::getNumeric("PASSING_ERROR_RATE"))
      { 
	_isPassingError = true; 
      }                  
    }
  }
  // 右左折エラーにおける傲慢なエージェントかどうかを確率的に計算
  if(GVManager::getNumeric("LR_ERROR_RATE") != 0)
  {
    if(Random::uniform() < GVManager::getNumeric("LR_ERROR_RATE"))
    {
      _isArroganceLR = true;
    }
  }       
  // 出会い頭と右左折の両方が発生している時にどちらか一方のみを選択
  if(_isPassingError ||_isArroganceLR )
  {
    if(Random::uniform() > 0.5)
    {
      _isPassingError = false;
    }else
    {
      _isArroganceLR = false;
      _errorOccur();  
    }
  }
}

//======================================================================
string ErrorController::errorType() const{
  if(_vehicle->section() == NULL)
  {
    return _errorType;
  }else{
    if(_isPassingError)
    {
      return "PASSING";
    }else if(_isLRError)
    {
      return "LR";
    }
  }
  return "NOT_ERROR";
}
//======================================================================
void ErrorController::accidentOccur(std::string collidType){

  cout << "=================================" <<endl;
  cout << "Accident occured: car id is " <<  _vehicle->id() << endl;
  cout << "=================================" <<endl;
  _isAccident = true;
  _vehicle->setBodyColor(0,0,0); 
  VehicleIO::instance().writeVehicleAccidentData(TimeManager::time(),_vehicle,collidType);
}
//======================================================================
void ErrorController::_errorOccur(){
  _vehicle->setBodyColor(1.0,0.0,0);
  VehicleIO::instance().writeVehicleErrorData(TimeManager::time(),_vehicle);
}
 //======================================================================
void ErrorController::resetInvisibleVehicles(){
  _invisibleVehicles.clear();
}
//======================================================================
void ErrorController::_errorEnd(){
 VFAttribute* vfa
    = VehicleFamilyManager::vehicleFamilyAttribute(_vehicle->type());
  if (!vfa)
  {
    if (VehicleFamily::isTruck(_vehicle->type()))
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
  double r,g,b;
  vfa->getBodyColor(&r, &g, &b);
  _vehicle->setBodyColor(r,g,b);
  _errorType = "NOT_ERROR";
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

//======================================================================    
void ErrorController::checkStatData(){
  vector<DetectorUnit*>* detectors = ObjManager::detectorUnits();
  if(detectors->size() > 0)
  {
    int totalP =0;
    int totalT =0;
    TimeManager::stopClock("ERROR_MODE");
    string  time = std::to_string(TimeManager::getTime("ERROR_MODE")); 
    TimeManager::startClock("ERROR_MODE");

    
    for(int i=0;i<detectors->size();i++)
    {
      DetectorUnit* detector = detectors->at(i);
      detector->monitorLanes();
      DetectorUnit::StatVehicleData svd = detector->statVehicleData(); 
      totalP+=svd.totalAllPassengers;
      totalT+=svd.totalAllTrucks; 
    }

    cout << "===============================\n"
      << "statitic accident data\n" 
      << "計算時間:" <<time <<"\n"
      << "発生小型車両台数:" << totalP<< "\n"
      << "発生大型車両台数:" << totalT<< "\n" 
      << "発生事故数:" << GVManager::getNumeric("ACCIDENT_COUNT") << "\n" 
      << "==============================="<<endl; 
    if( GVManager::getNumeric("MAX_ACCIDENT") < GVManager::getNumeric("ACCIDENT_COUNT")
	|| TimeManager::time() >= GVManager::getNumeric("MAX_TIME"))
    {
      _stopRun = true;
      TimeManager::stopClock("ERROR_MODE");
      time = std::to_string(TimeManager::getTime("ERROR_MODE")); 
    }
    writeStatData(totalP,totalT,time);
  }else
  {
    cerr << "no detector file" <<endl;
  }   
}
//======================================================================
void ErrorController::writeStatData(int totalP,int totalT,string time){
  // error.txtのオープン
  string file;
  GVManager::getVariable("RESULT_STAT_ACCIDENT_FILE", &file);
  ofstream& ofsGD1 = FileManager::getOFStream(file);
  // オープンに失敗した場合は関数内で落ちるはず。
  // 車両台数等の動的グローバル情報の書き出し
  if(!_initWrite)
  {
  ofsGD1 << time<<"," 
    << TimeManager::time()/1000 << ","
    <<  totalP<< ","
    <<  totalT<< ","
    << GVManager::getNumeric("ACCIDENT_COUNT") << ","
      << GVManager::getNumeric("VEHICLE_EXIST_COUNT") 
      << endl;
    }else
    {
   ofsGD1 << "#rear:" << GVManager::getNumeric("REAR_ERROR_RATE")
     << " passing:" << GVManager::getNumeric("PASSING_ERROR_RATE")
     << " lr:" << GVManager::getNumeric("LR_ERROR_RATE") 
     << " shift:" << GVManager::getNumeric("SHIFT_ERROR_RATE") 
     << " head:" << GVManager::getNumeric("HEAD_ERROR_RATE") << "\n"
     << time<<"," 
     << TimeManager::time()/1000 << ","
     <<  totalP<< ","
    <<  totalT<< ","
    << GVManager::getNumeric("ACCIDENT_COUNT") << ","
     << GVManager::getNumeric("VEHICLE_EXIST_COUNT") 
    << endl;
    _initWrite = false;
    }
}
//====================================================================== 
bool ErrorController::stopRun(){
  return _stopRun;
}

