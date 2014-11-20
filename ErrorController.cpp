#include <vector>
#include <sstream>
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

bool ErrorController::_isRearOn = false;
bool ErrorController::_isPassingOn = false; 
bool ErrorController::_isLROn = false;
bool ErrorController::_isSlideOn = false;
bool ErrorController::_isHeadOn = false; 
int ErrorController::_stopNAccident = 100;
int ErrorController::_maxTotal = 1000*1000*5/5.0;
bool ErrorController::_stopRun = false;

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
  _type = "not_error";
  double r = Random::uniform();
  if(r < GVManager::getNumeric("ARROGANCE_LR"))
    _isArrogance =true;
  else
    _isArrogance = false;
}
//======================================================================
VirtualLeader* ErrorController::rearError(VirtualLeader* resultLeader){
  if(!_isRearOn)
    return resultLeader;
  RoadOccupant* front = _vehicle->lane()->frontAgent(dynamic_cast<RoadOccupant*>(_vehicle));
  if (front){
    Vehicle* frontVehicle = dynamic_cast<Vehicle *>(front);
    if(CollisionJudge::isCollid(_vehicle,frontVehicle)){
      accidentOccur("rear-end");
      frontVehicle->errorController()->accidentOccur("rear-end");
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
void ErrorController::LRError(double thisTti,double thatTtp) {
  if(!_isArrogance||_isLRError)
    return;         
  // 現在位置から交差点を通過仕切るまでの時間[秒]
  double thisTtp;
  // [msec]->[sec]
  thatTtp = thatTtp/1000;
  // 現在いるレーンの次のレーン、つまり交差点内のレーン
  Lane* thisNextLane = _vehicle->localRoute().next(_vehicle->lane());
  if(thisNextLane!=NULL)
    thisTtp = thisTti + thisNextLane->length()/GVManager::getNumeric("VELOCITY_AT_TURNING_LEFT")*3.6;
  // 誤差時間
  double mistakeTime = 2.0;
  if(thisTtp<thatTtp+mistakeTime)
  {

    RelativeDirection turning = _vehicle->localRoute().turning(); 
    std::cout << "thisTtp: "<< thisTtp << "[s] thatTtp: "<< thatTtp <<"[s]"<< " Turning is "<<turning <<endl;
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

    _isLRError = true;
  }
  else
  {
    _isLRError = false;
  }
}

//======================================================================
void ErrorController::LRError(Vehicle* thatV,double thisTti,double thatTti) {
  if(!_isArrogance||_isLRError)
    return;         
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

    RelativeDirection turning = _vehicle->localRoute().turning(); 
    std::cout << "thisTtp: "<< thisTtp << "[s] thatTtp: "<< thatTtp <<"[s]"<< " Turning is "<<turning <<endl;
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
    _isLRError = true;
  }
  else
  {
    _isLRError = false;
  }
}
//======================================================================
bool ErrorController::headError(){
  //多くの対称を認知したときに先行者の速度等を認知するかわりに予測する処理
  //追突事故の再現用
  //をonにする
  //確率をあげるポイント
  if(!_isHeadOn)
    return false;
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
    int x = Random::uniform(0,10000);
    if(x*p<GVManager::getNumeric("NOLOOK_HEAD")){
      errorOccur("head");
      _isHeadError=true;
    }
  }
  return _isHeadError;
}
//======================================================================
double ErrorController::errorVelocity() 
{
  {
    double error = _vehicle->error();
    if(_isHeadError ){

      if(error==0.0)
	return -2.0/60.0/60.0;
      else if(error<-3.0)
	return 2.0/60.0/60.0;
      else if(error>0)
	return 0.0;
    }
  }
}

//======================================================================
void ErrorController::checkHeadAccident()
{
  {
    Section* section = _vehicle->section();
    if(!_isAccident && section != NULL){
      const map<string, Lane*, less<string> >* lanes = _vehicle->laneBundle()->lanes();
      map<string, Lane*, less<string> >::const_iterator  ite = lanes->begin();
      // 自分の方向
      int myDirection = _vehicle->section()->isUp(_vehicle->lane());
      //自分の対向車線
      Lane* onComingLane=NULL;
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

      if(onComingLane!=NULL){
	Vehicle* frontSideVehicle = onComingLane->followingVehicle(_vehicle->lane()->length()-_vehicle->length());
	if(frontSideVehicle!=NULL){
	  if(CollisionJudge::isCollid(_vehicle,frontSideVehicle)){
	    accidentOccur("head");
	    frontSideVehicle->errorController()->accidentOccur("head");
	    //_velocity -> _errorVelocity=0.0;
	    _isHeadError=false;
	  }
	}
      }
    }
  }   
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
  string ErrorController::type() const{
    return _type;
  }
  //======================================================================
  void ErrorController::accidentOccur(std::string type){

    cout << "=================================" <<endl;
    cout << "Accident occured: car id is " <<  _vehicle->id() << endl;
    cout << "error type:" << type <<endl;
    cout << "=================================" <<endl;

    _isAccident = true;
    _rearError=false;
    _isPassingError = false;
    _vehicle->setBodyColor(0,0,0); 
    //writeAccident();
    VehicleIO::instance().writeVehicleAccidentData(TimeManager::time(),_vehicle);
  }
  //======================================================================
  void ErrorController::errorOccur(string type){
    _type = type;
    cout << "=================================" <<endl;
    cout << "Error occured: car id is " <<  _vehicle->id() << endl;
    cout << "error type:"<<type <<endl; 
    cout << "=================================" <<endl;
    _vehicle->setBodyColor(0.8,0.8,0);
    VehicleIO::instance().writeVehicleErrorData(TimeManager::time(),_vehicle);
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
  //======================================================================  
  bool ErrorController::initErrorParams(){
    // 参考：http://tsuyushiga.hatenablog.jp/entry/2014/06/04/232104
    //ファイルパスの取得
    return "../simulations/LRError/";
    string s1,s2;
    string path = "./_input.json";

    // ファイルオープン
    ifstream inputStream;
    string thisLine;
    inputStream.open(path);
    if (!inputStream.is_open())
    {
      cerr << "cannot open file!" << endl;
      exit(1);
    }

    stringstream sstream;
    while (getline(inputStream, thisLine))
    {
      sstream << thisLine;
    }
    inputStream.close();
    cout << "finish opening file!" << endl;

    // CCLOG("sstream:%s", sstream.str().c_str());

    // JSONのパース
    picojson::value v; 
    picojson::parse(v, sstream);

    picojson::object& all = v.get<picojson::object>();
    /*
       picojson::array& array = all["hoge"].get<picojson::array>();
       for (picojson::array::iterator it = array.begin(); it != array.end(); it++)
       {
       picojson::object& all = it->get<picojson::object>();
     */
    GVManager::setNewNumeric("NOLOOK_REAR",0.0);
    GVManager::setNewNumeric("ARROGANCE_PASSING",0.0);
    GVManager::setNewNumeric("NOLOOK_SHIFT",0.0);
    GVManager::setNewNumeric("NOLOOK_HEAD",0.0);
    /*
       GVManager::setNewNumeric("NOLOOK_REAR",all["nolook_rear"].get<double>());
       GVManager::setNewNumeric("ARROGANCE_PASSING",all["arrogance_passing"].get<double>());
       GVManager::setNewNumeric("ARROGANCE_LR",all["arrogance_LR"].get<double>());
       GVManager::setNewNumeric("NOLOOK_SHIFT",all["nolook_shift"].get<double>());
       GVManager::setNewNumeric("NOLOOK_HEAD",all["nolook_head"].get<double>());
     */
    // CCLOG("x:%d, y:%d, z:%d", x, y, z);
    //}
  } 
  //======================================================================   
  std::string ErrorController::setDataPath(){
    // 参考：http://tsuyushiga.hatenablog.jp/entry/2014/06/04/232104
    //ファイルパスの取得
     std::string dataPath = "../simulations/LRError/";
    //std::string dataPath = "../simulations/rearError/";
    return dataPath;
/*
    const char* path = "./_input.json";

    // ファイルオープン
    ifstream inputStream;
    string thisLine;
    inputStream.open(path);
    if (!inputStream.is_open())
    {
      cerr << "cannot open file!" << endl;
      exit(1);
    }

    stringstream sstream;
    while (getline(inputStream, thisLine))
    {
      sstream << thisLine;
    }
    inputStream.close();
    cout << "finish opening file!" << endl;

    // CCLOG("sstream:%s", sstream.str().c_str());

    // JSONのパース
    picojson::value v; 
    picojson::parse(v, sstream);
    picojson::object& all = v.get<picojson::object>(); 
    std::string dataPath = (std::string) all["data_path"].get<std::string>().c_str(); 
    cout << dataPath <<endl;
    return dataPath;
    */
  }

  //======================================================================    
  void ErrorController::checkStatData(){
    vector<DetectorUnit*>* detectors = ObjManager::detectorUnits();
    cout << detectors->size() << endl;
    if(detectors->size() > 0)
    {
      int totalP =0;
      int totalT =0;
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
	<< "エラー率：" << GVManager::getNumeric("ARROGANCE_LR") << "\n"  
	<< "計算時間:" <<TimeManager::getTime("TOTALRUN")<<"\n"
	<< "発生小型車両台数:" << totalP<< "\n"
	<< "発生大型車両台数:" << totalT<< "\n" 
	<< "発生事故数:" << GVManager::getNumeric("ACCIDENT_COUNT") << "\n" 
	<< "==============================="<<endl; 
      writeStatData(totalP,totalT);
      if(totalP+totalT> _maxTotal || _stopNAccident < GVManager::getNumeric("ACCIDENT_COUNT"))
	_stopRun = true;
    }else
    {
      cerr << "no detector file" <<endl;
    }   
  }
  //======================================================================
  void ErrorController::writeStatData(int totalP,int totalT){
    // error.txtのオープン
    string file;
    GVManager::getVariable("RESULT_STAT_ACCIDENT_FILE", &file);
    ofstream& ofsGD1 = FileManager::getOFStream(file);
    // オープンに失敗した場合は関数内で落ちるはず。
    // 車両台数等の動的グローバル情報の書き出し
    ofsGD1 << TimeManager::getTime("TOTALRUN")<<"," 
      << TimeManager::time()/1000 << ","
      <<  totalP<< ","
      <<  totalT<< ","
      << GVManager::getNumeric("ACCIDENT_COUNT") << ","
      << endl;
  }
  //====================================================================== 
  bool ErrorController::stopRun(){
    return _stopRun;
  }

