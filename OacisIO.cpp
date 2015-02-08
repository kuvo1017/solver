#include <vector>                               
#include <sstream>
#include "OacisIO.h"
#include "GVManager.h"
#include "picojson.h"
#include "FileManager.h"

using namespace std;
//======================================================================  
std::string OacisIO::inputParams(){
  // 参考：http://tsuyushiga.hatenablog.jp/entry/2014/06/04/232104
  //ファイルパスの取得
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
  GVManager::setNewNumeric("REAR_ERROR_RATE",all["rear_error_rate"].get<double>());
  GVManager::setNewNumeric("PASSING_ERROR_RATE",all["passing_error_rate"].get<double>());
  GVManager::setNewNumeric("LR_ERROR_RATE",all["lr_error_rate"].get<double>());
  GVManager::setNewNumeric("SHIFT_ERROR_RATE",all["shift_error_rate"].get<double>());
  GVManager::setNewNumeric("HEAD_ERROR_RATE",all["head_error_rate"].get<double>());
  double volume = all["small_traffic_volume"].get<double>();
  GVManager::setNewNumeric("SMALL_TRAFFIC_VOLUME",volume);
  GVManager::setNewNumeric("LARGE_TRAFFIC_VOLUME",volume * 0.3);
  GVManager::setNewNumeric("MAX_TIME",1000*all["max_time"].get<double>());
  GVManager::setNewNumeric("MAX_ACCIDENT",all["max_accident"].get<double>());
//  GVManager::setNewNumeric("LARGE_TRAFFIC_VOLUME",all["large_traffic_volume"].get<double>());
  std::string dataPath =  all["data_path"].get<std::string>(); 
  GVManager::setNewString("PARAM_NAME",all["param_name"].get<std::string>()); 
  if(all["no_input_signal"].get<bool>())
  {
  GVManager::resetFlag("FLAG_INPUT_SIGNAL",false);
  }
   std::cout << dataPath<<endl;
  return dataPath;
} 
