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
  GVManager::setNewNumeric("NOLOOK_REAR",all["nolook_rear"].get<double>());
  GVManager::setNewNumeric("ARROGANCE_PASSING",all["arrogance_passing"].get<double>());
  GVManager::setNewNumeric("ARROGANCE_LR",all["arrogance_LR"].get<double>());
  GVManager::setNewNumeric("NOLOOK_SHIFT",all["nolook_shift"].get<double>());
  GVManager::setNewNumeric("NOLOOK_HEAD",all["nolook_head"].get<double>());
  GVManager::setNewNumeric("SMALL_TRAFFIC_VOLUME",all["small_traffic_volume"].get<double>());
  GVManager::setNewNumeric("LARGE_TRAFFIC_VOLUME",all["large_traffic_volume"].get<double>());
  cout << "nolook_rear is " <<GVManager::getNumeric("NOLOOK_REAR") <<endl;
  return "../simulations/lr-error/LRError/";
} 
