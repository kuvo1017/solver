#include "VehicleIO.h"
#include "Vehicle.h"
#include "ObjManager.h"
#include "Lane.h"
#include "LaneBundle.h"
#include "GVManager.h"
#include "FileManager.h"
#include "AmuConverter.h"
#include "AmuVector.h"
#include "Conf.h"
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#ifdef USE_MINGW
#include <io.h>
#else // USE_MINGW
#include <sys/types.h>
#include <sys/stat.h>
#endif // USE_MINGW
#include <cassert>
#include <cmath>
#include <sstream>

#define NUM_FIGURE_FOR_TIMELINE_FILENAME 10

#ifdef USE_ZLIB
#include <zlib.h>
#define fzopen   gzopen
#define fzclose  gzclose 
#define fzprintf gzprintf
#define fz_ok    Z_OK
#else // USE_ZLIB
#define fzopen   fopen
#define fzclose  fclose
#define fzprintf fprintf
#define fz_ok    0
#endif // USE_ZLIB

using namespace std;

const string VehicleIO::_timePrefix = "#Time=";

//======================================================================
VehicleIO::VehicleIO()
{
    string resultDir, resultFile, distanceFile;
    GVManager::getVariable("RESULT_OUTPUT_DIRECTORY", &resultDir);
    GVManager::getVariable("RESULT_VEHICLE_STATIC_FILE", &resultFile);
    GVManager::getVariable("RESULT_VEHICLE_DISTANCE_FILE", &distanceFile);
    _extension
        = static_cast<int>(
            GVManager::getNumeric("OUTPUT_VEHICLE_EXTENSION"));

    _staticOutFileName = resultDir+resultFile;
    _distanceOutFileName  = resultDir+distanceFile;

    // 以前の結果を消去する
    _staticOut.open(_staticOutFileName.c_str(), ios::trunc);
    if (_staticOut.good())
    {
        _staticOut.close();
    }
    _distanceOut.open(_distanceOutFileName.c_str(), ios::trunc);
    if (_distanceOut.good())
    {
        _distanceOut.close();
    }
}

//======================================================================
VehicleIO& VehicleIO::instance()
{
    static VehicleIO instance;
    return instance;
}

//======================================================================
bool VehicleIO::writeVehiclesDynamicData(const ulint time,
					 vector<Vehicle*>* vehicles)
{
    // (可視)車両のカウンター
    int nVisible = 0;
    int nAll = 0;

    // ファイル名を決定する
    vector<string> paths;

    string resultDir;
    GVManager::getVariable("RESULT_TIMELINE_DIRECTORY", &resultDir);
    paths.push_back(resultDir+"vehicle/");

    string strTime
        = AmuConverter::itos(time,
                             NUM_FIGURE_FOR_TIMELINE_FILENAME); 
    paths.push_back(strTime.substr(0,3)+"/");
    paths.push_back(strTime.substr(3,3)+"/");
#ifdef USE_ZLIB
    paths.push_back(strTime.substr(6,4)+".txt.gz");
#else // USE_ZLIB
    paths.push_back(strTime.substr(6,4)+".txt");
#endif // USE_ZLIB

    string dynamicPath = "";
    for (unsigned int i=0; i<paths.size(); i++)
    {
        dynamicPath += paths[i];
    }

    // 車両情報ファイルを開く
#ifdef USE_ZLIB
    _dynamicOut = fzopen(dynamicPath.c_str(), "wb6f");
#else // USE_ZLIB
    _dynamicOut = fzopen(dynamicPath.c_str(), "w");
#endif // USE_ZLIB
    if (_dynamicOut==NULL)
    {
        // ディレクトリが準備されていない場合には作成を試みる
        paths.pop_back();
        if(_makeDirectories(paths))
        {
#ifdef USE_ZLIB
            _dynamicOut = fzopen(dynamicPath.c_str(), "wb6f");
#else // USE_ZLIB
            _dynamicOut = fzopen(dynamicPath.c_str(), "w");
#endif // USE_ZLIB
        }
    }
    // ディレクトリを作ろうとしてもファイルが開けない場合には
    // 諦めて落ちる。
    if (_dynamicOut==NULL)
    {
        cerr << "cannot open file:" << dynamicPath << endl;
        assert(0);
        exit(EXIT_FAILURE);
    }
    // 車両情報ファイルの出力
    if (GVManager::getNumeric("OUTPUT_COMMENT_IN_FILE")!=0)
    {
        fzprintf(_dynamicOut, "%s\n", _timePrefix.c_str());
    }
    for (int i=0; i<static_cast<signed int>((*vehicles).size()); i++)
    {
        // 車両データの出力
        /* 発生直後の自動車は速度が非現実的であると言う理由から
         * 出力されない場合がある．
         */
        if ((*vehicles)[i]->isAwayFromOriginNode())
        {
            _writeVehicleDynamicData((*vehicles)[i]);
            nVisible++;
        }
    }
    int fzcl = fzclose(_dynamicOut);
    if (fzcl != fz_ok)
    {
        cerr << "cannot close file:" << dynamicPath << endl;
        exit(EXIT_FAILURE);
    }

    // global_dynamic.txtのオープン
    string fGlobalDynamic;
    GVManager::getVariable("RESULT_GLOBAL_DYNAMIC_FILE", &fGlobalDynamic);
    ofstream& ofsGD = FileManager::getOFStream(fGlobalDynamic);
    // オープンに失敗した場合は関数内で落ちるはず。

    // 車両台数等の動的グローバル情報の書き出し
    nAll = static_cast<signed int>((*vehicles).size());

    ofsGD << time << "," << nAll << "," << nVisible << endl;

    return true;
}

//======================================================================
bool VehicleIO::writeVehicleAccidentData(ulint time,Vehicle* vehicle) {
  bool result = false;
  // accident.txtのオープン
  string fAccident[2];
  GVManager::getVariable("RESULT_ACCIDENT_FILE", &fAccident[0]);
  GVManager::getVariable("RESULT_ACCIDENT_FILE_QUVO", &fAccident[1]);
  GVManager::setNewNumeric("ACCIDENT_COUNT",GVManager::getNumeric("ACCIDENT_COUNT")+1);

  for(int i=0;i<2;i++){
  ofstream& ofsGD1 = FileManager::getOFStream(fAccident[i]); // オープンに失敗した場合は関数内で落ちるはず。
  // 車両台数等の動的グローバル情報の書き出し
  ofsGD1 << time/1000 << ","
         <<GVManager::getNumeric("ACCIDENT_COUNT") << "," <<
    vehicle->id() << ","<<
    vehicle->x() << ","<<
    vehicle->y() << ","<< endl;
  }
  return result;
}
//======================================================================
bool VehicleIO::writeVehicleErrorData(ulint time, Vehicle* vehicle,string type) {
  bool result = false;
  // accident.txtのオープン
  string fError[2];
  GVManager::getVariable("RESULT_ERROR_FILE", &fError[0]);
  GVManager::getVariable("RESULT_ERROR_FILE_QUVO", &fError[1]);
  GVManager::setNewNumeric("ACCIDENT_COUNT",GVManager::getNumeric("ACCIDENT_COUNT")+1); 
  for(int i=0;i<2;i++){
    ofstream& ofsGD1 = FileManager::getOFStream(fError[i]); // オープンに失敗した場合は関数内で落ちるはず。
    // 車両台数等の動的グローバル情報の書き出し
    ofsGD1 <<time/1000 << ","
      <<GVManager::getNumeric("ACCIDENT_COUNT") << "," << 
     vehicle->id() << ","<<
      vehicle->x() << ","<<
      vehicle->y() << ","<<
      type << endl;
  }
  return result;
}
 
//======================================================================
bool VehicleIO::_writeVehicleDynamicData(Vehicle* vehicle)
{
    assert(_dynamicOut);

    // 角度の計算
    /*
     * theta = xy平面に投影したときの(0,1,0)方向との間の角
     * (反時計回りを正とする)
     *
     * phi   = 勾配.xy平面となす角(上り坂が正，下り坂が負となる)
     */
    double theta = vehicle->directionVector().calcAngle
        (AmuVector(0,1,0))*180*M_1_PI;
    // -pi<=theta<piであるので，0<=theta<2piに変更
    if (theta<0)
        theta += 360;
    double phi = atan(vehicle->lane()->gradient()/100.0)*180*M_1_PI;
  
    stringstream ss;
    ss.str("");
    ss << vehicle->id() << ","
       << vehicle->type() << ","
       << vehicle->x() << ","
       << vehicle->y() << ","
       << vehicle->z() << ","
       << theta << ","
       << phi << ","
       << vehicle->velocity()*1000 << ","
       << vehicle->accel()*1.0e+6 << ",";
    if (vehicle->intersection()!=NULL)
    {
        ss << vehicle->intersection()->id() << ",";
    }
    else
    {
        ss << "NULL,";
    }
    if (vehicle->section()!=NULL)
    {
        ss << vehicle->section()->id() << ",";
    }
    else
    {
        ss << "NULL,";
    }

    // 追加情報出力、0:なし、1:ウィンカ、2:先行者ID
    if (_extension >= 1)
    {
        ss << vehicle->blinker().state() << ",";
        if (_extension >= 2)
        {
            int leaderSize = vehicle->virtualLeaders()->size();
            ss << leaderSize << ",";
            for (int i = 0; i < leaderSize ; i++)
            {
                ss << vehicle->virtualLeaders()->at(i)->id() << ",";
            }
        }
    }

    fzprintf(_dynamicOut, "%s\n", ss.str().c_str());

    return true;
}

//======================================================================
bool VehicleIO::_makeDirectories(vector<string> paths) const
{
    bool result = false;

    string tmpPath = "";
    for (unsigned int i=0; i<paths.size(); i++)
    {
        tmpPath += paths[i];
    }

#ifndef USE_MINGW
    // パーミッション設定
    mode_t mode = S_IRUSR | S_IRGRP | S_IXUSR
        | S_IXGRP | S_IWUSR | S_IWGRP;
#endif //USE_MINGW

    // ディレクトリ作成
#ifdef USE_MINGW
    if (_mkdir(tmpPath.c_str())!=0)
#else
        if (mkdir(tmpPath.c_str(), mode)!=0)
#endif
        {
            /*
             * mkdirは成功したときに0，失敗したときに-1を返し，
             * 失敗したときはエラーコードが変数errno(cerrnoで宣言)に
             * 格納される．
             * man 2 mkdir を参照．
             */
            if (errno==ENOENT && paths.size()>=2)
            {
                // tmpPathの中のどれかのディレクトリが存在しない場合には
                // 成功するまで(可能な限り)上の階層にもどる
                paths.pop_back();
#ifdef USE_MINGW
                if(_makeDirectories(paths)
                   && _mkdir(tmpPath.c_str())==0)
#else
                    if(_makeDirectories(paths)
                       && mkdir(tmpPath.c_str(), mode)==0)
#endif
                    {
                        result = true;
                    }
            }
            else
            {
                cerr << errno << endl;
                cerr << "cannot make directory: " << tmpPath << endl;
                assert(0);
            }
        }
        else {
            result = true;
        }
    return result;
}

//======================================================================
bool VehicleIO::writeVehicleStaticData(Vehicle* vehicle) 
{
    bool result = false;

    _staticOut.open(_staticOutFileName.c_str(), ios::app);
    if (_staticOut.good())
    {
        _staticOut << vehicle->id() << ","
                   << vehicle->type() << ","
                   << vehicle->bodyLength() << ","
                   << vehicle->bodyWidth() << ","
                   << vehicle->bodyHeight() << endl;

        _staticOut.close();
        result = true;
    }
    return result;
}

//======================================================================
bool VehicleIO::writeVehicleDistanceData(Vehicle* vehicle)
{
    bool result = false;

    _distanceOut.open(_distanceOutFileName.c_str(), ios::app);
    if (_distanceOut.good())
    {
        _distanceOut << vehicle->id() << ","
                     << vehicle->tripLength() << ","
                     << (TimeManager::time() - vehicle->startTime())
                     << endl;

        _distanceOut.close();
        result = true;
    }
    return result;
}

//======================================================================
bool VehicleIO::writeAllVehiclesDistanceData()
{
    bool result = false;

    _distanceOut.open(_distanceOutFileName.c_str(), ios::app);
    if (_distanceOut.good())
    {
        vector<Vehicle*>* vehicles = ObjManager::vehicles();

        for (unsigned int i=0; i<vehicles->size(); i++)
        {
            _distanceOut << (*vehicles)[i]->id() << ","
                         << ( *vehicles)[i]->tripLength() << ","
                         << (TimeManager::time()
                             - (*vehicles)[i]->startTime()) << endl;
        }

        _distanceOut.close();
        result = true;
    }
    return result;
}
