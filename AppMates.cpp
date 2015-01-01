#include "AppMates.h"
#include "AmuConverter.h"
#include "GVInitializer.h"
#include "GVManager.h"
#include "Random.h"
#include "Simulator.h"
#include "ErrorController.h"
#include "OacisIO.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <unistd.h>
#ifndef USE_MINGW
#include <getopt.h>
#endif
#ifdef _OPENMP
#include <omp.h>
#endif

#define MATES_NDEBUG

using namespace std;

//======================================================================
AppMates::AppMates() :  _simulator(), _dataPath(), _key()
{
#ifdef OACIS
  _dataPath = OacisIO::inputParams();
#else
 //_dataPathと_keyのデフォルト値を設定
  _dataPath = "./";
#endif 

#ifdef MATES_NDEBUG
  _key = time(NULL);
#else
  _key = 2;
#endif
}

//======================================================================
AppMates::~AppMates()
{
  if (_simulator)
  {
    delete _simulator;
  }
}

//======================================================================
void AppMates::init(int argc, char** argv, bool output)
{
  // オプションの処理
  AppMates::parseArgument(argc, argv);

  cout << "random seed   : " << _key << endl;
  cout << "data directory: " << _dataPath << endl;

  // 乱数の準備
  srand(_key);
  Random::setSeed(_key);

  // パスの設定
  initPath();

  // グローバル変数の初期化
  GVManager::init();
  GVInitializer::init(_dataPath);

  // グローバル変数を読み込む(_initPathよりも後ろでなければならない)
  string fileName;
  if (GVManager::getVariable("GV_INIT_FILE", &fileName))
  {
    GVManager::setVariablesFromFile(fileName);
  }
  if (GVManager::getFlag("FLAG_VERBOSE"))
  {
    GVManager::print();
  }

#ifdef _OPENMP
  // スレッド数設定
  int thread
    = static_cast<int>(GVManager::getNumeric("THREAD_NUM"));
  if (thread <= 0)
  {
    thread = omp_get_num_procs();
    GVManager::resetNumeric("THREAD_NUM", thread);
  }
  cout << "Thread number: " << thread;
  omp_set_num_threads(thread);

  // 並列ストック最大数設定
  int randomStock
    = static_cast<int>(GVManager::getNumeric("RANDOM_MULTI_STOCK"));
  if (randomStock == 0 && thread > 1)
  {
    Random::multiStockSetMax(1);
    cout << ", Random stock: 1 (auto)" << endl;
  }
  else if (randomStock > 0)
  {
    Random::multiStockSetMax(randomStock);
    cout << ", Random stock: " << randomStock << endl;
  }
  else
    cout << ", Random stock: none" << endl;
#endif //_OPENMP

  // シミュレータの準備
  // RoadMapの作成
  bool isReady = getReadySimulator();
  if (!isReady)
  {
    cerr << "Error: failed to get ready for running simulator." << endl;
  }
}

//======================================================================
Simulator* AppMates::simulator()
{
  return _simulator;
}

//======================================================================
int AppMates::optionIndex;
string AppMates::shortOptions = "HhD:d:R:r:T:t:SsLlMmGgQq";
struct option AppMates::longOptions[] =
{
  {"help", 0, 0, 'h'},
  {"time", 1, 0, 't'},
  {"no-input",         0, 0, 30},
  {"no-input-map",     0, 0, 31},
  {"no-input-signal",  0, 0, 32},
  {"no-input-vehicle", 0, 0, 33},
  {"no-output-timeline",   0, 0, 's'},
  {"no-output-tripinfo",   0, 0, 'l'},
  {"no-output-generate",   0, 0, 'g'},
  {"no-output-instrument", 0, 0, 'm'},
  {"no-verbose", 0, 0, 'q'},
  {"no-output-monitor-d", 0, 0, 20},
  {"no-output-monitor-s", 0, 0, 21},
  {"no-generate-random-vehicle", 0, 0, 40},
  {"auto-start",0,0,50},
  {"lr",0,0,60},
  {0, 0, 0, 0}
};

//======================================================================
void AppMates::parseArgument(int argc, char** argv)
{
 std::string str;
  int opt;
#ifdef USE_MINGW
  while ((opt = getopt(argc, argv,
	  AppMates::shortOptions.c_str())) != -1)
#else //USE_MINGW
    while ((opt = getopt_long(argc, argv,
	    AppMates::shortOptions.c_str(),
	    AppMates::longOptions,
	    &AppMates::optionIndex)) != -1)
#endif //USE_MINGW
    {
      switch (opt)
      {
      case 'H':
      case 'h': // ヘルプを出力する
	printUsage();
	break;
      case 'D':
      case 'd': // データディレクトリを指定する
	_initDataPath(optarg);
        //double rate = (double) *optarg;
	/*
	str = optarg;
	cout << "optarg is " << std::stof(str) <<endl;
	GVManager::setNewNumeric("ARROGANCE_LR",(double) std::stof(str));  
	*/
	break;
      case 'R':
      case 'r': // 乱数の種を指定する
	_initRandomSeed(optarg);
	break;
      case 'Q': // 情報表示をonに（デフォルトでon）
	GVManager::resetFlag("FLAG_VERBOSE", true);
	break;
      case 'q': // 情報表示をoffに
	GVManager::resetFlag("FLAG_VERBOSE", false);
	break;
      case 'a': // 情報表示をoffに
 	cout << "optarg is " << optarg <<endl;
	GVManager::setNewNumeric("ARROGANCE_LR",(double) *optarg); 
	break;
 
#ifndef USE_MINGW
      case 30:  // 入力をoffに
	GVManager::resetFlag("FLAG_INPUT_MAP", false);
	GVManager::resetFlag("FLAG_INPUT_SIGNAL", false);
	GVManager::resetFlag("FLAG_INPUT_VEHICLE", false);
	break;
      case 31:  // 地図データ入力をoffに
	GVManager::resetFlag("FLAG_INPUT_MAP", false);
	break;
      case 32:  // 信号データ入力をoffに
	GVManager::resetFlag("FLAG_INPUT_SIGNAL", false);
	break;
      case 33:
	GVManager::resetFlag("FLAG_INPUT_VEHICLE", false);
	break;
      case 40:
	GVManager::setNewFlag("FLAG_GEN_RAND_VEHICLE", false);
	break;
      case 50:
	GVManager::setNewFlag("FLAG_AUTO_START", true);
       break; 
#endif //USE_MINGW
      default:
	break;
      }
    }

  // 派生クラスのparseArgumentを呼ぶ
  optind = 1;
  parseArgument(argc, argv);
}

//======================================================================
void AppMates::printUsage()
{
  cout <<
    "Options:\n"
    " -d <DataDir>      : set root path of input and output directory.\n"
    "                     (default: current directory)\n"
    " -r <Number>       : set random seed.\n"
    "                     (default: variable number on account of current time)\n"
#ifndef USE_MINGW
    " --no-verbose      : hide detail information.\n"
    " --no-input        : do not read any input files.\n"
    " --no-input-map    : do not read network.txt and mapPosition.txt.\n"
    " --no-input-signal : do not read signal input files.\n"
    "                     All signals show blue sign.\n"
    " --no-generate-random-vehicle\n"
    "                   : do not generate vehicles without input data."
#endif //USE_MINGW
    << endl;
}

//======================================================================
void AppMates::initPath()
{
  // データ関連の設定をする
  // RoadMap関連クラスからも使用する
  GVInitializer::init(_dataPath);
}

//======================================================================
bool AppMates::getReadySimulator()
{
  bool result = true;

  //_simulatorの新規作成
  if (_simulator != NULL)
  {
    _simulator = NULL;
  }
  _simulator = new Simulator();

  // 地図を準備する
  if (GVManager::getFlag("FLAG_INPUT_MAP"))
  {
    result = _simulator->getReadyRoadMap();

    // 路側器に関するファイルを読み込む
    // 自動車以外を計測する場合もあるかもしれないので，
    // 下のifndefの外にした
    result = _simulator->getReadyRoadsideUnit();
    if (!result)
    {
      cerr << "Cannot Load RoadsideUnit Files." << endl;
      return result;
    }
#ifndef EXCLUDE_VEHICLES
    // 車両に関するファイルを読み込む
    result = _simulator->getReadyVehicles();
    if (!result)
    {
      cerr << "Cannot Load Vehicle Files." << endl;
      return result;
    }
#endif //EXCLUDE_VEHICLES
  }
  else
  {
    // 地図を読み込まない場合->サンプルシナリオ
    result = _simulator->getReadySampleScenario(-500, 500,
	-500, 500,
	200, 200,
	100, 10);
  }
  if (!result)
  {
    cerr << "Cannot Create RoadMap." << endl;
    return result;
  }

  // レーンチェック
  _simulator->checkLane();
  return result;
}

//======================================================================
bool AppMates::_initDataPath(string arg)
{

  if (!arg.empty())
  {
    if (arg[arg.length()-1] != '/')
    {
      arg += '/';
    }
    _dataPath = arg;

  }
  return true;
}

//======================================================================
bool AppMates::_initRandomSeed(string arg)
{
  if (!arg.empty())
  {
    _key = static_cast<unsigned int>(AmuConverter::strtoul(arg));
    return true;
  }
  else
  {
    return false;
  }
}
