#include "GVInitializer.h"
#include "GVManager.h"
#include "ErrorController.h"
#include <iostream>

using namespace std;

//======================================================================
void GVInitializer::init(const string& dataPath)
{
 GVManager::setNewString("DATA_DIRECTORY",
      dataPath);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // フラグの定義
  /*
   * 既に別の箇所で定義されている場合は再定義できない
   */

  // 詳細情報を出力するか
  GVManager::setNewFlag("FLAG_VERBOSE", true);

  // 地図情報を入力するか
  GVManager::setNewFlag("FLAG_INPUT_MAP", true);

  // 信号情報を入力するか
  GVManager::setNewFlag("FLAG_INPUT_SIGNAL", true);

  // 車両情報の龍力があるか
  GVManager::setNewFlag("FLAG_INPUT_VEHICLE", true);

  // 車両発生情報が定義されていない交差点から車両を発生させるか
  GVManager::setNewFlag("FLAG_GEN_RAND_VEHICLE", true);


#ifdef OACIS
  // 時系列データを出力するか
   GVManager::setNewFlag("FLAG_OUTPUT_TIMELINE", false);
#else
  // 時系列データを出力するか
   GVManager::setNewFlag("FLAG_OUTPUT_TIMELINE", true);
#endif
  // 計測機器の詳細データを出力するか
  GVManager::setNewFlag("FLAG_OUTPUT_MONITOR_D", true);

  // 計測機器の統計データを出力するか
  GVManager::setNewFlag("FLAG_OUTPUT_MONITOR_S", true);

  // エージェント発生データを出力するか
  GVManager::setNewFlag("FLAG_OUTPUT_GEN_COUNTER", true);

  // エージェントの走行距離，旅行時間を出力するか
  GVManager::setNewFlag("FLAG_OUTPUT_TRIP_INFO", true);

  // 自動で開始するか
  GVManager::setNewFlag("FLAG_AUTO_START", false);
   
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // グローバル変数設定ファイル
  GVManager::setNewString("GV_INIT_FILE",
      dataPath + "init.txt");

  // 地図に関連するファイル
  GVManager::setNewString("MAP_POSITION_FILE",
      dataPath + "mapPosition.txt");
  GVManager::setNewString("MAP_NETWORK_FILE",
      dataPath + "network.txt");
  GVManager::setNewString("SPEED_LIMIT_FILE",
      dataPath + "speedLimit.txt");

  // 車両発生に関するファイル
  GVManager::setNewString("GENERATE_TABLE", 
      dataPath + "generateTable.txt");
  GVManager::setNewString("DEFAULT_GENERATE_TABLE", 
      dataPath + "defaultGenerateTable.txt");

  // 車種に関するファイル
  GVManager::setNewString("VEHICLE_FAMILY_FILE",
      dataPath + "vehicleFamily.txt");

  // 自動車エージェントの経路に関するファイル
  GVManager::setNewString("VEHICLE_ROUTE_PARAM_FILE",
      dataPath + "vehicleRoutingParam.txt");

  // 路側器に関するファイル
  GVManager::setNewString("GENCOUNTER_FILE",
      dataPath + "genCounter.txt");
  GVManager::setNewString("DETECTOR_FILE",
      dataPath + "detector.txt");

  // 信号に関するディレクトリorファイルor拡張子
  GVManager::setNewString("SIGNAL_CONTROL_DIRECTORY",
      dataPath + "signals/");
  GVManager::setNewString("SIGNAL_CONTROL_FILE_DEFAULT",
      "default");
  GVManager::setNewString("SIGNAL_ASPECT_FILE_DEFAULT_PREFIX",
      "defaultInter");
  GVManager::setNewString("CONTROL_FILE_EXTENSION",
      ".msf");
  GVManager::setNewString("ASPECT_FILE_EXTENSION",
      ".msa");

  // 交差点属性指定ファイル
  GVManager::setNewString("INTERSECTION_ATTRIBUTE_DIRECTORY",
      dataPath+"intersection/");

  // 道路形状に関するファイル
  GVManager::setNewString("INTERSECTION_STRUCT_FILE",
      dataPath + "intersectionStruct.txt");
  GVManager::setNewString("SECTION_STRUCT_FILE",
      dataPath + "sectionStruct.txt");
#ifdef OACIS
  // エラー率を定義しているファイル
  GVManager::setNewString("ERROR_PARAMS_FILE",
      "./_input.json");
#endif
  // 出力先
  string resultPath = dataPath + "result/";
  GVManager::setNewString("RESULT_OUTPUT_DIRECTORY",
      resultPath);
  GVManager::setNewString("RESULT_TIMELINE_DIRECTORY",
      resultPath + "timeline/");
  GVManager::setNewString("RESULT_IMG_DIRECTORY",
      resultPath + "img/");
  GVManager::setNewString("RESULT_INSTRUMENT_DIRECTORY",
      resultPath + "inst/");
  GVManager::setNewString("RESULT_DETECTORD_PREFIX",
      "detD");
  GVManager::setNewString("RESULT_DETECTORS_PREFIX",
      "detS");
  GVManager::setNewString("RESULT_GENCOUNTER_PREFIX",
      "gen");
  GVManager::setNewString("RESULT_RUN_INFO_FILE",
      resultPath + "runInfo.txt");
  GVManager::setNewString("RESULT_NODE_SHAPE_FILE",
      resultPath + "nodeShape.txt");
  GVManager::setNewString("RESULT_LINK_SHAPE_FILE",
      resultPath + "linkShape.txt");

  GVManager::setNewString("RESULT_SIGNAL_COUNT_FILE",
      resultPath + "signalCount.txt");

  GVManager::setNewString("RESULT_VEHICLE_ATTRIBUTE_FILE",
      resultPath + "vehicleAttribute.txt");
  GVManager::setNewString("RESULT_VEHICLE_TRIP_FILE",
      resultPath + "vehicleTrip.txt");
  GVManager::setNewString("RESULT_VEHICLE_COUNT_FILE",
      resultPath + "vehicleCount.txt");
#ifndef OACIS
  GVManager::setNewString("RESULT_ERROR_FILE",
      resultPath + "_error.txt");
  GVManager::setNewString("RESULT_ACCIDENT_FILE",
      resultPath + "_accident.txt");
#else
   double rate = GVManager::getNumeric("NOLOOK_HEAD");
   GVManager::setNewString("RESULT_ERROR_FILE",
      "./_error_"+std::to_string(rate)+" .txt");
  GVManager::setNewString("RESULT_ACCIDENT_FILE",
       "./_accident"+std::to_string(rate)+".txt");
   GVManager::setNewString("RESULT_STAT_ACCIDENT_FILE",
       "./_stat_accident"+std::to_string(rate)+" .txt");
#endif

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 定数の定義
  /*
   * 後でGVManager::setVariablesFromFile()によって上書きされる
   */

  /* シミュレーションの実行に関するもの */

  // 出力ファイルに各種コメントを付して出力する
  GVManager::setNewNumeric("OUTPUT_COMMENT_IN_FILE", 0);

  // 計時の有無、0 以外ならあり
  GVManager::setNewNumeric("CHECK_TIME", 0);

  /* エージェントに関するもの */

  // 自動車の反応遅れ時間[sec]
  GVManager::setNewNumeric("REACTION_TIME_VEHICLE", 0.74);

  // 自動車が交差点で右折する場合などの
  // 対向車とのギャップアクセプタンス[sec]
  GVManager::setNewNumeric("GAP_ACCEPTANCE_VEHICLE_CROSS", 3.0);

  // 普通車(VehicleFamily::passenger())の最大加速度，減速度[m/(sec^2)]
  GVManager::setNewNumeric("MAX_ACCELERATION_PASSENGER", 3.0);
  GVManager::setNewNumeric("MAX_DECELERATION_PASSENGER", -5.0);

  // バス(VehicleFamily::bus())の最大加速度，減速度[m/(sec^2)]
  GVManager::setNewNumeric("MAX_ACCELERATION_BUS", 3.0);
  GVManager::setNewNumeric("MAX_DECELERATION_BUS", -5.0);

  // 大型車(VehicleFamily::truck())の最大加速度，減速度[m/(sec^2)]
  GVManager::setNewNumeric("MAX_ACCELERATION_TRUCK", 3.0);
  GVManager::setNewNumeric("MAX_DECELERATION_TRUCK", -5.0);

  // 車線変更時に与える横向きの速度[km/h]
  GVManager::setNewNumeric("ERROR_VELOCITY", 7.5);

  // 発生端点からL[m]以内の車両は出力しない
  GVManager::setNewNumeric("NO_OUTPUT_LENGTH_FROM_ORIGIN_NODE", 0);

  // 車両ログの追加情報を出力する
  GVManager::setNewNumeric("OUTPUT_VEHICLE_EXTENSION", 0);

  // 普通車(VehicleFamily::passenger())のサイズ[m]
  GVManager::setNewNumeric("VEHICLE_LENGTH_PASSENGER", 4.400);
  GVManager::setNewNumeric("VEHICLE_WIDTH_PASSENGER",  1.830);
  GVManager::setNewNumeric("VEHICLE_HEIGHT_PASSENGER", 1.315);

  // バス(VehicleFamily::bus())のサイズ[m]
  GVManager::setNewNumeric("VEHICLE_LENGTH_BUS", 8.465);
  GVManager::setNewNumeric("VEHICLE_WIDTH_BUS",  2.230);
  GVManager::setNewNumeric("VEHICLE_HEIGHT_BUS", 3.420);

  // 大型車(VehicleFamily::truck())のサイズ[m]
  GVManager::setNewNumeric("VEHICLE_LENGTH_TRUCK", 8.465);
  GVManager::setNewNumeric("VEHICLE_WIDTH_TRUCK",  2.230);
  GVManager::setNewNumeric("VEHICLE_HEIGHT_TRUCK", 3.420);

  // 交錯を厳密に評価
  GVManager::setNewNumeric("STRICT_COLLISION_CHECK", 1);

  // 速度履歴を保存するか
  GVManager::setNewFlag("VEHICLE_VELOCITY_HISTORY_RECORD", true);
  // 速度履歴を保存するステップ数
  GVManager::setNewNumeric("VEHICLE_VELOCITY_HISTORY_SIZE", 180);
  // 速度履歴を保存するステップ間隔
  GVManager::setNewNumeric("VEHICLE_VELOCITY_HISTORY_INTERVAL", 10);

  /* 道路に関するもの */

  // 右折専用レーンの標準長さ[m]
  GVManager::setNewNumeric("RIGHT_TURN_LANE_LENGTH", 30);

  // 標準制限速度[km/h]
  /* ただしSPEED_LIMIT_INTERSECTIONが用いられることはほとんど無く，
   * 右左折時は下のVELOCITY_AT〜が使われ，
   * 直進時は次の単路でのSPEED_LIMITが参照される．*/
  GVManager::setNewNumeric("SPEED_LIMIT_SECTION", 60);
  GVManager::setNewNumeric("SPEED_LIMIT_INTERSECTION", 60);

  // 徐行速度[km/h]
  GVManager::setNewNumeric("VELOCITY_CRAWL", 10);

  // 右左折時の速度[km/h]
  GVManager::setNewNumeric("VELOCITY_AT_TURNING_RIGHT", 40);
  GVManager::setNewNumeric("VELOCITY_AT_TURNING_LEFT",  40);

  // 車両発生時の制限速度[km/h]、負ならなし
  GVManager::setNewNumeric("GENERATE_VELOCITY_LIMIT", -1);

  // 右左折時の最小ヘッドウェイ[秒]
  GVManager::setNewNumeric("MIN_HEADWAY_AT_TURNING", 1.7);

  // 車両発生量既定値[台/h]、入口レーン 3 以上 / 2 / 1、基本交通容量の10%
  GVManager::setNewNumeric("DEFAULT_TRAFFIC_VOLUME_WIDE",   660);
  GVManager::setNewNumeric("DEFAULT_TRAFFIC_VOLUME_NORMAL", 440);
  GVManager::setNewNumeric("DEFAULT_TRAFFIC_VOLUME_NARROW", 125);

  // 標準のレーン幅、歩道幅、横断歩道幅、路側幅
  GVManager::setNewNumeric("DEFAULT_LANE_WIDTH",      3.5);
  GVManager::setNewNumeric("DEFAULT_SIDEWALK_WIDTH",  5.0);
  GVManager::setNewNumeric("DEFAULT_CROSSWALK_WIDTH", 5.0);
  GVManager::setNewNumeric("DEFAULT_ROADSIDE_WIDTH",  1.0);

  // 単路の歩道を自動設定する際のレーン数、-1 なら自動設定なし
  /* 自動設定ありの場合、単路の全レーン数がこれ以上なら歩道を設定する
   * これ以下なら車道通行可能にする */
  GVManager::setNewNumeric("AUTO_SIDEWALK_SECTION_LANE", -1);

  // 道路エンティティの厳密な内部判定、1 ならあり
  /* 凹型の道路エンティティでも判定可能になる、ただし処理速度は遅い */
  GVManager::setNewNumeric("ROAD_ENTITY_STRICT_JUDGE_INSIDE", 1);

  // 交差点サイズ制限、中心からの距離[m]
  GVManager::setNewNumeric("INTERSECTION_SIZE_LIMIT", 20);

  // 起きたエラーの回数
  GVManager::setNewNumeric("ERROR_COUNT",0);

  // 起きた事故の回数
  GVManager::setNewNumeric("ACCIDENT_COUNT",0);

#ifdef ERROR_MODE
ErrorController::initErrorParams();
#endif

#ifdef _OPENMP
  /* マルチスレッドに関するもの */

  // スレッド数，0ならomp_get_num_procs()が返すプロセッサ数
  /*
   * 0の場合はAppMates::initの中でresetする
   */
  GVManager::setNewNumeric("THREAD_NUM", 0);

  // 集中車両発生
  GVManager::setNewNumeric("GENERATE_INTENSIVE", 1);

  // 経路検索最大ステップ
  GVManager::setNewNumeric("ROUTER_SEARCH_STEP", 10000);

  // 乱数並列ストック、0 なら自動、並列実行時のみストック
  GVManager::setNewNumeric("RANDOM_MULTI_STOCK", 0);
#else //_OPENMP

  // シングルスレッドであればスレッド数は1
  GVManager::setNewNumeric("THREAD_NUM", 1);

#endif //_OPENMP
}

