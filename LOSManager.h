#ifndef __LOS_MANAGER_H__
#define __LOS_MANAGER_H__

#include <string>
#include <vector>
#include <map>
#include <LOSConf.h>
#include <Geometry.h>
#include <shapefil.h>

using namespace std;

class QuadTree;
class Vehicle;

class LOSManager{
 public:
  LOSManager();
  ~LOSManager();

  // ファイル読み込みと建造物ポリゴンデータを作成する
  void init();

  // 全車両間の可視判定を行う
  /* 結果は配列に格納する */
  void calcVehicleLOS(vector<Vehicle*>* vehicles);

  //　vehicleが含まれるレーンが障害物の近くにあるかを
  //  判断していき、そのvehicleがあるlaneでは一時停止するようにする。
  void checkLanes(vector<Vehicle*>* vehicles);

  // 地図領域を返す
  void field(double *xmin, double *xmax,
	     double *ymin, double *ymax) const;

  // 二車両間にある障害物数を返す
  int lookupLOS(int srcId, int dstId);

 private:
  // ファイル読み込み関数
  void _readFiles(string shpFileName, string dbfFileName);
  // SHPファイルハンドラ
  SHPHandle _hSHP;
  // DBFファイルハンドラ
  DBFHandle _hDBF;

  // 建造物のポリゴンをマニュアルで作成
  void _createBuildings();

  // ポリゴンの作成
  void _makePolygon(int entityId, int partId, SHPObject* shape);

  // 線分の傾きとy切片を算出
  void _calcStraightLine(LOSVehicle* vt, LOSVehicle* vr, double* a, double* b);

  // 境界
  double _adfMaxBounds[4];
  double _adfMinBounds[4];

  // ポリゴン総数
  int _numPolygons;
  // 壁総数
  int _numWalls;
  // 車体総数
  int _numVehicles;
 
  // MATESのID（int に直したもの）とLOS用のIDの関連付け
  map<int,int> _idMap;
  
  // MATESのID（int に直したもの）とLOS用のIDの関連付けたももの
  //キーと値を逆にしたもの
  map<int,int> _idMapR;

  // 結果を格納する配列
  int _los[MAX_VEHICLES*MAX_VEHICLES];
};
#endif //__LOS_MANAGER_H__
