#ifndef __COLLISION_JUDGE_H__
#define __COLLISION_JUDGE_H__

#include <string>
#include "Vehicle.h"
#include "AmuVector.h"
#include "AmuPoint.h"

//事故が起きた時の判定をするクラス
using namespace std;
 /// 事故シミュレーション用に衝突判定をするクラス
/**
 * 簡単な判定には計算不可の低いアルゴリズムを実装
 * しているため、各事故に対して各衝突判定が存在する
 */
 
class CollisionJudge{
 public:

  // 追突事故の衝突を判定
  static bool isFrontCollid(Vehicle* v1,Vehicle* v2);

  // 2つの車が側面から衝突しているかどうかを判定、進路変更事故用
  static void isSideCollid(Vehicle* v1);

  // 正面衝突しているかどうかを判定
  static bool isHeadCollid(Vehicle* v1,Vehicle* v2);

   // 交差点内における衝突判定、出会い頭、右左折事故用
  static bool isCollidInIntersection(Vehicle* v1,Vehicle* v2);

  // 
  static bool _checkCross(AmuVector& ea1, AmuVector& ea2, AmuVector& eb1,double dist);
 private:
  
};
#endif //__COLLISION_JUDGE_H__
