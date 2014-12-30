#ifndef __COLLISION_JUDGE_H__
#define __COLLISION_JUDGE_H__

#include <string>
#include "Vehicle.h"
#include "AmuVector.h"
#include "AmuPoint.h"

//事故が起きた時の判定をするクラス
using namespace std;

class CollisionJudge{
 public:
  //二つの車が衝突しているかどうかを判定
  static bool isCollid(Vehicle* v1,Vehicle* v2);
  //対向車線にある車と正面衝突したかどうかの判定
  static bool isFrontCollid(Vehicle* v1,Vehicle* v2);
  //二つの車が衝突しているかどうかを判定
  static void isSideCollid(Vehicle* v1);
  //二つの車が衝突しているかどうかを判定
  static bool isCollidInIntersection(Vehicle* v1,Vehicle* v2);
  //二つの車が衝突しているかどうかを判定
  static bool isHeadCollid(Vehicle* v1,Vehicle* v2);
  
  static bool _checkCross(AmuVector& ea1, AmuVector& ea2, AmuVector& eb1,double dist);
 private:
  
};
#endif //__COLLISION_JUDGE_H__
