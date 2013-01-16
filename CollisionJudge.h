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
  //x軸、y軸に水平な事故しか判定できない
  static bool isCollid(Vehicle* v1,Vehicle* v2);

  //二つの車が衝突しているかどうかを判定
  static bool isCollidStrict(Vehicle* v1,Vehicle* v2);

  //対向車線にある車と正面衝突したかどうかの判定
  static bool isHeadCollid(Vehicle* v1,Vehicle* v2);
  
 private:
  
};
#endif //__COLLISION_JUDGE_H__
