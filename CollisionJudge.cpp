#include <math.h>
#include "CollisionJudge.h"
#include "Vehicle.h"
#include "AmuVector.h"
#include "ErrorController.h"
#include "AmuPoint.h"
#include "Lane.h"
#include "RoadOccupant.h"



//======================================================================
bool CollisionJudge::isFrontCollid(Vehicle* v1,Vehicle* v2){
  if(!(v1->errorController()->isRearError()) 
      &&  !(v2->errorController()->isRearError()) ) 
  {
    return false;
  }
  double x,y;
  double gap = 0.5;
  x = v1->x() - v2->x();  
  y = v1->y() - v2->y();
  double distance = sqrt(x*x + y*y);
  if(distance + 0.5< (v1->bodyLength() + v2->bodyLength())*0.5)
  {                 
    string type1 = v1 ->errorController()->errorType();
    string type2 = v2 ->errorController()->errorType();
    v1->errorController()->accidentOccur("front_"+type1);
    v2->errorController()->accidentOccur("front_"+type2);
    return true;
  }else{
    return false;
  }
}
//======================================================================
void CollisionJudge::isSideCollid(Vehicle* v1){
  if(v1->errorController()->isShiftError())
  {
    double x1 = v1->x();
    double y1 = v1->y();
    double bodyLength = v1->bodyLength();
    double bodyWidth = v1->bodyWidth();
    Lane* laneTo = v1->laneShifter().laneTo();
    AmuVector direction = v1->directionVector();
    direction.normalize();
    AmuVector sideDirection = direction;
    sideDirection.revoltXY(M_PI/2);
    std::vector<RoadOccupant*>* agents = laneTo->agents();
    for(int i=0;i<agents->size();i++)
    {
      Vehicle* v2 = dynamic_cast<Vehicle*>(agents->at(i));
      double x = x1 - v2->x();  
      double y = y1 - v2->y();
      AmuVector* aToB = new AmuVector(x,y,0);
      double rearDistance = fabs(direction.calcScalar(*aToB));
      if(rearDistance < (bodyLength + v2 ->bodyLength())*0.5)
      {
	double sideDistance = fabs(sideDirection.calcScalar(*aToB));
	if(sideDistance < (bodyWidth + v2->bodyWidth())*0.5)
	{
	  if(v1->laneShifter().isActive())
	  {
	    v1->laneShifter().endShift();
	  }                
	  string type1 = v1 ->errorController()->errorType();
	  string type2 = v2 ->errorController()->errorType();
	  v1->errorController()->accidentOccur("side_"+type1);
	  v2->errorController()->accidentOccur("side_"+type2);
	}
      }
    }
  }
} 
//======================================================================
bool CollisionJudge::isHeadCollid(Vehicle* v1,Vehicle* v2){
  if(v1->errorController()->isHeadError()
      || v2->errorController()->isHeadError())
  {
    AmuVector direction = v1->directionVector();
    direction.normalize();
    AmuVector sideDirection = direction;
    sideDirection.revoltXY(M_PI/2);
    double x = v1->x() - v2->x();  
    double y = v1->y() - v2->y();
    AmuVector* aToB = new AmuVector(x,y,0);
    double rearDistance = fabs(direction.calcScalar(*aToB));
    if(rearDistance < (v1->bodyLength() + v2 ->bodyLength())*0.5)
    {
      double sideDistance = fabs(sideDirection.calcScalar(*aToB));
      if(sideDistance < (v1->bodyWidth() + v2->bodyWidth())*0.5)
      {        
	string type1 = v1 ->errorController()->errorType();
	string type2 = v2 ->errorController()->errorType();
	v1->errorController()->accidentOccur("head_"+type1);
	v2->errorController()->accidentOccur("head_"+type2);
	return true;
      }
    }
  }
  return false;
} 

//======================================================================
bool CollisionJudge::isCollidInIntersection(Vehicle* v1,Vehicle* v2){
  if(v1->intersection() == NULL 
      || v2->intersection() == NULL
      || (v1->errorController()->errorType() == "NOT_ERROR"
	&& v2->errorController()->errorType() == "NOT_ERROR")
    )
  {
    return false;
  }
  /// 参考URL
  /// http://marupeke296.com/COL_3D_No13_OBBvsOBB.html
  AmuPoint* center[] = {new AmuPoint(v1->x(),v1->y(),0),new AmuPoint(v2->x(),v2->y(),0)};
  AmuVector* aToB = new AmuVector(*center[0],*center[1]);

  // 車両同士の距離が離れすぎていたら、ループ離脱
  // お互いの距離 > 車体の対角線の合計値のとき
  if(aToB->size()*2 > v1->bodyDiagnoalXY() + v2->bodyDiagnoalXY())
  {
    return false;
  }
  // 車両の進行方向のベクトル
  AmuVector direction[] = {v1->directionVector(),v2->directionVector()};
  // 車両の進行方向の法線ベクトル
  AmuVector normalDirection[2];
  for(int i=0;i<2;i++)
  {
    direction[i].normalize();
    normalDirection[i] = direction[i];
    normalDirection[i].revoltXY(M_PI/2);
  }
  // 2つの車両の進行方向、法線ベクトルを格納(正規化)
  AmuVector toEdgeOne[2][2] ={{direction[0],normalDirection[0]},{direction[1],normalDirection[1]}};
  // 2つの車両の進行方向、法線ベクトルを格納、大きさは車両のサイズを反映
  AmuVector toEdge[2][2] ={{v1->bodyLength()*0.5*direction[0],v1->bodyWidth()*0.5*normalDirection[0]},{ v2->bodyLength()*0.5*direction[1],v2->bodyWidth()*0.5*normalDirection[1]}};

  /**
   * 2つの車両の中心距離を4つの辺
   * (2つの車両の進行方向とその法線)
   * に対して射影したベクトルの大きさ
   */

  double dist[4];

  dist[0] = fabs(aToB->calcScalar(toEdgeOne[1][0]));
  dist[1] = fabs(aToB->calcScalar(toEdgeOne[1][1]));
  dist[2] = fabs(aToB->calcScalar(toEdgeOne[0][0]));
  dist[3] = fabs(aToB->calcScalar(toEdgeOne[0][1]));

  if(
      _checkCross(toEdge[0][0],toEdge[0][1],toEdge[1][0],dist[0])
      &&_checkCross(toEdge[0][0],toEdge[0][1],toEdge[1][1],dist[1])
      &&_checkCross(toEdge[1][0],toEdge[1][1],toEdge[0][0],dist[2])
      &&_checkCross(toEdge[1][0],toEdge[1][1],toEdge[0][1],dist[3])
    )
  {
    string type1 = v1 ->errorController()->errorType();
    string type2 = v2 ->errorController()->errorType();
    v1->errorController()->accidentOccur("intersection_"+type1);
    v2->errorController()->accidentOccur("intersection_"+type2);
    return true;
  }else
  {
    return false;
  }
}

//======================================================================
bool CollisionJudge::_checkCross(AmuVector& ea1,AmuVector& ea2,AmuVector& eb1,double dist){

  double rb = eb1.size();
  eb1.normalize();
  double ra = fabs(ea1.calcScalar(eb1)) + fabs(ea2.calcScalar(eb1));
  if(dist > ra + rb)
  {
    return false;
  }else
  {
    return true;
  }
}
 
