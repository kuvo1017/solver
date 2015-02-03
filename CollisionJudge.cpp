#include <math.h>
#include "CollisionJudge.h"
#include "Vehicle.h"
#include "AmuVector.h"
#include "ErrorController.h"
#include "AmuPoint.h"
#include "Lane.h"
#include "RoadOccupant.h"


//======================================================================
bool CollisionJudge::isCollidInIntersection(Vehicle* v1,Vehicle* v2){
 if(v1->intersection() == NULL 
      || v2->intersection() == NULL
      || (v1->errorController()->type() == "not_error"
      && v2->errorController()->type() == "not_error")
    )
  {
    return false;
  }
  /// Refference URL
  /// http://marupeke296.com/COL_3D_No13_OBBvsOBB.html
  AmuPoint* center[] = {new AmuPoint(v1->x(),v1->y(),0),new AmuPoint(v2->x(),v2->y(),0)};
  AmuVector* aToB = new AmuVector(*center[0],*center[1]);
  // 車両同士の距離が離れすぎていたら、ループ離脱
  // お互いの距離 > 車体の対角線の合計値のとき
  if(aToB->size()*2 > v1->bodyDiagnoalXY() + v2->bodyDiagnoalXY())
  {
    return false;
  }
  AmuVector direction[] = {v1->directionVector(),v2->directionVector()};
  AmuVector normalDirection[2];
  for(int i=0;i<2;i++)
  {
    direction[i].normalize();
    normalDirection[i] = direction[i];
    normalDirection[i].revoltXY(M_PI/2);
  }
  AmuVector toEdgeOne[2][2] ={{direction[0],normalDirection[0]},{direction[1],normalDirection[1]}};
  AmuVector toEdge[2][2] ={{v1->bodyLength()*0.5*direction[0],v1->bodyWidth()*0.5*normalDirection[0]},{ v2->bodyLength()*0.5*direction[1],v2->bodyWidth()*0.5*normalDirection[1]}};
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
    string type1 = v1 ->errorController()->type();
    string type2 = v2 ->errorController()->type();
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
    /*
       cout << "ea1 (" << ea1.x() << ", "  << ea1.y() << ")"<<endl; 
       cout << "ea2 (" << ea2.x() << ", "   << ea2.y() << ")"<<endl; 
       cout << "eb1 (" << eb1.x() << ", "   << eb1.y() << ")"<<endl; 
       cout << "dist is " << dist << endl;
       cout << "ra is " << ra << endl; 
       cout << "rb is " << rb << endl;
       cout << "normal rb is " << eb1.size() << endl;
       cout << "true!!!!!" <<endl;
     */
    return true;
  }
}

//======================================================================
/*
bool CollisionJudge::isCollidStrict(Vehicle* v1,Vehicle* v2)

{
  Vehicle* vehicle[] = {v1,v2};
  AmuPoint* vertice[2][4];
  AmuVector* edge[2][4];
  AmuVector* calcVector[2][4];
  for(int i=0;i<2;i++)
  {
    double x = vehicle[i]->x();
    double y = vehicle[i]->y();
    AmuVector direction = vehicle[i]->directionVector();
    direction.normalize();
    AmuVector normal = direction;
    normal.revoltXY(M_PI/4.0);
    direction = direction*vehicle[i]->bodyLength()/2.0;
    normal = normal * vehicle[i]->bodyWidth()/2.0;
    vertice[0][0] = new AmuPoint(x + direction.x() + normal.x(),
	y+direction.y()+normal.y(),0);
    vertice[0][1] = new AmuPoint(x + direction.x() - normal.x(),
	y+direction.y()-normal.y(),0);
    vertice[0][2] = new AmuPoint(x - direction.x() - normal.x(),
	y-direction.y()-normal.y(),0);
    vertice[0][3] = new AmuPoint(x - direction.x() + normal.x(),
	y-direction.y()+normal.y(),0);
  }
  for(int i=0;i<2;i++)
  {
    for(int j=0;j<4;j++)
    {
      if(j!=3)
      {
	edge[i][j] = new AmuVector(*vertice[i][j],*vertice[i][j+1]);
      }else
      {
	edge[i][j] = new AmuVector(*vertice[i][j],*vertice[i][0]);
      }
      if(i ==0)
      {
	calcVector[0][j] = new AmuVector(*vertice[0][j],*vertice[1][j]);
       if(edge[0][j]->calcCrossProduct(*calcVector[1][j]) > 0)
	{
	  return false;
	}
      }else
      {
	calcVector[1][j] = new AmuVector(*vertice[1][j],*vertice[0][j]);
	if(edge[1][j]->calcCrossProduct(*calcVector[0][j]) > 0)
	{
	  return false;
	}
      }
      return true;
    }
  }
}
*/
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
  string type1 = v1 ->errorController()->type();
    string type2 = v2 ->errorController()->type();
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
	  string type1 = v1 ->errorController()->type();
	  string type2 = v2 ->errorController()->type();
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
	string type1 = v1 ->errorController()->type();
	string type2 = v2 ->errorController()->type();
	v1->errorController()->accidentOccur("head_"+type1);
	v2->errorController()->accidentOccur("head_"+type2);
	return true;
      }
    }
  }
  return false;
}
