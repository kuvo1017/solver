#include <math.h>
#include "CollisionJudge.h"
#include "Vehicle.h"
#include "AmuVector.h"
#include "ErrorController.h"

//======================================================================
bool CollisionJudge::isCollid(Vehicle* v1,Vehicle* v2){
   if(v1->intersection() == NULL 
      || v2->intersection() == NULL)
 
  /*
  if(v1->errorController()->type() =="not_error" 
      && v2->errorController()->type() =="not_error")
      */
  {
    return false;
  }
  /// Refference URL
  /// http://marupeke296.com/COL_3D_No13_OBBvsOBB.html
  AmuPoint* center[] = {new AmuPoint(v1->x(),v1->y(),0),new AmuPoint(v2->x(),v2->y(),0)};
  AmuVector* aToB = new AmuVector(*center[0],*center[1]);
  // toosugitara ridatsu
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
//    cout << "false!!!!!" <<endl;
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
bool CollisionJudge::isHeadCollid(Vehicle* v1,Vehicle* v2){
  double x1,y1,x2,y2;
  x1 = v1->x();  
  y1 = v1->y();
  x2 = v2->x();
  y2 = v2->y();
  if((fabs(y1-y2)<(v1->bodyLength()*0.5+v2->bodyWidth()*0.5))
      && (fabs(x1-x2)<(v2->bodyLength()*0.5+v1->bodyWidth()*0.5))){
    return true;
  }else{
    return false;
  }

}
