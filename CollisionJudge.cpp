#include <math.h>
#include "CollisionJudge.h"
#include "Vehicle.h"
#include "AmuVector.h"
#include "ErrorController.h"

//======================================================================
bool CollisionJudge::isCollid(Vehicle* v1,Vehicle* v2){
  if(v1->errorController()->type() =="not_error" 
      && v2->errorController()->type() =="not_error")
  {
    return false;
  }

  double x1,y1,x2,y2;
  x1 = v1->x();  
  y1 = v1->y();
  x2 = v2->x();
  y2 = v2->y();
  if(fabs(x1-x2)<(v1->bodyLength()*0.5+v2->bodyLength()*0.5)
      && (fabs(y1-y2)<(v2->bodyWidth()*0.5+v1->bodyWidth()*0.5))){
    return true;
  }else{
    return false;
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
