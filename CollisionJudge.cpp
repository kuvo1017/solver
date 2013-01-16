#include <math.h>
#include "CollisionJudge.h"
#include "Vehicle.h"
#include "AmuVector.h"
#include "ErrorController.h"
#include "AmuPoint.h"

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
	cout << "自車["<<j<<"] ("<< vertice[0][j]->x()<<","<<vertice[0][j]->y()<<") "<<endl;
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
