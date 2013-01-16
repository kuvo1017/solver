#include "LOSManager.h"
#include "Vehicle.h"
#include "AmuVector.h"
#include "AmuPoint.h"
#include "Barrier.h"

//======================================================================
bool LOSManager::_isVisible(Vehicle* v1,Vehicle* v2,Barrier* barrier)
{
  Vehicle vehicles[] = {*v1,*v2}
  AmuVector vectors[] = {v1->directionVector(),v2->directionVector()};
  AmuPoint points[2];
  for(int i=0;i<2;i++)
  {
    AmuVector* vector = vectors[i];
    Vehicle* vehicle = vehicles[i];
    vectors[i].normalize();
    points[i] = new AmuPoint(vehicle.x()+vector.x()*vehicle.bodylength()/2,
	vehicle.y()+vector.y()*vehicle.bodylength()/2,0); 
  for(int i=0;i<2;i++)
  {
    AmuPoint* beginPoint = diagnoals[i]->
      AmuVector* vectors[3] = { new AmuVector(barrier->vertices(i),vertices(i+2)),
	new AmuVector(barrier->vertices(i),points[0]), 
	new AmuVector(barrier->vertices(i),points[1])};
    vectors[0]->calcCrossProduct(vectors[1])*vectors[0]->calcCrossProduct(vectors[2])>0 ? (return true):(return false);
 
  }
}


