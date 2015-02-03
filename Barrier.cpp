#include <iostream>
#include "Barrier.h"
#include "Intersection.h"
#include "Section.h"
#include "AmuPoint.h"
#include "Lane.h"
#include "RoadOccupant.h"
#include "Vehicle.h"
#include "ErrorController.h"

using namespace std;
// ====================================
Barrier::Barrier(Intersection* i0,
    Section* s1,
    Section* s2,
    std::string id)
{
  _id = id;
  /*
     cout << "enter into Barrier.cpp!!!!!"<<endl;
     cout << "Intersection id: " <<i0->id()<<endl;
     cout << "Section1 id: " <<s1->id()<<endl; 
     cout << "Section2 id: " <<s2->id()<<endl; 
   */
  // IntersectionとSectionの4つの交点
  AmuPoint* rawVertices[4];
  // Barrierの4つの交点
  _intersection=i0;
  _section[0]=s1;
  _section[1]=s2;
  //cout << "AmuPoint呼び出し" << endl;
  std::vector<AmuPoint> iAmuPoints = _intersection->vertices();
  std::vector<AmuPoint> sAmuPoints1 = _section[0]->vertices(); 
  std::vector<AmuPoint> sAmuPoints2 = _section[1]->vertices(); 
  bool once1=false;
  bool once2=true; 
  // cout << "共通するAmuPoint探し" << endl; 
  for(int i=0;i<iAmuPoints.size();i++)
  {
    AmuPoint iAmuPoint = iAmuPoints[i];
    for(int j=0;j<sAmuPoints1.size();j++){
      AmuPoint sAmuPoint1 = sAmuPoints1[j];
      if(iAmuPoint.x()==sAmuPoint1.x()
	  && ( iAmuPoint.y()==sAmuPoint1.y()))
      {
	if(once1)
	{
	  rawVertices[0] = new AmuPoint(iAmuPoint.x(),iAmuPoint.y(),0);
	  once1 = false;
	}
	else
	{
	  once1 = true;
	}
      }
    }
    for(int j=0;j<sAmuPoints2.size();j++){
      AmuPoint sAmuPoint2 = sAmuPoints2[j];
      if(iAmuPoint.x()==sAmuPoint2.x()
	  && ( iAmuPoint.y()==sAmuPoint2.y()))
      {
	if(once2)
	{
	  rawVertices[1] = new AmuPoint(iAmuPoint.x(),iAmuPoint.y(),0);
	  once2 = false;
	}
      }
    } 
  }
  // cout << "共通しないAmuPoint探し" << endl;  

  for(int i=0;i<sAmuPoints1.size();i++)
  {
    AmuPoint* sap = &sAmuPoints1[i];
    if(sap->x()==rawVertices[0]->x()
	&& ( sap->y()==rawVertices[0]->y())) 
    {
      if(i < sAmuPoints1.size()-1)
	rawVertices[3]=&sAmuPoints1[i+1];
      else
	rawVertices[3]=&sAmuPoints1[0];
    }
  }
  for(int i=0;i<sAmuPoints2.size();i++)
  {
    AmuPoint* sap = &sAmuPoints2[i];
    if(sap->x()==rawVertices[1]->x()
	&& ( sap->y()==rawVertices[1]->y())) 
    {
      if(i>0)
	rawVertices[2]=&sAmuPoints2[i-1];
      else
	rawVertices[2]=&sAmuPoints2[sAmuPoints2.size()-1];
    }
  } 

  for(int i=0;i<4;i++){
//    std::cout <<"("<< rawVertices[i]->x() <<","<<rawVertices[i]->y()<<")" <<std::endl;
  }

  // 四角形の中心点を計算する
  AmuPoint* center = new AmuPoint;
  // 対角線の傾き
  double grad1 = (rawVertices[0]->y()-rawVertices[2]->y())/(rawVertices[0]->x()-rawVertices[2]->x());
  double grad2 = (rawVertices[1]->y()-rawVertices[3]->y())/(rawVertices[1]->x()-rawVertices[3]->x());
  // 対角線の切片
  double intercept1 = rawVertices[0]->y()-grad1*rawVertices[0]->x();
  double intercept2 = rawVertices[1]->y()-grad2*rawVertices[1]->x();
  center->setX(-(intercept1-intercept2)/(grad1-grad2));
  center->setY(-grad1*(intercept1-intercept2)/(grad1-grad2)+intercept1); 
  for(int i=0;i<4;i++)
  {
    //std::cout << "ループ処理開始" << endl; 
    AmuPoint* v = new AmuPoint;
    // 四角形の中心点から各点に向けて、縮尺をつける
    // 道路の端からどれくらい離すか
    double separate = 1.0; //[m]
    double length = sqrt(pow(rawVertices[i]->x()-center->x(),2)+pow(rawVertices[i]->y()-center->y(),2));
    double t = 1-separate/length;
    //std::cout <<i<< "kokoha?" <<center->x()<< endl;  
    v->setX((1-t)*center->x() + t*(rawVertices[i]->x()));
    v->setY((1-t)*center->y() + t*(rawVertices[i]->y()));  
    v->setZ(0.0);
    _vertices.push_back(v);
  }
  //std::cout << "ループ処理終了" << endl;  

  _diagnoal[0] = new AmuVector(*_vertices[0] ,*_vertices[2]);
  _diagnoal[1] = new AmuVector(*_vertices[1],*_vertices[3]);
  // 見通しが悪い情報を単路に登録
  s1->setBadView();
  s2->setBadView();
} 
//======================================================================
std::string Barrier::id()
{
  return _id;
}
//======================================================================
void Barrier::checkVehiclesVisible()
{
  const RMAPLAN* lanes1 = _section[0]->lanes();
  const RMAPLAN* lanes2 = _section[1]->lanes(); 
  CITRMAPLAN itl1;
  for (itl1=lanes1->begin(); itl1!=lanes1->end(); itl1++)
  {            
    LaneBundle* nextBundle = _section[0]->nextBundle(itl1->second);
    if(nextBundle == NULL)
    {
      continue;
    }
    if(nextBundle->id() == _intersection->id()) 
    {
      CITRMAPLAN itl2;
      for (itl2=lanes2->begin(); itl2!=lanes2->end(); itl2++)
      {
	nextBundle = _section[1]->nextBundle(itl2->second);
	if(nextBundle == NULL)
	{
	  continue;
	}
	if(nextBundle->id() == _intersection->id()) 
	{
	  std::vector<RoadOccupant*>* agents1 = itl1->second->agents();
	  std::vector<RoadOccupant*>* agents2 = itl2->second->agents();
	  for(int i=0;i<agents1->size();i++)
	  {
	    Vehicle* vehicle1 =  reinterpret_cast<Vehicle*>(agents1->at(i)); 
	    for(int j=0;j<agents2->size();j++)
	    {
	      Vehicle* vehicle2 =  reinterpret_cast<Vehicle*>(agents2->at(j)); 
	      if(_isVisible(vehicle1,vehicle2))
	      {
		vehicle1->errorController()->setInvisibleVehicle(vehicle2);
		vehicle2->errorController()->setInvisibleVehicle(vehicle1); 
	      }
	    }
	  }
	}
      }
    }
  }
}

//======================================================================
bool Barrier::_isVisible(Vehicle* v1,Vehicle* v2)
{
  Vehicle* vehicles[] = {v1,v2};
  AmuVector vehicleVectors[] = {v1->directionVector(),v2->directionVector()};
  AmuPoint* points[2];
  for(int i=0;i<2;i++)
  {
    AmuVector* vehicleVector = &vehicleVectors[i];
    Vehicle* vehicle = vehicles[i];
    vehicleVector->normalize();
    points[i] = new AmuPoint(vehicle->x()+vehicleVector->x()*vehicle->bodyLength()/2,
	vehicle->y()+vehicleVector->y()*vehicle->bodyLength()/2,0); 
  }
  for(int i=0;i<2;i++)
  {
    // 外積計算のための三つのベクトル
    AmuVector* calcVectors[] = {new AmuVector(*_vertices[i],*_vertices[i+2]),
      new AmuVector(*_vertices[i],*points[0]), 
      new AmuVector(*_vertices[i],*points[1]),
      new AmuVector(*points[0],*points[1]),
      new AmuVector(*points[0],*_vertices[i]), 
      new AmuVector(*points[0],*_vertices[i+2])};
    //２つの外積の積が正であれば交差せず
    //負であれば交差する
    if(calcVectors[0]->calcCrossProduct(*calcVectors[1])*calcVectors[0]->calcCrossProduct(*calcVectors[2]) < 0 && (calcVectors[3]->calcCrossProduct(*calcVectors[4])*calcVectors[3]->calcCrossProduct(*calcVectors[5]) < 0))
    {
      return false;
    }
  }
  return true;
}
//====================================================================== 
AmuPoint* Barrier::vertices(int i)
{
  return _vertices[i];
}

//====================================================================== 
AmuVector* Barrier::diagnoal(int i)
{
  return _diagnoal[i];
}
//====================================================================== 
Intersection* Barrier::intersection()
{
  return _intersection;
}  
//====================================================================== 
Section* Barrier::section(int i)
{
  return _section[i];
}  
