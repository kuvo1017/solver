#include <iostream>
#include "Barrier.h"
#include "Intersection.h"
#include "Section.h"
#include "AmuPoint.h"

using namespace std;
// ====================================
Barrier::Barrier(Intersection* i0,
    Section* s1,
    Section* s2)
{
  cout << "enter into Barrier.cpp!!!!!"<<endl;
  cout << "Intersection id: " <<i0->id()<<endl;
  cout << "Section1 id: " <<s1->id()<<endl; 
  cout << "Section2 id: " <<s2->id()<<endl; 
  // IntersectionとSectionの4つの交点
  AmuPoint* rawVertices[4];
  // Barrierの4つの交点
  _intersection=i0;
  _section[0]=s1;
  _section[1]=s2;
  cout << "AmuPoint呼び出し" << endl;
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
	  rawVertices[0] = &sAmuPoint1;
	else
	  once1 = true;
      }
    }
    for(int j=0;j<sAmuPoints2.size();j++){
      AmuPoint sAmuPoint2 = sAmuPoints2[j];
      if(iAmuPoint.x()==sAmuPoint2.x()
	  && ( iAmuPoint.y()==sAmuPoint2.y()))
      {
	if(once2)
	  rawVertices[1] = &sAmuPoint2;
	else
	  once2 = false;
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
      if(i>0)
	rawVertices[3]=&sAmuPoints1[i-1];
      else
	rawVertices[3]=&sAmuPoints1[sAmuPoints1.size()-1];
    }
  }
  for(int i=0;i<sAmuPoints2.size();i++)
  {
    AmuPoint* sap = &sAmuPoints2[i];
    if(sap->x()==rawVertices[1]->x()
	&& ( sap->y()==rawVertices[1]->y())) 
    {
      if(i<sAmuPoints2.size())
	rawVertices[2]=&sAmuPoints2[i+1];
      else
	rawVertices[2]=&sAmuPoints2[0];
    }
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
  //std::cout << "傾きとかの計算終了" << endl;
  for(int i=0;i<4;i++)
  {
    //std::cout << "ループ処理開始" << endl; 
    AmuPoint* v = new AmuPoint;
    // 四角形の中心点から各点に向けて、縮尺をつける
    // 道路の端からどれくらい離すか
    double separate = 2.0; //[m]
    double t = 1-separate/sqrt(pow(rawVertices[i]->x()-center->x(),2)+pow(rawVertices[i]->y()-center->y(),2)); 
    std::cout <<i<< "kokoha?" <<center->x()<< endl;  
    v->setX((1-t)*center->x() + rawVertices[i]->x());
    v->setY((1-t)*center->y() + rawVertices[i]->y());  
    v->setZ(0.0);
    _vertices.push_back(v);
  }
  std::cout << "ループ処理終了" << endl;  
  std::cout << "rawVertices:"<<endl;
  for(int i=0;i<4;i++){
    if(_vertices[i]!=NULL)
      std::cout <<"("<< rawVertices[i]->x() <<","<<rawVertices[i]->y()<<")" <<std::endl;
  }
} 
//====================================================================== 
double Barrier::x(int i)
{
  return _vertices[i]->x();
}
///====================================================================== 
double Barrier::y(int i)
{
  return _vertices[i]->y();
}
///====================================================================== 
double Barrier::z(int i)
{
  return _vertices[i]->z();
}        

