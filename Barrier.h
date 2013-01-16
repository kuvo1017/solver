#ifndef __BARRIER_H__
#define __BARRIER_H__
#include <vector>
#include "AmuVector.h"
#include "Lane.h"


class Intersection;
class Section;
class AmuPoint;

typedef std::map<std::string,
	Lane*,
	std::less<std::string> > RMAPLAN;
typedef std::map<std::string,
	Lane*,
	std::less<std::string> >::iterator ITRMAPLAN;
typedef std::map<std::string,
	Lane*,
	std::less<std::string> >::const_iterator CITRMAPLAN;

class Barrier
{
  public:
    Barrier(Intersection* i0,
	Section* s1,
	Section* s2,
	std::string id);
    // 全ての車両に対して、視認できているかをチェック
    void checkVehiclesVisible();                          
    // 4つの頂点を返す 
    AmuPoint* vertices(int i);
    // 対角線を返す 
    AmuVector* diagnoal(int i);
    // 中心となる交差点を返す
    Intersection* intersection();
    // 交差点につながっている二本の単路を返す 
    Section* section(int i);
    // IDを返す
    std::string id();
  private:
    //ID
    std::string _id;
    // 4つの頂点
    std::vector<AmuPoint*> _vertices;
    // 対角線
    AmuVector* _diagnoal[2];
   //中心となる交差点
    Intersection* _intersection; 
   //  交差点につながっている二本の単路
    Section* _section[2];
    // 2つの車両が互いに目視できるかをチェック
    // 計算としては外積による２直線（車両同士の視線と、障害物の対角線)
    // の交差判定を用いている
    bool _isVisible(Vehicle* v1,Vehicle* v2);
 
};

#endif //__BARRIER_H__
