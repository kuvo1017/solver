#include "Barrier.h"
#include "Intersection.h"

using namespace std;
// ====================================
vector<Barrier*>* makeBarrier(Intersection* is){
  int nNext = is->numNext();
  for(int i=0;i<nNext;i++)
  {
    Intersection* nexts[2];
    for(int j=0;j<2;j++){
      if(i+j<4)
	nexts[j] = is->next(i+j);
      else
	nexts[j] = is->next(0);
    }
    Barrier* barrier = new Barrier(is,nexts[0],nexts[1]);
    _barriers->push_back(barrier);
  }
}

