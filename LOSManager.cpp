#include "LOSManager.h"
#include <GeoManager.h>
#include <QuadTree.h>
#include "Vehicle.h"
#include "Lane.h"
#include <iostream>
#include <cstdlib>
#include <cassert>
#include <iomanip>

using namespace std;

//#define SHP_DEBUG

//======================================================================
LOSManager::LOSManager(){
  _numPolygons = 0;
  _numWalls    = 0;
  _numVehicles = 0;
  _hSHP = NULL;
  _hDBF = NULL;

  for (int i=0; i<4; i++) {
    _adfMaxBounds[i] = 200.0;
    _adfMinBounds[i] = -200.0;
  }
  for (int i=0; i<MAX_VEHICLES*MAX_VEHICLES; i++) {
    _los[i] = 0;
  }
}

//======================================================================
LOSManager::~LOSManager(){
  GeoManager::deleteAllTrees();
}

//======================================================================
void LOSManager::init(){
  /*
  string shpFileName = "sdo_building.shp";
  string dbfFileName = "sdo_building.dbf";
  
  // ファイルを読み込んで建造物の形状データを作成
  _readFiles(shpFileName, dbfFileName);
  */

  // ポリゴンをマニュアルで作成する
  _createBuildings();

  // ルートとなる四分木
  QuadTree* qt = new QuadTree(0,0);
  qt->set(_adfMinBounds[0], _adfMaxBounds[0],
	  _adfMinBounds[1], _adfMaxBounds[1]);
  GeoManager::addTree(qt);

  // ポリゴンを四分木に登録
  for (int i=0; i<_numPolygons; i++) {
    qt->pushQuadTree(GeoManager::polygon(i), i, 0, GeoManager::trees());
  }
}

//======================================================================
void LOSManager::calcVehicleLOS(vector<Vehicle*>* vehicles){
  // これまでの車体用四分木を消去
  GeoManager::deleteVTrees();
  _idMap.clear();

  QuadTree* vqt = new QuadTree();
  vqt->set(_adfMinBounds[0], _adfMaxBounds[0],
	   _adfMinBounds[1], _adfMaxBounds[1]);
  GeoManager::addVTree(vqt);

  vector<Vehicle*>::iterator itv;
  int id = 0;
  int numVehiclePolygons = 0;
  for (itv=vehicles->begin(); itv!=vehicles->end(); itv++) {
    int matesId = atoi((*itv)->id().c_str());
    LOSVehicle*  vehicle     = GeoManager::vehicle(id);
    VehicleWall* vehicleBody = GeoManager::vehicleWall(id);
    _idMap.insert(map<int,int>::value_type(matesId, id));
    vehicle->setAttribute(id, (*itv)->x(), (*itv)->y(), (*itv)->z(),
			  (*itv)->bodyLength(),
			  (*itv)->bodyWidth(),
			  (*itv)->bodyHeight());

    // 車体を直方体と見たときの8頂点
    // 車体中心を原点とみなしたとき
    MyVector3D p,v1,v2,v3,v4,v5,v6,v7,v8;
    p.set((*itv)->x(), (*itv)->y(), (*itv)->z());
    v1.set(-vehicle->length()*0.5,
	   -vehicle->width()*0.5,
	   0);
    v2.set( vehicle->length()*0.5,
	   -vehicle->width()*0.5,
	   0);
    v3.set( vehicle->length()*0.5,
	    vehicle->width()*0.5,
	   0);
    v4.set(-vehicle->length()*0.5,
	    vehicle->width()*0.5,
	   0);
    v5.set(-vehicle->length()*0.5,
	   -vehicle->width()*0.5,
	    vehicle->height());
    v6.set( vehicle->length()*0.5,
	   -vehicle->width()*0.5,
	    vehicle->height());
    v7.set( vehicle->length()*0.5,
	    vehicle->width()*0.5,
	    vehicle->height());
    v8.set(-vehicle->length()*0.5,
	    vehicle->width()*0.5,
	    vehicle->height());

    // 壁面の作成
    vehicleBody->setPolygon(0,p+v1, p+v5, p+v2);
    vehicleBody->setPolygon(1,v2+p, v6+p, v5+p);
    vehicleBody->setPolygon(2,v2+p, v3+p, v6+p);
    vehicleBody->setPolygon(3,v3+p, v7+p, v6+p);
    vehicleBody->setPolygon(4,v3+p, v4+p, v7+p);
    vehicleBody->setPolygon(5,v4+p, v8+p, v7+p);
    vehicleBody->setPolygon(6,v4+p, v1+p, v8+p);
    vehicleBody->setPolygon(7,v1+p, v5+p, v8+p);
    vehicleBody->setPolygon(8,v5+p, v6+p, v7+p);
    vehicleBody->setPolygon(9,v5+p, v8+p, v7+p);

    for (int i=0; i<10; i++) {
      GeoManager::vehiclePolygon(numVehiclePolygons)->setTriangle
	(vehicleBody->wall(i).getVector(0),
	 vehicleBody->wall(i).getVector(1),
	 vehicleBody->wall(i).getVector(2));
      vqt->pushQuadTree(GeoManager::vehiclePolygon(numVehiclePolygons),
			id*10+i, 0, GeoManager::vtrees());
      numVehiclePolygons++;
      /*  
      cout << "wall:" << i << endl;
      for (int j=0; j<3; j++) {
	cout << "vector:" << j << "-";
	cout << vehicleBody->wall(i).getVector(j).getX() << ","
	     << vehicleBody->wall(i).getVector(j).getY() << ","
	     << vehicleBody->wall(i).getVector(j).getZ() << endl;
      }
      */
    }
    id++;
  }
  _numVehicles = id;
  GeoManager::setNumVehicles(_numVehicles);
  GeoManager::setNumVehiclePolygons(numVehiclePolygons);
  //cout << "#vehicles = " << _numVehicles << endl;
  for (int t = 0; t<_numVehicles; t++) {
    for (int r=t+1; r<_numVehicles; r++) {
      LOSVehicle* vt = GeoManager::vehicle(t);
      LOSVehicle* vr = GeoManager::vehicle(r); 

#ifdef COLLISION_DEBUG
      /cout << "TRANSMIT:" << vt->id()
	   << " RECEIVE:" << vr->id() << endl;
#endif //COLLISION_DEBUG
      // 線分の傾きとy切片求める
      double a,b;
      _calcStraightLine(vt, vr, &a, &b);

      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // 建造物による遮蔽
      // チェック用四分木を作成
      ChkTree* cq = new ChkTree();
      // ルートノードから探索開始
      GeoManager::root()->checkNode(vt->vec(), vr->vec(), 0, cq, a, b);
      // 衝突判定
      int collisions = 0;
      double arrayT[_numWalls+_numVehicles*2];
      GeoManager::root()->countCollision(vt->vec(), vr->vec(), cq,
					 &collisions, arrayT);
      //      cout << "id is " << vt->id() << "collision is " << collisions <<endl;
      delete cq;

      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // 他車による遮蔽
      ChkTree* vcq = new ChkTree();
      vqt->checkNode(vt->vec(), vr->vec(), 0, vcq, a, b);
      vqt->countCollision(vt, vr, vcq, &collisions, arrayT);
      delete vcq;
      
      _los[t*MAX_VEHICLES + r] = collisions;
      _los[r*MAX_VEHICLES + t] = collisions;

    }
  }
}
//======================================================================
void LOSManager::checkLanes(vector<Vehicle*>* vehicles){
  // これまでの車体用四分木を消去
  GeoManager::deleteVTrees();
  _idMap.clear();
  _idMapR.clear();

  QuadTree* vqt = new QuadTree();
  vqt->set(_adfMinBounds[0], _adfMaxBounds[0],
	   _adfMinBounds[1], _adfMaxBounds[1]);
  GeoManager::addVTree(vqt);

  vector<Vehicle*>::iterator itv;
  int id = 0;
  int numVehiclePolygons = 0;
  for (itv=vehicles->begin(); itv!=vehicles->end(); itv++) {
    int matesId = atoi((*itv)->id().c_str());
    LOSVehicle*  vehicle     = GeoManager::vehicle(id);
    VehicleWall* vehicleBody = GeoManager::vehicleWall(id);
    _idMap.insert(map<int,int>::value_type(matesId, id));
    _idMapR.insert(map<int,int>::value_type(id, matesId));
    
    vehicle->setAttribute(id, (*itv)->x(), (*itv)->y(), (*itv)->z(),
			  (*itv)->bodyLength(),
			  (*itv)->bodyWidth(),
			  (*itv)->bodyHeight());

    // 車体を直方体と見たときの8頂点
    // 車体中心を原点とみなしたとき
    MyVector3D p,v1,v2,v3,v4,v5,v6,v7,v8;
    p.set((*itv)->x(), (*itv)->y(), (*itv)->z());
    v1.set(-vehicle->length()*0.5,
	   -vehicle->width()*0.5,
	   0);
    v2.set( vehicle->length()*0.5,
	   -vehicle->width()*0.5,
	   0);
    v3.set( vehicle->length()*0.5,
	    vehicle->width()*0.5,
	   0);
    v4.set(-vehicle->length()*0.5,
	    vehicle->width()*0.5,
	   0);
    v5.set(-vehicle->length()*0.5,
	   -vehicle->width()*0.5,
	    vehicle->height());
    v6.set( vehicle->length()*0.5,
	   -vehicle->width()*0.5,
	    vehicle->height());
    v7.set( vehicle->length()*0.5,
	    vehicle->width()*0.5,
	    vehicle->height());
    v8.set(-vehicle->length()*0.5,
	    vehicle->width()*0.5,
	    vehicle->height());

    // 壁面の作成
    vehicleBody->setPolygon(0,p+v1, p+v5, p+v2);
    vehicleBody->setPolygon(1,v2+p, v6+p, v5+p);
    vehicleBody->setPolygon(2,v2+p, v3+p, v6+p);
    vehicleBody->setPolygon(3,v3+p, v7+p, v6+p);
    vehicleBody->setPolygon(4,v3+p, v4+p, v7+p);
    vehicleBody->setPolygon(5,v4+p, v8+p, v7+p);
    vehicleBody->setPolygon(6,v4+p, v1+p, v8+p);
    vehicleBody->setPolygon(7,v1+p, v5+p, v8+p);
    vehicleBody->setPolygon(8,v5+p, v6+p, v7+p);
    vehicleBody->setPolygon(9,v5+p, v8+p, v7+p);

    for (int i=0; i<10; i++) {
      GeoManager::vehiclePolygon(numVehiclePolygons)->setTriangle
	(vehicleBody->wall(i).getVector(0),
	 vehicleBody->wall(i).getVector(1),
	 vehicleBody->wall(i).getVector(2));
      vqt->pushQuadTree(GeoManager::vehiclePolygon(numVehiclePolygons),
			id*10+i, 0, GeoManager::vtrees());
      numVehiclePolygons++;
      /*  
      cout << "wall:" << i << endl;
      for (int j=0; j<3; j++) {
	cout << "vector:" << j << "-";
	cout << vehicleBody->wall(i).getVector(j).getX() << ","
	     << vehicleBody->wall(i).getVector(j).getY() << ","
	     << vehicleBody->wall(i).getVector(j).getZ() << endl;
      }
      */
    }
    id++;
  }
  _numVehicles = id;
  GeoManager::setNumVehicles(_numVehicles);
  GeoManager::setNumVehiclePolygons(numVehiclePolygons);
  //cout << "#vehicles = " << _numVehicles << endl;
  for (int t = 0; t<_numVehicles; t++) {
    for (int r=t+1; r<_numVehicles; r++) {
      LOSVehicle* vt = GeoManager::vehicle(t);
      LOSVehicle* vr = GeoManager::vehicle(r); 

#ifdef COLLISION_DEBUG
      /cout << "TRANSMIT:" << vt->id()
	   << " RECEIVE:" << vr->id() << endl;
#endif //COLLISION_DEBUG
      // 線分の傾きとy切片求める
      double a,b;
      _calcStraightLine(vt, vr, &a, &b);

      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // 建造物による遮蔽
      // チェック用四分木を作成
      ChkTree* cq = new ChkTree();
      // ルートノードから探索開始
      GeoManager::root()->checkNode(vt->vec(), vr->vec(), 0, cq, a, b);
      // 衝突判定
      int collisions = 0;
      double arrayT[_numWalls+_numVehicles*2];
      GeoManager::root()->countCollision(vt->vec(), vr->vec(), cq,
					 &collisions, arrayT);
      cout << "collisions is " << collisions
	   << " matesId is " << _idMapR[vt->id()] << endl;
      if(collisions > 0){
	int matesIdt = _idMapR[vt->id()];
	Lane* lanet = vehicles->at(matesIdt)->lane();
	lanet->nearWall();
	int matesIdr = _idMapR[vr->id()];
	Lane* lanev = vehicles->at(matesIdr)->lane();
	lanev->nearWall();
	
	cout << "(near Wall T) x is " << vehicles->at(matesIdt)->x() 
	     << " y is " << vehicles->at(matesIdt)->y() 
	     << " z is " << vehicles->at(matesIdt)->z() <<endl;
	cout << "(near Wall V) x is " << vehicles->at(matesIdr)->x() 
	     << " y is " << vehicles->at(matesIdr)->y() 
	     << " z is " << vehicles->at(matesIdr)->z() <<endl;

      } 

      delete cq;

      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // 他車による遮蔽
      ChkTree* vcq = new ChkTree();
      vqt->checkNode(vt->vec(), vr->vec(), 0, vcq, a, b);
      vqt->countCollision(vt, vr, vcq, &collisions, arrayT);
      delete vcq;
      
      _los[t*MAX_VEHICLES + r] = collisions;
      _los[r*MAX_VEHICLES + t] = collisions;

    }
  }

  /*
  for (int t=0; t<_numVehicles; t++) {
    cout << "t=" << t << ":";
    for (int r=0; r<_numVehicles; r++) {
      if (t==r) cout << " --";
      else cout << " " << setw(2) << _los[t*MAX_VEHICLES+r];
    }
    cout << endl;
  }
  /*/
}

//======================================================================
void LOSManager::field(double *xmin, double *xmax,
		       double *ymin, double *ymax) const{
  *xmin = _adfMinBounds[0];
  *xmax = _adfMaxBounds[0];
  *ymin = _adfMinBounds[1];
  *ymax = _adfMaxBounds[1];
}

//======================================================================
int LOSManager::lookupLOS(int srcId, int dstId){
  int t = _idMap[srcId];
  int r = _idMap[dstId];
  return _los[t*MAX_VEHICLES + r];
}

//======================================================================
void LOSManager::_readFiles(string shpFileName, string dbfFileName){
  // SHPファイルを開く
  _hSHP = SHPOpen(shpFileName.c_str(), "rb");
  if (_hSHP==NULL) {
    cerr << "cannot open " << shpFileName << endl;
    exit(EXIT_FAILURE);
  }
  
  int nEntities;  // 要素数
  int nShapeType; // 要素種別

  // 要素数と要素種別を取得する
  SHPGetInfo(_hSHP, &nEntities, &nShapeType,
	     _adfMinBounds, _adfMaxBounds);

#ifdef SHP_DEBUG  
  cout << nEntities << endl;
  cout << nShapeType << endl;
  for (int i=0; i<4; i++) {
    cout << _adfMinBounds[i] << "," << _adfMaxBounds[i] << endl;
  }
#endif //SHP_DEBUG
  
  // dbfファイルを開く
  _hDBF = DBFOpen(dbfFileName.c_str(), "rb");
  if (_hDBF==NULL) {
    cerr << "cannot open " << dbfFileName << endl;
    exit(EXIT_FAILURE);
  }
  if (DBFGetFieldCount(_hDBF)==0) {
    cerr << "The are no field in this table." << endl;
    exit(EXIT_FAILURE);
  }

  // 図形データの読み込み
  SHPObject *psShape;
  //for (int i=0; i<1; i++) {
  for (int i=0; i<nEntities; i++) {
    psShape = SHPReadObject(_hSHP, i);
    for (int j=0, iPart=1; j<psShape->nVertices; j++) {
      if (iPart<psShape->nParts
	  && psShape->panPartStart[iPart]==j+1) {
	iPart++;
      }
      else if (j<psShape->nVertices-1){
	_makePolygon(i, j, psShape);
      }
    }
    _numWalls += psShape->nVertices - psShape->nParts;
    SHPDestroyObject(psShape);
  }
  cout << "#polygons = " << _numPolygons << endl;
  cout << "#walls    = " << _numWalls    << endl;
  GeoManager::setNumPolygons(_numPolygons);
  GeoManager::setNumWalls(_numWalls);

  SHPClose(_hSHP);
  DBFClose(_hDBF);
}

//======================================================================
void LOSManager::_createBuildings(){
  GeoManager::polygon(_numPolygons)->setTriangle
    (5,  -5,10,
     5,  -5,10,
     5,-100, 0);
  _numPolygons++;
  GeoManager::polygon(_numPolygons)->setTriangle
    (5,  -5,10,
     5,-100, 0,
     5,-100,10);
  _numPolygons++;
  GeoManager::polygon(_numPolygons)->setTriangle
    (  5,-5, 0,
     100,-5, 0,
       5,-5,10);
  _numPolygons++;
  GeoManager::polygon(_numPolygons)->setTriangle
    (  5,-5,10,
     100,-5, 0,
     100,-5,10);
  _numPolygons++;
  GeoManager::polygon(_numPolygons)->setTriangle
    (  5,  -5,10,
       5,-100,10,
     100,-100,10);
  _numPolygons++;
  GeoManager::polygon(_numPolygons)->setTriangle
    (  5,  -5,10,
     100,-100,10,
     100,  -5,10);
  _numPolygons++;

  _numWalls = _numPolygons/2;
  cout << "#polygons = " << _numPolygons << endl;
  cout << "#walls    = " << _numWalls    << endl;
  GeoManager::setNumPolygons(_numPolygons);
  GeoManager::setNumWalls(_numWalls);
}

//======================================================================
void LOSManager::_makePolygon(int entityId, int partId, SHPObject* shape){
  assert(_hDBF);

  GeoManager::polygon(_numPolygons)->setTriangle
    (shape->padfX[partId],
     shape->padfY[partId],
     0.0,
     shape->padfX[partId],
     shape->padfY[partId],
     DBFReadIntegerAttribute(_hDBF, entityId, HEIGHTDBF),
     shape->padfX[partId+1],
     shape->padfY[partId+1],
     0.0);
  _numPolygons++;
  GeoManager::polygon(_numPolygons)->setTriangle
    (shape->padfX[partId+1],
     shape->padfY[partId+1],
     0.0,
     shape->padfX[partId+1],
     shape->padfY[partId+1],
     DBFReadIntegerAttribute(_hDBF, entityId, HEIGHTDBF),
     shape->padfX[partId],
     shape->padfY[partId],
     DBFReadIntegerAttribute(_hDBF, entityId, HEIGHTDBF));
  _numPolygons++;

}

//======================================================================
void LOSManager::_calcStraightLine(LOSVehicle* vt, LOSVehicle* vr,
				   double* a, double* b) {
  *a = (vt->y()-vr->y())/(vt->x()-vr->x());
  *b = vt->y() - *a * vt->x();
}
