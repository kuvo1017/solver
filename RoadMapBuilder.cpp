#include "RoadMapBuilder.h"
#include "RoadMap.h"
#include "GVManager.h"
#include "AmuStringOperator.h"
#include "AmuConverter.h"
#include "LaneBundle.h"
#include "Intersection.h"
#include "ODNode.h"
#include "Section.h"
#include "Lane.h"
#include "RoadEntity.h"
#include "Signal.h"
#include "SignalIO.h"
#include "AmuPoint.h"
#include "Conf.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <vector>

using namespace std;

//======================================================================
RoadMapBuilder::RoadMapBuilder()
{
    _currentRoadMap = 0;

    _isIntersectionCreated          = false;
    _isConnectionSet                = false;
    _isPositionSet                  = false;
    _isIntersectionStructInfoSet    = false;
    _isIntersectionStructureCreated = false;

    _isSectionCreated          = false;
    _isSectionStructInfoSet    = false;
    _isSectionStructureCreated = false;

    _isNetworkCreated        = false;
    _isSubnetworkCreated     = false;
    _isLaneConnectionCreated = false;
}

//======================================================================
RoadMapBuilder::~RoadMapBuilder()
{
    if (_currentRoadMap!=NULL)
    {
        delete _currentRoadMap;
    }
}

//======================================================================
void RoadMapBuilder::buildRoadMap()
{
    if (_currentRoadMap!=NULL)
    {
        delete _currentRoadMap;
    }
    _currentRoadMap = new RoadMap();

    // 交差点(Intersection)を作成する
    _isIntersectionCreated = buildIntersections();
    if (!_isIntersectionCreated)
    {
        cerr << "Error: create intersections failed." << endl;
    }

    // 単路(Section)を作成する
    _isSectionCreated = buildSections();
    if (!_isSectionCreated)
    {
        cerr << "Error: create sections failed." << endl;
    }

    // ネットワークを作成する
    _isNetworkCreated = buildNetwork();
    if (!_isNetworkCreated)
    {
        cerr << "Error: create network failed" << endl;
    }

    // 単路の構造情報を設定する、ファイルを読み込む、依存関係のため単路が先
    _isSectionStructInfoSet = setSectionStructInfo(true);
    if (!_isSectionStructInfoSet)
    {
        cerr << "Error: struct info of section not set." << endl;
    }

    // 交差点の構造情報を設定する、ファイルを読み込む
    _isIntersectionStructInfoSet = setIntersectionStructInfo(true);
    if (!_isIntersectionStructInfoSet)
    {
        cerr << "Error: struct info of intersection not set." << endl;
    }

    // 交差点の詳細構造を設定する
    cout << endl << "*** Intersection Information ***" << endl;
    _isIntersectionStructureCreated = createIntersectionStructure();
    if (!_isIntersectionStructureCreated)
    {
        cerr << "Error: internal structure of intersections not created.";
    }
    cout << "kitemasu!!!!!!!!!!!" <<endl;
    // 交差点が信号を持たない場合に設定設定する
    if(!checkSignals())
    {
        cerr << "Error: internal structure of intersections not created.";
    } 
 
    cout << endl;
    // 単路の内部構造を設定する
    _isSectionStructureCreated = createSectionStructure();
    if (!_isSectionStructureCreated)
    {
        cerr << "Error: internal structure of intersections not created."
             << endl;
    }

    // レーンの接続情報を設定する
    _isLaneConnectionCreated = buildLaneConnection();
    if (!_isLaneConnectionCreated)
    {
        cerr << "Error: create lane connection failed." << endl;
    }

    // サブネットワークを作成する
    // 交差点と単路が作成済でなければならない
    _isSubnetworkCreated = buildSubnetwork();
    if (!_isSubnetworkCreated)
    {
        cerr << "Error: create subnetwork failed." << endl;
    }
}

//======================================================================
void RoadMapBuilder::buildGridRoadMap(double xmin, double xmax,
				      double ymin, double ymax,
				      double xsp,  double ysp)
{
    if (_currentRoadMap!=NULL)
    {
        delete _currentRoadMap;
    }
    _currentRoadMap = new RoadMap();

    // 道路作成可能な引数かチェックする
    if (xsp>(xmax-xmin)/2)
    {
        cout << "xsp must be less than or equal to "
             << (xmax-xmin)/2 << endl;
        exit(EXIT_FAILURE);
    }
    if (ysp>(ymax-ymin)/2)
    {
        cout << "ysp must be less than or equal to "
             << (ymax-ymin)/2 << endl;
        exit(EXIT_FAILURE);
    }

    // 交差点(Intersection)を作成する
    {
        unsigned int id   = 0;
        unsigned int nrow = 0; // 行数
        unsigned int ncol = 0; // 列数
        // 交差点の生成と中心座標のセット
        id++;
        for (double x=xmin+xsp; x<=xmax-xsp; x+=xsp)
        {
            Intersection* ptInter
                = new ODNode
                (AmuConverter::itos(id, NUM_FIGURE_FOR_INTERSECTION), 
                 AmuConverter::itos(11, 2),
                 _currentRoadMap);
            ptInter->addCenter(AmuPoint(x,ymax,0));
            _currentRoadMap->addIntersection(ptInter);
            id++;
            ncol++;
        }
        id++;
        nrow++;
        ncol += 2;
        for (double y=ymax-ysp; y>=ymin+ysp; y-=ysp)
        {
            {
                Intersection* ptInter
                    = new ODNode
                    (AmuConverter::itos(id, NUM_FIGURE_FOR_INTERSECTION),
                     AmuConverter::itos(11, 2),
                     _currentRoadMap);
                ptInter->addCenter(AmuPoint(xmin,y,0));
                _currentRoadMap->addIntersection(ptInter);
                id++;
            }
            for (double x=xmin+xsp; x<=xmax-xsp; x+=xsp)
            {
                Intersection* ptInter
                    = new Intersection
                    (AmuConverter::itos(id,
                                        NUM_FIGURE_FOR_INTERSECTION),
                     AmuConverter::itos(11111111, 8),
                     _currentRoadMap);
                ptInter->addCenter(AmuPoint(x,y,0));
                _currentRoadMap->addIntersection(ptInter);
                id++;
            }
            {
                Intersection* ptInter
                    = new ODNode
                    (AmuConverter::itos(id,
                                        NUM_FIGURE_FOR_INTERSECTION),
                     AmuConverter::itos(11, 2),
                     _currentRoadMap);
                ptInter->addCenter(AmuPoint(xmax,y,0));
                _currentRoadMap->addIntersection(ptInter);
                id++;
            }
            nrow++;
        }
        id++;
        for (double x=xmin+xsp; x<=xmax-xsp; x+=xsp)
        {
            Intersection* ptInter
                = new ODNode
                (AmuConverter::itos(id,
                                    NUM_FIGURE_FOR_INTERSECTION),
                 AmuConverter::itos(11, 2),
                 _currentRoadMap);
            ptInter->addCenter(AmuPoint(x,ymin,0));
            _currentRoadMap->addIntersection(ptInter);
            id++;
        }
        nrow++;

        id = 0;
        //交差点の接続情報のセット
        id++;
        for (unsigned int j=1; j<=ncol-2; j++)
        {
            Intersection* ptInter
                = _currentRoadMap
                ->intersection
                (AmuConverter::itos(id,
                                    NUM_FIGURE_FOR_INTERSECTION));
            ptInter->setNext
                (_currentRoadMap
                 ->intersection
                 (AmuConverter::itos(id+ncol,
                                     NUM_FIGURE_FOR_INTERSECTION)));
            id++;
        }
        id++;
        for (unsigned int i=1; i<=nrow-2; i++)
        {
            {
                Intersection* ptInter
                    = _currentRoadMap
                    ->intersection
                    (AmuConverter::itos(id,
                                        NUM_FIGURE_FOR_INTERSECTION));
                ptInter->setNext
                    (_currentRoadMap
                     ->intersection
                     (AmuConverter::itos(id+1,
                                         NUM_FIGURE_FOR_INTERSECTION)));
                id++;
            }
            for (unsigned int j=1; j<=ncol-2; j++)
            {
                Intersection* ptInter
                    = _currentRoadMap
                    ->intersection
                    (AmuConverter::itos(id,
                                        NUM_FIGURE_FOR_INTERSECTION));
                ptInter->setNext
                    (_currentRoadMap
                     ->intersection
                     (AmuConverter::itos(id-ncol,
                                         NUM_FIGURE_FOR_INTERSECTION)));
                ptInter->setNext
                    (_currentRoadMap
                     ->intersection
                     (AmuConverter::itos(id-1,
                                         NUM_FIGURE_FOR_INTERSECTION)));
                ptInter->setNext
                    (_currentRoadMap
                     ->intersection
                     (AmuConverter::itos(id+ncol,
                                         NUM_FIGURE_FOR_INTERSECTION)));
                ptInter->setNext
                    (_currentRoadMap
                     ->intersection
                     (AmuConverter::itos(id+1,
                                         NUM_FIGURE_FOR_INTERSECTION)));
                id++;
            }
            {
                Intersection* ptInter
                    = _currentRoadMap
                    ->intersection
                    (AmuConverter::itos(id,
                                        NUM_FIGURE_FOR_INTERSECTION));
                ptInter->setNext
                    (_currentRoadMap
                     ->intersection
                     (AmuConverter::itos(id-1,
                                         NUM_FIGURE_FOR_INTERSECTION)));
                id++;
            }
        }
        id++;
        for (unsigned int j=1; j<=ncol-2; j++)
        {
            Intersection* ptInter
                = _currentRoadMap
                ->intersection
                (AmuConverter::itos(id,
                                    NUM_FIGURE_FOR_INTERSECTION));
            ptInter->setNext
                (_currentRoadMap
                 ->intersection
                 (AmuConverter::itos(id-ncol,
                                     NUM_FIGURE_FOR_INTERSECTION)));
            id++;
        }
        _isIntersectionCreated = true;
        _isPositionSet = true;
        _isConnectionSet = true;
    }

    // 単路(Section)を作成する
    _isSectionCreated = buildSections();
    if (!_isSectionCreated)
    {
        cerr << "Error: create sections failed." << endl;
    }

    // ネットワークを作成する
    _isNetworkCreated = buildNetwork();
    if (!_isNetworkCreated)
    {
        cerr << "Error: create network failed";
    }

    // 単路の構造情報を設定する、ファイルを読まない、依存関係のため単路が先
    _isSectionStructInfoSet = setSectionStructInfo(false);
    if (!_isSectionStructInfoSet)
    {
        cerr << "Error: struct info of section not set." << endl;
    }

    // 交差点の構造情報を設定する、ファイルを読まない
    _isIntersectionStructInfoSet = setIntersectionStructInfo(false);
    if (!_isIntersectionStructInfoSet)
    {
        cerr << "Error: struct info of intersection not set." << endl;
    }

    // 交差点の詳細構造を設定する
    _isIntersectionStructureCreated = createIntersectionStructure();
    if (!_isIntersectionStructureCreated)
    {
        cerr << "Error: internal structure of intersections not created.";
    }
 cout << "kitemasu!!!!!!!!!!!" <<endl;
    // 交差点が信号を持たない場合に設定設定する
    if(!checkSignals())
    {
        cerr << "Error: internal structure of intersections not created.";
    } 
 
   // 単路の内部構造を設定する
    _isSectionStructureCreated = createSectionStructure();
    if (!_isSectionStructureCreated)
    {                  
        cerr << "Error: internal structure of intersections not created.";
    }

    // サブネットワークを作成する
    // 交差点と単路が作成済でなければならない
    _isSubnetworkCreated = buildSubnetwork();
    if (!_isSubnetworkCreated)
    {
        cerr << "Error: create subnetwork failed." << endl;
    }
}


//======================================================================
bool RoadMapBuilder::buildIntersections()
{
    // 設定ファイルの名前を取得する
    string fMapPosition, fNetwork;

    GVManager::getVariable("MAP_POSITION_FILE", &fMapPosition);
    GVManager::getVariable("MAP_NETWORK_FILE", &fNetwork);

    // 交差点の生成
    _isIntersectionCreated = createIntersection(fNetwork);
    if (!_isIntersectionCreated)
    {
        cerr << "Error: intersections not created." << endl; 
        return false;
    }

    // 接続情報の設定
    _isConnectionSet = setConnection(fNetwork);
    if (!_isConnectionSet)
    {
        cerr << "Error: connection between intersections not set." << endl;
        return false;
    }

    // 座標の設定
    _isPositionSet = setPosition(fMapPosition);
    if (!_isPositionSet)
    {
        cerr << "Error: position of intersection not set." << endl;
        return false;
    }

    return true;
}

//======================================================================
bool RoadMapBuilder::createIntersection(const string& fNetwork)
{
    if (!_isIntersectionCreated)
    {
        assert(_currentRoadMap);

        // ファイルを読み込む
        ifstream inNetworkFile(fNetwork.c_str(), ios::in);
        if (!inNetworkFile)
        {
            cerr << "Error: cannot open file" << fNetwork << "." << endl;
            exit(EXIT_FAILURE);
        }

        // 交差点の生成
        string str;
        while (inNetworkFile.good())
        {
            getline(inNetworkFile, str);
            AmuStringOperator::getAdjustString(&str);
            if (!str.empty())
            {
                vector<string> tokens;
                string id;
                string type;
                Intersection* ptInter;
	
                AmuStringOperator::getTokens(&tokens, str, ',');
                assert(tokens.size()>2);
                // 1番目のカラムは交差点の識別番号
                id = tokens[0];
                // 2番目のカラムは交差点のタイプ
                type = tokens[1];
	
                if (type.length()==2)
                {
                    ptInter
                        = new ODNode
                        (AmuConverter::formatId
                         (id, NUM_FIGURE_FOR_INTERSECTION),
                         type,
                         _currentRoadMap);
                }
                else
                {
                    ptInter
                        = new Intersection
                        (AmuConverter::formatId
                         (id, NUM_FIGURE_FOR_INTERSECTION),
                         type,
                         _currentRoadMap);
                }
                _currentRoadMap->addIntersection(ptInter);
            }
        }
        inNetworkFile.close();
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::setConnection(const string& fNetwork)
{
    if (!_isConnectionSet)
    {
        assert(_currentRoadMap);
        assert(_isIntersectionCreated);

        // ファイルを読み込む
        ifstream inNetworkFile(fNetwork.c_str(), ios::in);
        if (!inNetworkFile)
        {
            cerr << "Error: cannot open file" << fNetwork << "." << endl;
            exit(EXIT_FAILURE);
        }

        // 交差点の接続情報をセット
        string str;
        while (inNetworkFile.good())
        {
            getline(inNetworkFile, str);
            AmuStringOperator::getAdjustString(&str);
            if (!str.empty())
            {
                vector<string> tokens;
                string id;
                string type;
	
                AmuStringOperator::getTokens(&tokens, str, ',');
                assert(tokens.size()>2);
                id = tokens[0];
                type = tokens[1];

                Intersection* ptInter = _currentRoadMap
                    ->intersection
                    (AmuConverter::formatId
                     (id, NUM_FIGURE_FOR_INTERSECTION));

                // 隣接交差点のIDをセットする
                for (unsigned int i=2; i< tokens.size(); i++)
                {
                    ptInter->setNext
                        (_currentRoadMap->intersection
                         (AmuConverter::formatId
                          (tokens[i],NUM_FIGURE_FOR_INTERSECTION)));
                }
            }
        }
        inNetworkFile.close();
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::setPosition(const string& fMapPosition)
{
    if (!_isPositionSet)
    {
        assert(_currentRoadMap);
        assert(_isIntersectionCreated);

        //ファイルを読み込む
        ifstream inPositionFile(fMapPosition.c_str(), ios::in);
        if (!inPositionFile)
        {
            cerr << "Error: cannot open file " << fMapPosition << "."
                 << endl;
            exit(EXIT_FAILURE);
        }

        //交差点の座標をセット
        string str;
        while (inPositionFile.good())
        {
            getline(inPositionFile, str);
            AmuStringOperator::getAdjustString(&str);
            if (!str.empty())
            {
                vector<string> tokens;
                string id;
                double x = 0, y = 0, z = 0;

                AmuStringOperator::getTokens(&tokens, str, ',');
                assert(tokens.size()>=3);
                id = tokens[0];
                x = static_cast<double>(atof(tokens[1].c_str()));
                y = static_cast<double>(atof(tokens[2].c_str()));
                //z座標はオプション
                if (tokens.size()==4)
                {
                    z = static_cast<double>(atof(tokens[3].c_str()));
                }
                else
                {
                    z = 0;
                }

                //値をセットする
                if (_currentRoadMap
                    ->intersection
                    (AmuConverter::formatId
                     (id, NUM_FIGURE_FOR_INTERSECTION)) != NULL)
                {
                    _currentRoadMap
                        ->intersection
                        (AmuConverter::formatId
                         (id, NUM_FIGURE_FOR_INTERSECTION))
                        ->addCenter(AmuPoint(x,y,z));
                }
            }
        }
        inPositionFile.close();
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::setIntersectionStructInfo(bool readFile)
{
    if (!_isIntersectionStructureCreated)
    {
        assert(_currentRoadMap);
        assert(_isIntersectionCreated);

        string fStruct;
        double sidewalkWidth, crosswalkWidth;
        GVManager::getVariable("INTERSECTION_STRUCT_FILE", &fStruct);

        // ファイルを読み込む
        ifstream inStructFile;
        if (readFile)
        {
            inStructFile.open(fStruct.c_str(), ios::in);
            if (!inStructFile)
            {
                cout << "no intersection struct file: " << fStruct << endl;
            }
        }
        if (readFile && inStructFile)
        {
            // 交差点の構造情報を設定する
            string str;
            while (inStructFile.good())
            {
                getline(inStructFile, str);
                AmuStringOperator::getAdjustString(&str);
                if (!str.empty())
                {
                    vector<string> tokens;
                    string id;

                    AmuStringOperator::getTokens(&tokens, str, ',');
                    if (tokens.size() != 3)
                    {
                        cerr << "Error: intersection struct invalid token number "
                             << tokens.size() << endl;
                    }
                    else
                    {
                        // 1番目のカラムは交差点の識別番号
                        id = AmuConverter::formatId
                            (tokens[0], NUM_FIGURE_FOR_INTERSECTION);
                        // 2番目のカラムは歩道幅
                        sidewalkWidth  = atof
                            (tokens[1].c_str());
                        // 3番目のカラムは横断歩道幅
                        crosswalkWidth = atof(tokens[2].c_str());
                        Intersection* intersection
                            = _currentRoadMap->intersection(id);
                        if (!intersection)
                        {
                            cerr << "Error: intersection struct invalid input - "
                                 << "intersection " << id
                                 << " does not exist." << endl;
                        }
                        else
                        {
                            if (!intersection->setStructInfo(sidewalkWidth, crosswalkWidth))
                            {
                                cerr << "Error: intersection struct invalid input of intersection "
                                     << id << endl;
                            }
                        }
                    }
                }
            }
            inStructFile.close();
        }

        // 単路の構造情報に既定値を設定する、設定済みなら内部で無視する
        // 車道頂点を先に作るために基礎構造を生成する
        CITRMAPI iti = _currentRoadMap->intersections()->begin();
        sidewalkWidth  = GVManager::getNumeric("DEFAULT_SIDEWALK_WIDTH");
        crosswalkWidth = GVManager::getNumeric("DEFAULT_CROSSWALK_WIDTH");
        while (iti != _currentRoadMap->intersections()->end())
        {
            if (!(*iti).second->setStructInfo(sidewalkWidth, crosswalkWidth))
            {
                cerr << "intersection " << (*iti).second->id()
                     << ": default struct info error." << endl;
            }
            if (!(*iti).second->createBaseStructure(readFile))
            {
                cerr << "intersection " << (*iti).second->id()
                     << ": create base structure error." << endl;
            }
            iti++;
        }
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::createIntersectionStructure()
{
    if (!_isIntersectionStructureCreated)
    {
        assert(_currentRoadMap);
        assert(_isIntersectionCreated);
        assert(_isIntersectionStructInfoSet);
        assert(_isConnectionSet);

        bool check = false;

        CITRMAPI iti = _currentRoadMap->intersections()->begin();
        while (iti != _currentRoadMap->intersections()->end())
        {
            // 各Intersectionのcreate関数を呼び出す
            check = (*iti).second->create();
            if (!check)
            {
                cerr << "intersection " << (*iti).second->id()
                     << ": internal structure error." << endl;
                return false;
            }
            iti++;
        }
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::checkSignals()
{                         
  cout << "!!!!!!checkSignal!!!!!" <<endl; 
  string fNoSignal;
 
  GVManager::getVariable("NOSIGNAL_FILE", &fNoSignal);

  cout << "fNoSignal:" << fNoSignal <<endl;
  ifstream inNoSignalFile(fNoSignal.c_str(), ios::in);
   
  string str;
  while (inNoSignalFile.good())
  {
    getline(inNoSignalFile, str);
    AmuStringOperator::getAdjustString(&str);
    if (!str.empty())
    {
      cout << "kiteruyo2" <<endl;
      vector<string> tokens;
      AmuStringOperator::getTokens(&tokens, str, ',');
      assert(tokens.size()==1);
      // 3番目のカラムは終点となる交差点の識別番号
      std::string id = tokens[0].c_str();
      cout << "id:" << id <<endl;
      CITRMAPI iti = _currentRoadMap->intersections()->begin();
      while (iti != _currentRoadMap->intersections()->end())
      {
            if((*iti).second->id() == id )
	    {
	      cout << "intersection " << (*iti).second->id()
		<< " has no signal" << endl;
	      (*iti).second-> noSignal();
	    }
	    iti++;
	}
    }
  }
  return true;
}
 
//======================================================================
bool RoadMapBuilder::buildSections()
{
    // 単路の生成
    _isSectionCreated = createSection();
    if (!_isSectionCreated)
    {
        cerr << "Error: Section not created." << endl; 
        return false;
    }

    return true;
}

//======================================================================
bool RoadMapBuilder::createSection()
{
    if (!_isSectionCreated)
    {
        // 接続関係を持つ交差点間を結ぶ単路を作成する
        CITRMAPI iti = _currentRoadMap->intersections()->begin();
        while (iti != _currentRoadMap->intersections()->end())
        {
            for (int i=0;
                 i<static_cast<signed int>((*iti).second->numNext());
                 i++)
            {
                // IDを決定する
                Intersection* begin = NULL;
                Intersection* end   = NULL;
                string id = "";

                // 該当の交差点と接続先の交差点のIDを比べ、
                // 該当交差点のIDが小さい場合のみ単路を作成
                if ((*iti).second->id() < (*iti).second->next(i)->id())
                {
                    id = (*iti).second->id()
                        + (*iti).second->next(i)->id();
                    begin
                        = const_cast<Intersection*>
                        ((*iti).second);
                    end
                        = const_cast<Intersection*>
                        ((*iti).second->next(i));
                    assert(begin!=NULL && end!=NULL);

                    // 既にSectionオブジェクトが作られていないかチェックする
                    // 重複する単路を認めない
                    if (_currentRoadMap->section(id)==NULL)
                    {
                        Section* ptSection
                            = new Section(id, begin, end, _currentRoadMap);
                        _currentRoadMap->addSection(ptSection);
                    }
                    else
                    {
                        cerr << "intersection " << (*iti).second->id()
                             << " to intersection "
                             << (*iti).second->next(i)->id()
                             << " section error." << endl;
                        return false;
                    }
                }
            }
            iti++;
        }
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::setSectionStructInfo(bool readFile)
{
    if (!_isSectionStructureCreated)
    {
        assert(_currentRoadMap);
        assert(_isSectionCreated);

        string fStruct;
        double laneWidth, roadsideWidth;
        double sidewalkWidthRight, sidewalkWidthLeft;
        GVManager::getVariable("SECTION_STRUCT_FILE", &fStruct);
        int autoSidewalkLane =
            (int)GVManager::getNumeric("AUTO_SIDEWALK_SECTION_LANE");

        // ファイルを読み込む
        ifstream inStructFile;
        if (readFile)
        {
            inStructFile.open(fStruct.c_str(), ios::in);
            if (!inStructFile)
            {
                cout << "no section struct file: " << fStruct << endl;
            }
        }
        if (readFile && inStructFile)
        {
            // 単路の構造情報を設定する
            string str;
            while (inStructFile.good())
            {
                getline(inStructFile, str);
                AmuStringOperator::getAdjustString(&str);
                if (!str.empty())
                {
                    vector<string> tokens;
                    string beginInterId, endInterId, sectionId;
                    int streetTrafficWalkerTypeNum;
                    vector<int> streetTrafficWalkerType;

                    AmuStringOperator::getTokens(&tokens, str, ',');
                    if (tokens.size() < 7
                        || static_cast<signed int>(tokens.size())
                        != 7 + atoi(tokens[6].c_str()))
                    {
                        cerr << "Error: section struct invalid token number "
                             << tokens.size() << endl;
                    }
                    else
                    {
                        // 1/2番目のカラムは交差点の識別番号
                        beginInterId
                            = AmuConverter::formatId
                            (tokens[0], NUM_FIGURE_FOR_INTERSECTION);
                        endInterId
                            = AmuConverter::formatId
                            (tokens[1], NUM_FIGURE_FOR_INTERSECTION);
                        if (atoi(beginInterId.c_str())
                            < atoi(endInterId.c_str()))
                        {
                            sectionId = beginInterId + endInterId;
                        }
                        else
                        {
                            sectionId = endInterId + beginInterId;
                        }
                        // 3番目のカラムはレーン幅
                        laneWidth          = atof(tokens[2].c_str());
                        // 4番目のカラムは路側幅
                        roadsideWidth      = atof(tokens[3].c_str());
                        // 5/6番目のカラムは横断歩道幅
                        sidewalkWidthRight = atof(tokens[4].c_str());
                        sidewalkWidthLeft  = atof(tokens[5].c_str());
                        // 7番目のカラムは車道通行歩行者種別数
                        streetTrafficWalkerTypeNum
                            = atoi(tokens[6].c_str());
                        // 8番目以降のカラムは車道通行歩行者種別
                        for (int i = 0; i < streetTrafficWalkerTypeNum; i++)
                        {
                            if (tokens[7 + i] == "*")
                            {
                                streetTrafficWalkerType.push_back
                                    (TRAFFIC_WALKER_TYPE_ANY);
                            }
                            else
                            {
                                streetTrafficWalkerType.push_back
                                    (atoi(tokens[7 + i].c_str()));
                            }
                        }
                        Section* section
                            = _currentRoadMap->section(sectionId);
                        if (!section)
                        {
                            cerr << "Error: section struct invalid input - "
                                 << "section " << sectionId
                                 << " does not exist." << endl;
                        }
                        else
                        {
                            if (!section->setStructInfo
                                (laneWidth, roadsideWidth,
                                 sidewalkWidthRight, sidewalkWidthLeft,
                                 streetTrafficWalkerType, false,
                                 autoSidewalkLane))
                            {
                                cerr << "Error: section struct invalid input of section "
                                     << sectionId << endl;
                            }
                        }
                    }
                }
            }
            inStructFile.close();
        }

        // 単路の構造情報に既定値を設定する、設定済みなら内部で無視する
        laneWidth     = GVManager::getNumeric("DEFAULT_LANE_WIDTH");
        roadsideWidth = GVManager::getNumeric("DEFAULT_ROADSIDE_WIDTH");
        sidewalkWidthRight = sidewalkWidthLeft
            = GVManager::getNumeric("DEFAULT_SIDEWALK_WIDTH");
        vector<int> streetTrafficWalkerTypeNone;
        CITRMAPS its = _currentRoadMap->sections()->begin();
        while (its != _currentRoadMap->sections()->end())
        {
            if (!(*its).second->setStructInfo
                (laneWidth, roadsideWidth,
                 sidewalkWidthRight, sidewalkWidthLeft,
                 streetTrafficWalkerTypeNone, true,
                 autoSidewalkLane))
            { 
                cerr << "section " << (*its).second->id()
                     << ": default struct info error." << endl;
            }
            its++;
        }
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::createSectionStructure()
{
    if (!_isSectionStructureCreated)
    {
        assert(_isSectionCreated);
        assert(_isSectionStructInfoSet);
   
        bool check = false;

        CITRMAPS its = _currentRoadMap->sections()->begin();
        while (its != _currentRoadMap->sections()->end())
        {
            // 各Sectionのcreate関数を呼び出す
            check = (*its).second->create();
            if (!check)
            {
                cerr << "section " << (*its).second->id()
                     << ": internal structure error." << endl;
            }
            its++;
        }
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::buildNetwork()
{
    assert(_isIntersectionCreated);
    assert(_isSectionCreated);

    // 具体的には交差点と単路のそれぞれに接続関係を伝える
    // Sectionのconnect()を呼び出す
    CITRMAPS its = _currentRoadMap->sections()->begin();
    while (its!=_currentRoadMap->sections()->end())
    {
        (*its).second->connect();
        its++;
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::buildSubnetwork()
{
    assert(_isIntersectionStructureCreated);
    assert(_isSectionStructureCreated);

    bool check = false;

    CITRMAPS its = _currentRoadMap->sections()->begin();
    while (its != _currentRoadMap->sections()->end())
    {
        check = (*its).second->createSubnetwork();
        if (!check)
        {
            cerr << "section " << (*its).second->id()
                 << ": create subnetwork error." << endl;
        }
        its++;
    }

    CITRMAPI iti = _currentRoadMap->intersections()->begin();
    while (iti != _currentRoadMap->intersections()->end())
    {
        check = (*iti).second->createSubnetwork();
        if (!check)
        {
            cerr << "intersection " << (*iti).second->id()
                 << ": create subnetwork error." << endl;
        }
        iti++;
    }

    return true;
}

//======================================================================
bool RoadMapBuilder::buildLaneConnection()
{
    assert(_isIntersectionStructureCreated);
    assert(_isSectionStructureCreated);

    const vector<LaneBundle*>* bundles = _currentRoadMap->laneBundles();
    for (unsigned int i=0; i<bundles->size(); i++)
    {
        const RMAPLAN* lanes = ((*bundles)[i])->lanes();
        for (CITRMAPLAN itl = lanes->begin();
             itl != lanes->end();
             itl++)
        {
            if (!((*itl).second->setConnection()))
            {
                cerr << "RoadMapBuilder::setLaneConnection(lane bundle:"
                     << (*bundles)[i]->id() << ")" << std::endl;
                return false;
            }
        }
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::buildSignals()
{
    assert(_currentRoadMap);
    CITRMAPI iti = _currentRoadMap->intersections()->begin();

    SignalIO& io = SignalIO::instance();
    vector<SignalAspect> aspect;
    SignalControlDataSet dataset;

    while (iti!=_currentRoadMap->intersections()->end())
    {
        if ((*iti).second->numNext()!=1
	    && (*iti).second->hasSignal())
        {
            // 接続数1の交差点（ODノード）には信号を設置しない
            Signal* signal = new Signal();
	    cout << (*iti).second->id() <<"の信号ができています" << endl;
            // 信号の設定ファイルを読み込む
            int sideNum = (*iti).second->numNext();
            aspect = io.aspect((*iti).first, sideNum);
            dataset = io.signalControlDataSet((*iti).first);

            // 信号に情報をセットする
            signal->setId((*iti).first);
            signal->setStateSet(aspect);
            signal->setSignalControlDataSet(dataset);

            // RoadMapと該当する交差点に登録する
            _currentRoadMap->addSignal(signal);
            (*iti).second->attachSignal(signal);

            // サブセクションに信号を登録する
            const RMAPENT* entities = (*iti).second->entities();
            CITRMAPENT ite;
            for (ite=entities->begin(); ite!=entities->end(); ite++)
            {
                if ((*ite).second->type()==SUBROAD_TYPE_CROSSWALK)
                {
                    int dir = (*iti).second->direction((*ite).second);
                    if (dir!=-1)
                    {
                        (*ite).second->attachSignal(signal, dir);
                    }
                }
            }
        }
        iti++;
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::buildSignalsAllBlue()
{
    assert(_currentRoadMap);
    CITRMAPI iti = _currentRoadMap->intersections()->begin();

    SignalIO& io = SignalIO::instance();
    vector<SignalAspect> aspect;
    SignalControlDataSet dataset;

    while (iti!=_currentRoadMap->intersections()->end())
    {
        if ((*iti).second->numNext()!=1)
        {
            // 接続数1の交差点（ODノード）には信号を設置しない
            Signal* signal = new Signal();

            // 信号現示パターンとスプリットを取得する
            int sideNum = (*iti).second->numNext();
            aspect = io.aspectBlue((*iti).first, sideNum);
            dataset = io.signalControlDataSetBlue((*iti).first);

            // 信号に情報をセットする
            signal->setId((*iti).first);
            signal->setStateSet(aspect);
            signal->setSignalControlDataSet(dataset);

            // RoadMapと該当する交差点に登録する
            _currentRoadMap->addSignal(signal);
            (*iti).second->attachSignal(signal);

            // サブセクションに信号を登録する
            const RMAPENT* entities = (*iti).second->entities();
            CITRMAPENT ite;
            for (ite=entities->begin(); ite!=entities->end(); ite++)
            {
                if ((*ite).second->type()==SUBROAD_TYPE_CROSSWALK)
                {
                    int dir = (*iti).second->direction((*ite).second);
                    if (dir!=-1)
                    {
                        (*ite).second->attachSignal(signal, dir);
                    }
                }
            }
        }
        iti++;
    }
    return true;
}

//======================================================================
bool RoadMapBuilder::setSpeedLimit()
{
    assert(_isSectionCreated);

    string fSpeedLimit;

    GVManager::getVariable("SPEED_LIMIT_FILE", &fSpeedLimit);

    ifstream ifs(fSpeedLimit.c_str(), ios::in);
    if (!ifs)
    {
        cout << "no speed limit file: " << fSpeedLimit << endl;
        return false;
    }

    string str;
    while (ifs.good())
    {
        getline(ifs, str);
        AmuStringOperator::getAdjustString(&str);
        if (!str.empty())
        {
            vector<string> tokens;
            string beginInterId;
            string endInterId;
            double limit;

            AmuStringOperator::getTokens(&tokens, str, ',');
            assert(tokens.size()==3);
            beginInterId = tokens[0];
            endInterId = tokens[1];
            limit = atof(tokens[2].c_str());

            // 指定された単路が本当に存在するかチェックする
            string sectionId;
            bool isUpDirection;
            if (beginInterId < endInterId)
            {
                sectionId
                    = AmuConverter::formatId
                    (beginInterId, NUM_FIGURE_FOR_INTERSECTION)
                    + AmuConverter::formatId
                    (endInterId,   NUM_FIGURE_FOR_INTERSECTION);
                isUpDirection = true;
            }
            else
            {
                sectionId
                    = AmuConverter::formatId
                    (endInterId,   NUM_FIGURE_FOR_INTERSECTION)
                    + AmuConverter::formatId
                    (beginInterId, NUM_FIGURE_FOR_INTERSECTION);
                isUpDirection = false;
            }
            Section* section = _currentRoadMap->section(sectionId);
            if (!section)
            {
                cerr << "speed limit: invalid input - "
                     << "section " << sectionId << " does not exist."
                     << endl;
            }
            else
            {
                // 値をセットする
                const RMAPLAN* lanes = section->lanes();
                CITRMAPLAN itl;

                for (itl=lanes->begin(); itl!=lanes->end(); itl++)
                {
                    if (section->isUp((*itl).second)==isUpDirection)
                    {
                        (*itl).second->setSpeedLimit(limit);
                    }
                }
            }
        }
    }
    ifs.close();
    return true;
}

//======================================================================
RoadMap* RoadMapBuilder::roadMap()
{
    RoadMap* tmp = _currentRoadMap;
    _currentRoadMap = 0;
    return tmp;
}
