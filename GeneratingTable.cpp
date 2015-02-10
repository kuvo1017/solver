#include "GeneratingTable.h"
#include "Conf.h"
#include "Router.h"
#include "AmuStringOperator.h"
#include "GVManager.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib>

using namespace std;

//######################################################################
// GTCellクラス
//======================================================================
GTCell::GTCell():_begin(0),_end(0), _volume(0){}

//======================================================================
GTCell::~GTCell(){}

//======================================================================
bool GTCell::setValue(ulint begin, ulint end, int volume,
    const std::string& start, const std::string& goal)
{
  if(begin <= end && volume >= 0)
  {
    _begin = begin;
    _end = end;
    _volume = volume;
    _vehicleType = 20;
  }
  else
  {
    _begin = 0;
    _end = 0;
    _od.clear();
    return false;
  }

  if(!_od.setValue(start, goal))
  {
    return false;
  }

  return true;
}

//======================================================================
bool GTCell::setValue(ulint begin, ulint end, int volume, int vehicleType,
    const std::string& start, const std::string& goal,
    const std::vector<std::string>& stopPoints)
{
  if (begin <= end && volume >= 0)
  {
    _begin = begin;
    _end = end;
    _volume = volume;
    _vehicleType = vehicleType;
  }
  else
  {
    _begin = 0;
    _end = 0;
    _od.clear();
    return false;
  }

  if(!_od.setValue(start, goal, stopPoints))
  {
    return false;
  }

  return true;
}

//======================================================================
ulint GTCell::begin() const
{
  return _begin;
}

//======================================================================
ulint GTCell::end() const
{
  return _end;
}

//======================================================================
int GTCell::volume() const
{
  return _volume;
}

//======================================================================
int GTCell::vehicleType() const
{
  return _vehicleType;
}

//======================================================================
const string& GTCell::start() const
{
  return _od.start();
}

//======================================================================
const string& GTCell::goal() const
{
  return _od.goal();
}

//======================================================================
const vector<string>* GTCell::stopPoints() const
{
  return _od.stopPoints();
}

//======================================================================
const OD* GTCell::od() const
{
  return &_od;
}

//======================================================================
void GTCell::print() const
{
  cout << "beginTime: " << _begin
    << ", endTime: " << _end
    << ", volume: " << _volume
    << ", origin: " << _od.start()
    << ", destination: " << _od.goal()
    << ", stopPoints: ";

  const vector<string>* stopPoints = _od.stopPoints();
  if (stopPoints->empty())
  {
    cout << "none." << endl;
    return;
  }
  for (unsigned int i=0; i<stopPoints->size(); i++)
  {
    cout << (*stopPoints)[i] << " ";
  }
  cout << endl;
}

//######################################################################
// GeneratingTableクラス
//======================================================================
GeneratingTable::GeneratingTable() :_presentTime(0){}

//======================================================================
GeneratingTable::~GeneratingTable()
{
  _table.clear();
  _presentCells.clear();
}

//======================================================================
void GeneratingTable::setTime(ulint time)
{
  _presentTime = time;
  _presentCells.clear();
  // tableから現在のtimeに関係する要素のみを抜きだして、_presentCellsにコピー
  vector<GTCell>::const_iterator where;
  for(where=_table.begin(); where!=_table.end(); where++)
  {
    if((*where).begin() <= _presentTime
        && (*where).end() > _presentTime)
    {
      _presentCells.push_back(*where);
    }
  }
}

//======================================================================
ulint GeneratingTable::time() const
{
  return _presentTime;
}

//======================================================================
void GeneratingTable::createGTCell(
    ulint begin, ulint end, int volume,
    const std::string& start, const std::string& goal)
{
  GTCell cell;
  cell.setValue(begin, end, volume, start, goal);
  _table.push_back(cell);
}

//======================================================================
void GeneratingTable::createGTCell(
    ulint begin, ulint end, int volume, int vehicleType,
    const std::string& start, const std::string& goal,
    const std::vector<std::string>& stopPoints)
{
  GTCell cell;
  cell.setValue(begin, end, vehicleType,
      volume, start, goal, stopPoints);
  _table.push_back(cell);
}

//======================================================================
int GeneratingTable::getTiming(const std::string& intersectionId,
    std::vector<std::string>* result_goals,
    std::vector<int>* result_volumes) const
{
  int result=0;
  //引数の初期化  
  (*result_goals).clear();
  (*result_volumes).clear();

  vector<GTCell>::const_iterator where;
  for(where=_presentCells.begin(); where!=_presentCells.end(); where++)
  {
    if((*where).start().compare(intersectionId)==0)
    {
      (*result_goals).push_back((*where).goal());
      (*result_volumes).push_back((*where).volume());
      result++;
    }
  }
  return result;
}

//======================================================================
void GeneratingTable::getActiveGTCells(
    const std::string& intersectionId,
    std::vector<const GTCell*>* result_GTCells) const
{
  //引数の初期化  
  (*result_GTCells).clear();

  vector<GTCell>::const_iterator where;
  for(where=_presentCells.begin(); where!=_presentCells.end(); where++)
  {
    if((*where).start().compare(intersectionId)==0)
    {
      (*result_GTCells).push_back(&(*where));
    }
  }
}

//======================================================================
void GeneratingTable::getValidGTCells(
    const std::string& intersectionId,
    std::vector<const GTCell*>* result_GTCells) const
{
  for(vector<GTCell>::const_iterator where=_table.begin();
      where!=_table.end();
      where++)
  {
    if((*where).start().compare(intersectionId)==0)
    {
      (*result_GTCells).push_back(&(*where));
    }
  }
}

//======================================================================
const GTCell* GeneratingTable::validGTCell(
    const std::string& intersectionId) const
{
  for(vector<GTCell>::const_iterator where=_table.begin();
      where!=_table.end();
      where++)
  {
    if((*where).start().compare(intersectionId)==0)
    {
      return &(*where);
    }
  }
  return NULL;
}

//======================================================================
bool GeneratingTable::init(const std::string& fileName)
{
  bool result = false;
  fstream fin;
  fin.open(fileName.c_str(), ios::in);
  if(!fin.good())
  {
    cout << "no vehicle generate table file: "
      << fileName << endl;
    return false;
  }
  else
  {
    result = true;
  }
  vector<string> tokens;
  string str;
  while(fin.good())
  {
    getline(fin, str);
    //文字列の整形
    AmuStringOperator::getAdjustString(&str);
    if(!str.empty())
    {
      AmuStringOperator::getTokens(&tokens, str, ',');
    }
    if(tokens.size() >= 6)
    {
      int i;
      int curIndex;
      GTCell cell;
      ulint begin;
      ulint end;
      int volume;
      int vehicleType;
      string start;
      string goal;
      int numStoppingInters;
      vector<string> stopPoints;
      vector<string>::iterator it;

      // 発生開始、終了時刻
      begin = atoi(tokens[0].c_str());
#ifdef JSON
      end = GVManager::getNumeric("MAX_TIME");
#else
      end = atoi(tokens[1].c_str());
#endif

      // 出発地、目的地
      ostringstream ost0, ost1;
      ost0.width(NUM_FIGURE_FOR_INTERSECTION);ost0.fill('0');
      ost1.width(NUM_FIGURE_FOR_INTERSECTION);ost1.fill('0');
      ost0 << (tokens[2]);
      ost1 << (tokens[3]);
      start = ost0.str();
      goal = ost1.str();

      // 発生量
      volume = atoi(tokens[4].c_str());

      // 車種ID
      vehicleType = atoi(tokens[5].c_str());
#ifdef JSON
      if(volume > 0)
      {
      if(vehicleType >=20 && vehicleType < 30)
      {
	/*
        cout << "small!!" << endl;
        volume = GVManager::getNumeric("SMALL_TRAFFIC_VOLUME");
         cout  << "volume is " << volume <<endl;
	 */
      }else if(vehicleType >=50 && vehicleType < 60) 
      {
        volume = GVManager::getNumeric("SMALL_TRAFFIC_VOLUME")*0.3;
      }
      }
#endif
      // 経由地
      curIndex = 6;
      numStoppingInters = atoi(tokens[curIndex].c_str());
      curIndex++;
      stopPoints.reserve(numStoppingInters);
      for(i = 0; i < numStoppingInters; i++)
      {
        ostringstream ost;
        string stopPoint;
        ost.width(NUM_FIGURE_FOR_INTERSECTION);ost.fill('0');
        ost << (tokens[curIndex]);
        curIndex++;
        stopPoint = ost.str();
        stopPoints.push_back(stopPoint);
      }

      if(cell.setValue(begin, end, volume, vehicleType, 
            start, goal, stopPoints))
      {
        _table.push_back(cell);
      }
      else
      {
        cerr << "Unknown error occured in GeneratingTable.\n"
          << "begin, end, traffic volume, start, goal\n"
          << begin << "\t" << end << "\t" << volume
          << "\t" << start << "\t" << goal << endl;
      }
      tokens.clear();
    }
  }
  fin.close();
  return result;
}
