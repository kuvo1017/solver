#include "GenerateVehicleIO.h"
#include "TimeManager.h"
#include "GVManager.h"
#include "RoadMap.h"
#include "Intersection.h"
#include "Section.h"
#include "Lane.h"
#include "Vehicle.h"
#include "ARouter.h"
#include "AmuConverter.h"
#include "AmuStringOperator.h"
#include "Conf.h"
#include <vector>
#include <string>
#include <cassert>
#include <fstream>

using namespace std;

//======================================================================
void GenerateVehicleIO::getReadyCounters(RoadMap* roadMap)
{
    assert(roadMap);

    string fGenCounter;
    GVManager::getVariable("GENCOUNTER_FILE", &fGenCounter);

    ifstream inGenCounterFile(fGenCounter.c_str(), ios::in);
    if (!inGenCounterFile.good())
    {
        cout << "no generate counter file: " 
             << fGenCounter << endl;
        return;
    }
  
    // ファイルの読み込み
    string str;
    vector<ODNode*> counterNodes;
    while (inGenCounterFile.good())
    {
        getline(inGenCounterFile, str);
        AmuStringOperator::getAdjustString(&str);
        if (!str.empty())
        {
            string id
                = AmuConverter::formatId(str,
                                         NUM_FIGURE_FOR_INTERSECTION);
            ODNode* node
                = dynamic_cast<ODNode*>(roadMap->intersection(id));
            if (node)
            {
                counterNodes.push_back(node);
            }
            else
            {
                cerr << "generate counter: invarid input"
                     << " - OD node " << id 
                     << " does not exist." << endl;
            }
        }
    }
    inGenCounterFile.close();

    string outputDir, prefix;
    GVManager::getVariable("RESULT_INSTRUMENT_DIRECTORY", &outputDir);
    GVManager::getVariable("RESULT_GENCOUNTER_PREFIX", &prefix);
    if (!outputDir.empty())
    {
        for (unsigned int i=0; i<counterNodes.size(); i++)
        {
            // 出力ファイルを準備する
            string fGVCounter
                = outputDir + prefix + counterNodes[i]->id() + ".txt";
            ofstream outGVCounterFile(fGVCounter.c_str(), ios::out);
            if (!outGVCounterFile.fail())
            {
                outGVCounterFile
                    << "#incident section:"
                    << counterNodes[i]->nextSection(0)->id()
                    << ", lanes:"
                    << (counterNodes[i]->nextSection(0)
                        ->lanesFrom(counterNodes[i])).size()
                    << endl
                    << "#time, lane, vehicle_id, vehicle_type,"
                    << " origin, destination, time-headway"
                    << endl;
            }
            outGVCounterFile.close();
        }
    }
    if (GVManager::getFlag("FLAG_VERBOSE"))
    {
        print(&counterNodes);
    }
}

//======================================================================
void GenerateVehicleIO::writeGeneratedVehicleData
(ODNode* node, vector<ODNode::GeneratedVehicleData>* gvd)
{
    /*
     * 1ノードにつき1ファイル作成する．出力形式は
     *   time, laneID, vehicleID, origin, destination, time-headway
     * とする．ここを変更したらgetReadyCounterの出力も変更．
     * 09/10/20 VehicleTypeを出力するよう変更 by H.Fujii
     */
    assert(node);

    string outputDir, prefix;
    GVManager::getVariable("RESULT_INSTRUMENT_DIRECTORY", &outputDir);
    GVManager::getVariable("RESULT_GENCOUNTER_PREFIX", &prefix);
    string fGVCounter = outputDir + prefix + node->id() + ".txt";

    if (!outputDir.empty())
    {
        ofstream outGVCounterFile(fGVCounter.c_str(), ios::app);
        if (!outGVCounterFile.fail())
        {
            for (unsigned int i=0; i<gvd->size(); i++)
            {
                outGVCounterFile
                    << AmuConverter::itos
                    (TimeManager::time(), 9) << ","
                    << (*gvd)[i].lane->id() << ","
                    << (*gvd)[i].vehicle->id() << ","
                    << (*gvd)[i].vehicle->type() << ","
                    << (*gvd)[i].vehicle->router()->start()->id() << ","
                    << (*gvd)[i].vehicle->router()->goal()->id() << "," 
                    << (*gvd)[i].headway << endl;
            }
        }
        outGVCounterFile.close();
    }
}

//======================================================================
void GenerateVehicleIO::print(vector<ODNode*>* nodes)
{
    cout << "*** Generate Counter Information ***" << endl;
    for (int i=0; i<static_cast<signed int>(nodes->size()); i++)
    {
        cout << (*nodes)[i]->id() << endl;
    }
    cout << endl;
}
