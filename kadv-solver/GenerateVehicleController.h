#ifndef __GENERATE_VEHICLE_CONTROLLER_H__
#define __GENERATE_VEHICLE_CONTROLLER_H__

#include "GeneratingTable.h"
#include "VehicleFamily.h"
#include <map>
#include <string>
#include <vector>

class RoadMap;
class Intersection;
class ODNode;
class Section;
class Vehicle;

/// 車両の発生を制御するクラス
/**
 * @ingroup Running
 */
class GenerateVehicleController
{
public:
    /// 唯一のインスタンスを返す
    static GenerateVehicleController& instance();
    
    /// RoadMapをセットする
    void setRoadMap(RoadMap* roadMap);

    /// 車両発生に関する初期設定を行う
    /**
     * @return 設定に成功したかどうか
     */
    bool getReadyGeneration();

    //======================================================================
    /** @name エージェントの発生 */
    //@{

    /// 車両を発生させる
    void generateVehicle();

    /// 手動で車両を発生させる
    void generateVehicleManual(const std::string& startId,
                               const std::string& goalId,
                               std::vector<std::string> stopPoints,
                               VehicleType vehicleType,
                               std::vector<double> params);

    /// サンプルシナリオにおいて車両の初期配置を行う
    void generateSampleVehicles(unsigned int numVehicles,
                                double headway);
    //@}

private:
    GenerateVehicleController();
    ~GenerateVehicleController();

    //======================================================================
    /** @name エージェントの発生に用いるprivate関数 */
    //@{

    /// ランダムテーブルを設定する
    void _createRandomTable();

    /// 最初の車両発生時刻を決定する
    /**
     * Generate Table，Default Generate Tableが持つ各GTCellおよび
     * 発生交通量がファイルで指定されていない各交差点に対して
     * 最初の車両発生時刻を決定する．以降，車両発生時に次の車両発生時刻を決定する
     */
    void _determineFirstGenerationTime();

    /// 次の車両発生時刻を設定する
    /**
     * 車両の発生はポワソン分布を仮定する．
     * すなわち，単位時間あたりの車両発生台数がポワソン分布に従う．
     * このとき，車両発生の時間間隔は指数分布となる．
     *
     * @param startTime 時間間隔の基準となる時刻
     * @param cell 処理するGTCell
     * @return 有効な時刻を設定できたか
     */
    bool _addNextGenerateTime(ulint startTime, const GTCell* cell);

    /// 確率的に車両を発生させる
    void _generateVehicleRandom();

    /// 確率的に車両を発生させる
    void _generateVehicleRandom2();

    /// 車両を生成し，経路選択する
    /**
     * 乱数によって車両生成フラグが立った後の処理．
     * ObjManager::createVehicle()で車両を生成し，経路選択．
     */
    Vehicle* _createVehicle(ODNode* start,
                            ODNode* goal,
                            Intersection* past,
                            Section* section,
                            OD* od,
                            VehicleType vehicleType);

    /// 車両を生成し，経路選択する
    /**
     * 引数として経路選択パラメータを与える．
     */
    Vehicle* _createVehicle(ODNode* start,
                            ODNode* goal,
                            Intersection* past,
                            Section* section,
                            OD* od,
                            VehicleType vehicleType,
                            std::vector<double> params);

    /// 車両の属性を設定
    void _setVehicleStatus(Vehicle* vehicle);

    /// ゴールをランダムに決定する
    ODNode* _decideGoalRandomly(ODNode* start);

    //@}
    
    /// ODノードのスタートレベルを返す
    /**
     * 単路への流入点でstartLevelを決定する
     * "単路の"流入点・流出点は"ODノードの"流入点・流出点と反対
     */
    int _odNodeStartLevel(ODNode* node) const;

    /// ODノードのゴールレベルを返す
    /**
     * 単路からの流出点でgoalLevelを決定する
     * "単路の"流入点・流出点は"ODノードの"流入点・流出点と反対
     */
    int _odNodeGoalLevel(ODNode* node) const;

    /// Router用パラメータの設定
    void _readRouteParameter();

private:
    /// 地図オブジェクト
    RoadMap* _roadMap;

    /// 車両発生イベント管理キュー
    /**
     * mapのインデックスは車両発生時刻，値は該当するGTCell
     */
    std::multimap<unsigned long, const GTCell*> _generateVehicleQueue;

    /// 車両発生を待つODノード
    std::vector<ODNode*> _waitingODNodes;

    /** @name 車両発生定義テーブル */
    //@{
    GeneratingTable _table;        //!< 始点終点の双方を設定
    GeneratingTable _defaultTable; //!< 始点のみ設定
    GeneratingTable _randomTable;  //!< 無設定（デフォルト交通量）
    //@}

    /// レベル分けされたOriginノード
    std::vector<ODNode*> _startLevel[3];

    /// レベル分けされたDestinationノード
    std::vector<ODNode*> _goalLevel[3];

    /// デフォルトのレベル別交通量
    int _defaultTrafficVolume[3];

    /// Router用パラメータセット
    std::vector<std::vector<double> > _vehicleRoutingParams;
};

#endif //__GENERATE_VEHICLE_CONTROLLER_H__
