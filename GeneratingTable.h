#ifndef __GENERATINGTABLE_H__
#define __GENERATINGTABLE_H__

#include <string>
#include <vector>
#include "TimeManager.h"
#include "OD.h"

/// ある期間における車両の発生データを格納するクラス
/** 
 * GTCell means Generating Table Cell.
 *
 * GeneratingTable で利用される．
 *
 * volume() はODノード(厳密にはODノードに接続する単路)あたりの
 * 発生率(交通量)を[veh./hr]の形で返す
 *
 * @sa GeneratingTable
 * @ingroup IO
 */
class GTCell
{
public:
    GTCell();
    ~GTCell();

    /// 内部パラメータ値を一括して設定する
    bool setValue(ulint begin, ulint end, int volume,
                  const std::string& start, const std::string& goal);

    /// 内部パラメータ値を一括して設定する (経由地，車種ID含む)
    bool setValue(ulint begin, ulint end, int volume, int vehicleType,
                  const std::string& start, const std::string& goal,
                  const std::vector<std::string>& stopPoints);

    /// 開始時刻を返す
    ulint begin() const;

    /// 終了時刻を返す
    ulint end() const;

    /// 発生交通量 [veh./hour]を返す
    int volume() const;

    /// 大型車混入率を返す
    int vehicleType() const;

    /// 出発地の交差点の識別番号を返す
    const std::string& start() const;

    /// 目的地の交差点の識別番号を返す
    const std::string& goal() const;

    /// 経由地の交差点の識別番号を返す
    const std::vector<std::string>* stopPoints() const;

    /// 出発地、目的地、経由地を格納したODを返す
    const OD* od() const;

    /// 情報を表示する
    void print() const;

protected:
    ulint _begin;        //!< 開始時刻
    ulint _end;          //!< 終了時刻
    int _volume;         //!< 交通量[veh./hour]
    int _vehicleType;    //!< 車種ID
    OD _od;              //!< 出発地, 目的地, 経由地
};

/// 車両発生データ処理クラス
/**
 * GeneratingTableクラスはinitでファイルを読みこんで、
 * setTimeを設定することによってpresentCellsを構築し、
 * あるタイムステップでのある交差点における1時間あたりの発生交通量を返す．
 *
 * @ingroup IO
 */
class GeneratingTable
{
public:
    GeneratingTable();
    ~GeneratingTable();

    /// ファイルを読みこんでメモリにテーブルを構築する
    bool init(const std::string& fileName);

    /// 外部からGTCellを作成する
    void createGTCell(ulint begin, ulint end, int volume,
                      const std::string& start, const std::string& goal);

    /// 外部からGTCellを作成する
    void createGTCell(ulint begin, ulint end, int volume, int vehicleType,
                      const std::string& start, const std::string& goal,
                      const std::vector<std::string>& stopPoints);

    /// 現在時刻に対して効力を持つCellの個数を返す
    /**
     * 一つのODノードに対して同時に複数のCellを設定できるように
     * 返り値をboolからintに変更した
     */
    int getTiming(const std::string& intersectionId, 
                  std::vector<std::string>* result_goals,
                  std::vector<int>* result_volumes) const;

    /// 現在時刻に対して効力を持つCellを取得する
    void getActiveGTCells(const std::string& intersectionId, 
                          std::vector<const GTCell*>* result_GTCells) const;

    /// ID@p intersectionIdで指定したODノードに関するCellを取得する
    void getValidGTCells(const std::string& intersectionId,
                         std::vector<const GTCell*>* result_GTCells) const;

    /// ID@p intersectionIdで指定したODノードに関するCellを取得する
    const GTCell* validGTCell(const std::string& intersectionId) const;

    /// 現在時刻を設定する
    void setTime(ulint time);

    /// 現在時刻を返す
    ulint time() const;

protected:
    std::vector<GTCell> _table;
    ulint _presentTime;
    std::vector<GTCell> _presentCells;
};

#endif // __GENERATINGTABLE_H__
