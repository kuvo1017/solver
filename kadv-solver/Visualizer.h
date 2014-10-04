#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

#include <string>

class Simulator;

/**
 * @addtogroup Visualization
 * @brief シミュレーションの可視化
 * @ingroup Procedure
 */

/// 可視化クラス
/**
 * AutoGLを用いた可視化とGUI
 *
 * @ingroup Visualization
 */
class Visualizer
{
public:
    Visualizer();
    ~Visualizer(){};

    /// シミュレータをセットする
    void setSimulator(Simulator* simulator);

    /// 可視化を開始する
    static void visualize();

    //******************************************************************
    /** @name エージェント等の描画 */
    //@{

    /// 路側器を描画する
    static void drawRoadsideUnits();

    /// 自動車を描画する
    static void drawVehicles();

    //@}

protected:
    //******************************************************************
    /** @name コールバックに関する関数 */
    //@{

    /// シミュレータを1ステップ進める
    static void timeIncrement();

    /// _poseTimeまでシミュレータを動かす
    static void run();

    /// 車両をマニュアルで発生させる
    static void generateVehicleManual();

    /// 指定したIDの車両を探す
    static void searchVehicle();

    /// 指定したIDの車両の情報を表示する
    static void showVehicleInfo();

    /// 指定したIDの交差点を探す
    static void searchIntersection();

    /// 指定したIDの交差点の情報を表示する
    static void showIntersectionInfo();

    /// ビューを再描画する
    static void viewRedrawCallback();

    /// Drawボタンが押されたときの動作
    static void drawButtonCallback();

    /// ビューの制御変数を出力する
    static void printViewingParamsCallback();

    /// ビューの制御変数を設定する
    static void setViewingParamsCallback();

    /// Quitボタンが押されたときの動作
    static void quitButtonCallback();

    /// Resetボタンが押されたときの動作
    static void resetButtonCallback();

    /// AutoIncrementボタンが押されたときの動作
    static void autoIncrementButtonCallback();

    /// SkipRunボタンが押されたときの動作
    static void skipRunButtonCallback();

    //@}

protected:
    /// 可視化の対象となるSimulator
    static Simulator* _sim;

    static int _poseTime;          //!< runの目標時刻
    static int _idleEventIsOn;     //!< アイドルイベントが有効か
    static int _withCapturing;     //!< スクリーンキャプチャを保存するか
    static int _outputTimeline;    //!< 時系列データを出力するか
    static int _outputInstrumentD; //!< 計測機器の詳細データを出力するか
    static int _outputInstrumentS; //!< 計測機器の統計データを出力するか
    static int _outputGenCounter;  //!< エージェント発生データを出力するか
    static int _outputTripInfo;    //!< エージェントのトリップ情報を出力するか

    static int _timeUnitRate;      //!< 1ステップあたりの時間[msec]
    static int _viewingTimeRate;   //!< 描画1回あたりの時間[msec]
    /*
     * 例えば、_viewingTimeRateに_timeUnitRateの2倍の値を設定すると
     * 2ステップに1回の描画となる
     */
    static int _skipRunTimeRate;   //!< 描画をスキップして実行する時間[msec]
    static int _frameNumber;       //!< 静止画出力する際のファイル番号 

    /** @name ビューの視線位置 */
    //@{
    static double _viewPositionXs;
    static double _viewPositionYs;
    static double _viewPositionZs;
    //@}

    /** @name ビューの視線方向 */
    //@{
    static double _viewDirectionXs;
    static double _viewDirectionYs;
    static double _viewDirectionZs;
    //@}

    /** @name ビューの制御変数設定用変数 */
    //@{
    static double _svpSize;
    static double _svpPositionXs;
    static double _svpPositionYs;
    static double _svpPositionZs;
    static double _svpDirectionXs;
    static double _svpDirectionYs;
    static double _svpDirectionZs;
    //@}

    /** @name 道路の表示パラメータ */
    //@{
    static int _isSectionId;       //!< 交差点や単路の識別番号を表示するか
    static int _isLaneId;          //!< 内部レーンの識別番号を表示するか
    static int _isLanesInter;      //!< 交差点の内部レーンを表示するか
    static int _isLanesSection;    //!< 単路の内部レーンを表示するか
    static int _isSimpleMap;       //!< 簡略化された出力を表示するか 
    static int _isSubsectionShape; //!< サブセクションを表示するか
    static int _isSubnetwork;      //!< サブネットワークを表示するか
    static int _isSubsectionId;    //!< サブセクションの識別番号を表示するか
    static int _surfaceMode;       //!< 路面描画モード(0:描画属性，1:通行権)
    static int _isSignals;         //!< 信号を表示するか
    static int _isRoadsideUnits;   //!< 路側器を表示するか
    static int _connectorIdMode;   //!< コネクタID描画モード(0:Disable，1:Global，2:Local)
    //@}

    /** @name エージェントの表示パラメータ */
    //@{
    static int _isVehicleId;       //!< 車両の識別番号を表示するか
    static int _vehicleColorMode;  //!< 車両色モード(0:VehicleFamily，1:AverageVelocity)

    static char _infoVehicleId[16];      //!< 出力する車両のID
    static char _infoIntersectionId[16]; //!< 出力する交差点のID
    //@}

    /** @name エージェント発生用のパラメータ */
    //@{
    static int _genVehicleType;       //!< 車種ID
    static char _genVehicleStart[16]; //!< 出発地ID
    static char _genVehicleGoal[16];  //!< 目的地ID
    static char _genVehicleParams[256];     //!< 経路選択用パラメータ
    static char _genVehicleStopPoints[512]; //!< 経由地
    //@}
};

#endif //__VISUALIZER_H__
