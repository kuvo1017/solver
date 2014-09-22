#ifndef __CONF_H__
#define __CONF_H__

/** @name 円周率の定義 */
//@{
#ifndef M_PI
#define M_PI           3.14159265358979323846  //< pi
#endif
#ifndef M_PI_2
#define M_PI_2         1.57079632679489661923  //< pi/2
#endif
#ifndef M_PI_4
#define M_PI_4         0.78539816339744830962  //< pi/4
#endif
#ifndef M_1_PI
#define M_1_PI         0.31830988618379067154  //< 1/pi
#endif
#ifndef M_2_PI
#define M_2_PI         0.63661977236758134308  //< 2/pi
#endif
//@}

/** @name 識別番号の桁数の設定 */
//@{

/// 交差点
#define NUM_FIGURE_FOR_INTERSECTION     6

/// 単路識別番号
#define NUM_FIGURE_FOR_SECTION          (NUM_FIGURE_FOR_INTERSECTION * 2)

/// サブセクション
#define NUM_FIGURE_FOR_SUBSECTION       2

/// レーン
#define NUM_FIGURE_FOR_LANE             8

/// コネクタのグローバル識別番号表示時の桁数
/**
 * @note
 * 実際の識別番号は生成されるたびに，
 * (コネクタ総数+1)で与えられる
 */
#define NUM_FIGURE_FOR_CONNECTOR_GLOBAL 2

/// コネクタのローカル識別番号
#define NUM_FIGURE_FOR_CONNECTOR_LOCAL  4

/// 車両
#define NUM_FIGURE_FOR_VEHICLE          6

/// 車両
#define NUM_FIGURE_FOR_DETECTORUNIT     4

/// 時刻
#define NUM_FIGURE_FOR_DRAW_TIME        6

//@}

/** @name 道路ネットワーク */
//@{

/// 横のレーンを探索するときの線分の長さ[m]
#define SEARCH_SIDE_LANE_LINE_LENGTH 5

//@}

/** @name 信号制御 */
//@{

/// スプリットの最大数
#define NUM_MAX_SPLIT 20 

//@}

/** @name エージェントに関する定数 */
//@{

/// 経路探索パラメータの数
#define VEHICLE_ROUTING_PARAMETER_SIZE  6

/// 旅行時間算出用の車両数
#define VEHICLE_PASS_TIME_INTERSECTION 10

//@}

#endif //__CONF_H__
