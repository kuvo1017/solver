#ifndef __ERROR_CONTROLLER_H__
#define __ERROR_CONTROLLER_H__

#include <iostream>
//#include "Vehicle.h"
#include "VirtualLeader.h"
#include "picojson.h"
using namespace std;

class Vehicle;
class Lane;
class Intersection;

class ErrorController{ 
  public:
    ErrorController();
    ///
    ErrorController(Vehicle* vehicle);

  //======================================================================
  /**
   * @name 追突に関する関数
   *
   * @note VehicleAccelDeterminer.cppで定義されている
   */
  //@{
    /// 追突事故の処理
    VirtualLeader* rearError(VirtualLeader* resultLeader); 
  protected:
    /// 見ない認知エラーが発生するための関数
    // 認知対象物が多いほど、この値が大きくなる
    double _objectPoint();
 
    /// 予測エラー時の前方車との間隔
    double _rearErrorLength;

    ///　予測エラー時の前方車の速度
    double _rearErrorVelocity;

    ///　予測エラー継続時間
    int _rearErrorTime;

                       ///  先行者のid
    string _rearId;    
    //@}

   //======================================================================
  /**
   * @name 出会い頭に関する関数
   *
   * @note VehicleAccelDeterminer.cppで定義されている
   */
  //@{
    /// 障害物で認知できていない車両の集合を返す
    void passingError();
     /// 障害物で認知できていない車両の集合を返す
    const std::vector<Vehicle*>* invisibleVehicles() const;
    /// 認知できていない車両でを追加
    void setInvisibleVehicle(Vehicle* vehicle);
    // 認知できてない車両を消去
    void resetInvisibleVehicles();

     /// 認知できていない車両の配列
    std::vector<Vehicle*> _invisibleVehicles;
 
  protected:

    /// 予測エラー時の前方車との間隔
    double _rearErrorLength;

    ///　予測エラー時の前方車の速度
    double _rearErrorVelocity;

    ///　予測エラー継続時間
    int _rearErrorTime;

    //@}

    //======================================================================
  /**
   * @name 右左折に関する関数
   */
  //@{

     /// 傲慢なエージェントであるかどうかを返す関数
     bool isArroganceLR();

    /// 右左折事故の処理
    void LRError(Vehicle* thatV,double thisTti,double thatTti) ; 
    void LRError(double thisTti,double thatTtp) ;  
  protected:


    //@}
    ///======================================================================
  /**
   * @name 進路変更事故に関する関数
   */
  //@{


    /// 進路変更エラーが起こるかどうかを計算する関数
    bool shiftError();

    /// 進路変更エラーを終わらせる
    void endShiftError();     
  protected:
              /// 一度車線変更してから立った時間
    int _shiftTime;
 

    //@}
    /// 
    // そのステップで発生するエラーを決める
    void checkError();
    /// 追突エラー状態かどうか↲
    bool isRearError() const;
    /// 右左折エラー状態かどうか↲
    bool isLRError() const;
    /// 出合い頭エラー状態かどうか↲
    bool isPassingError() const;
    /// 進路変更エラーかどうか↲
    bool isShiftError() const;
    /// 正面衝突エラーかどうか↲
    bool isHeadError() const;

    ///　事故状態かどうか↲
    bool isAccident() const;
   ///　事故時間↲
    int accidentTime() const;
    /// 事故が起きたときに外部ファイルに記入する
    void writeAccident();
    /// エラーが起きたときに外部ファイルに記入する
    void writeError();
    /// 事故が起こった時の処理
    void accidentOccur(std::string collidType);

    /// 横ずれエラーが起きている時間
    int headErrorTime();
    /// 
    void errorInIntersection(Intersection* next);

    /// 正面衝突エラーが起きるかを計算
    bool headError();
    /// 正面衝突事故が起きている時に横ずれの速度を返す
    double errorVelocity();
    /// 事故状態かどうかをチェックして、事故状態ならエージェント消去
    bool accidentCheck();
    /// エラーの種類を返す
    string errorType() const;

    /// OACIS用にファイルパスを返す
    static std::string setDataPath();
    /// 各エラー率の初期化
    static bool initErrorParams();
    /// シミュレーションの終了時間をJSONから読み込む
    static bool stopRun();
    /// 
    static void checkStatData(); 
    /// 
  static void writeStatData(int totalP,int totalT,string time);
  ///
  static void endRun();
  protected: 
  /// 車両オブジェクト
  Vehicle* _vehicle;
  int _errortime;
  /// 予測エラーか
  bool _isRearError;
  /// error ga 
  void _errorEnd();

  /// 横ずれエラー時間
  int _headErrorTime;
     ///　障害物エラーか
    int _isPassingError; 
    ///　傲慢なエージェントか
  bool _isArroganceLR;
  ///  左折エラーかどうか
  bool _isLRError;
  /// 進路変更エラーかどうか
  bool _isShiftError;
  /// 横にずれるエラーかどうか
    bool _isHeadError;
   /// 遅い車か
    bool _isSlow;
    ///  それぞれの制限速度
    double _vIde;
    ///  前の車との速度差
    double _velocityDifference;

   /// 正面衝突事故が起きてるかどうかをチェック
    bool _checkHeadAccident();
     /// エラーが起こった時の処理
    void _errorOccur(); 
    ///  事故が起こった瞬間か
    bool _accidentOccur;
    ///  事故が起こっているか
    bool _isAccident;
    ///  事故が起こってから何秒間か
    int _accidentTime;
   /// 横ずれの速度(m/msec)
    double _errorVelocity;
    /// 自車が最も右側にいる時の右側対向車線
    Lane* _onComingLane();
   /// エラーの種類
    string _errorType;
    /// エラーの候補
    // 同時にエラーが発生したとき対策
    std::vector<std::string> _candidates;
     
    /// statファイルへの書き込みが初回か
    static bool _initWrite;
    ///
    static int _stopNAccident;
    ///
    static int _maxTotal;
    /// シミュレーションを終わらせるか
    static bool _stopRun;
     /// シミュレーションを終わらせるか
    static time_t  _startTime;
 
}; 
#endif //__ERRORCONTROLLER_H_
