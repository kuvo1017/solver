#ifndef __ERROR_CONTROLLER_H__
#define __ERROR_CONTROLLER_H__

#include <iostream>
#include "VirtualLeader.h"
#include "picojson.h"

using namespace std;

class Vehicle;
class Lane;
class Intersection;

 /// 事故シミュレーション用のエラーや事故状態を管理するクラス
/**
 * 各車両が一つずつErrorControllerオブジェクトを
 * 保持しており、エラーになるかどうかなどの処理を
 * このクラスの中で行っている。
 * ERROR_MODEを定義することで、全エラーが発生する
 */

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

    /// 追突エラー状態かどうか↲
    bool isRearError() const;

    /// 追突事故の処理
    VirtualLeader* rearError(VirtualLeader* resultLeader); 

      /// エラー継続時間
    int rearErrorTime() const;
 
  protected:

    /// 予測エラーか
    bool _isRearError;

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

  public:

    /// 出合い頭エラー状態かどうか↲
    bool isPassingError() const;

    /// 障害物で認知できていない車両の集合を返す
    void passingError();

    /// 障害物で認知できていない車両の集合を返す
    const std::vector<Vehicle*>* invisibleVehicles() const;

    /// 認知できていない車両でを追加
    void setInvisibleVehicle(Vehicle* vehicle);

    // 認知できてない車両を消去
    void resetInvisibleVehicles();

  protected:

    ///　障害物エラーか
    int _isPassingError; 

    /// 認知できていない車両の配列
    std::vector<Vehicle*> _invisibleVehicles;

    //@}

    //======================================================================
    /**
     * @name 右左折に関する関数
     */
    //@{

  public:

    /// 右左折エラー状態かどうか↲
    bool isLRError() const;

    /// 傲慢なエージェントであるかどうかを返す関数
    bool isArroganceLR();

    /// 右左折事故が起こるかどうかを計算する関数
    void LRError(Vehicle* thatV,double thisTti,double thatTti) ; 
    void LRError(double thisTti,double thatTtp) ;  
  protected:

    ///　傲慢なエージェントか
    bool _isArroganceLR;
    ///  左折エラーかどうか
    bool _isLRError;

    //@}
    ///======================================================================
    /**
     * @name 進路変更事故に関する関数
     */
    //@{

  public:

    /// 進路変更エラーかどうか↲
    bool isShiftError() const;

    /// 進路変更エラーが起こるかどうかを計算する関数
    bool shiftError();

    /// 進路変更エラーを終わらせる
    void endShiftError();     

  protected:

    /// 進路変更エラーかどうか
    bool _isShiftError;

    /// 一度車線変更してから立った時間
    int _shiftTime;

    //@}

    ///======================================================================
    /**
     * @name 正面衝突事故に関する関数
     */
    //@{

  public:
    /// 正面衝突エラーかどうかを返す
    bool isHeadError() const;

     /// 正面衝突エラーが起きるかを計算
    bool headError();

    /// 横ずれエラーが起きている時間
    int headErrorTime();

    /// 正面衝突事故が起きている時に横ずれの速度を返す
    double errorVelocity();

  protected:

    /// 正面衝突エラーかどうか
    bool _isHeadError;

    /// 正面衝突エラーが発生している時間
    int _headErrorTime;

    /// 横ずれの速度(m/msec)
    double _errorVelocity;

    /// 自車が最も右側にいる時の右側対向車線
    Lane* _onComingLane();

    /// 正面衝突事故が起きてるかどうかをチェック
    bool _checkHeadAccident();


    //@}

    ///======================================================================
    /**
     * @name エラー全般に関する関数
     */
    //@{

  public:


    /// エラーの種類を返す
    string errorType() const;

    // そのステップで発生するエラーを決める関数
    /**
     * 複数のエラーが発生した時に、 どのエラーが
     * 発生するかを決める。エラーは同時に一種類しか起こらない
     */ 
    void chooseError();

    /// 交差点におけるエラーの発生を計算する関数
    /**
     * 交差点におけるエラーとは出会い頭、右左折エラーのこと
     * これは新たな単路に進入するとき、VehicleRunner.cpp内の
     * Intersection2Section内にて使用することで、次の交差点に
     * 進入するときに傲慢なエージェントかを決定する
     */ void errorInIntersection(Intersection* next);

    /// エラーが起きたときに外部ファイルに記入する
    void writeError();

  protected:
    /// エラーの種類
    string _errorType;

    /// エラーの候補
    // 同時にエラーが発生したとき対策
    std::vector<std::string> _candidates;

    /// エラーが終了した時に車両の色を変える関数 
    void _errorEnd();

    /// エラーが起こった時の処理をする関数
    void _errorOccur(); 
    //@}

    ///======================================================================
    /**
     * @name 事故全般に関する関数
     */
    //@{

  public:
    /// 事故状態かどうか↲
    bool isAccident() const;

    /// 事故が起こった時の処理
    void accidentOccur(std::string collidType);

    /// 事故時間↲
    int accidentTime() const;

    /// 事故状態かどうかをチェックして、事故状態ならエージェント消去
    bool accidentCheck();

    /// 事故が起きたときに外部ファイルに記入する
    void writeAccident();


  protected:
    ///  事故が起こった時の処理をする関数
    bool _accidentOccur;

    /// 事故が起きいるか
    /// 事故が起きてもしばらくは車両は道路に
    /// 存在するため
    bool _isAccident;

    ///  事故が起こってから何秒間か
    int _accidentTime;
    //@}

    ///======================================================================
    /**
     * @name その他
     */
    //@{

  public:
    /// シミュレーションの終了時間をJSONから読み込む
    static bool stopRun();

    /// 事故の統計データを計算
    static void checkStatData(); 

    /// 事故の統計データをファイルに書き出し
    static void writeStatData(int totalP,int totalT,string time);

    /// stopRunをtrueにする
    static void endRun();

  protected: 

    /// 車両オブジェクト
    Vehicle* _vehicle;

    /// statファイルへの書き込みが初回か
    static bool _initWrite;

    /// 最大事故発生数
    static int _stopNAccident;

    /// シミュレーションを終わらせるか
    static bool _stopRun;

    /// シミュレーションが始まった時間
    static time_t  _startTime;

    //@}
}; 
#endif //__ERRORCONTROLLER_H_
