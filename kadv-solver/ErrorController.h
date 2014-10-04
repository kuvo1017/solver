#ifndef __ERROR_CONTROLLER_H__
#define __ERROR_CONTROLLER_H__

#include <iostream>
//#include "Vehicle.h"
#include "VirtualLeader.h"
using namespace std;

class Vehicle;

class ErrorController{ 
  public:
    ///
    ErrorController(Vehicle* vehicle);
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
    /// 進路変更エラーか終わったかどうか↲
    bool isShiftEnd() const;
    /// 進路変更エラーを終わらせる↲
    void shiftEnd() const;
    ///　事故状態かどうか↲
    bool isAccident() const;
    ///  エラー時間↲
    int rearErrorTime() const;
    ///　事故時間↲
    int accidentTime() const;
    /// 事故が起きたときに外部ファイルに記入する
    void writeAccident();
    /// エラーが起きたときに外部ファイルに記入する
    void writeError();
   /// 事故が起こった時の処理
  void accidentOccur();
  /// エラーが起こった時の処理
  void errorOccur(string type);
  ///　交障害物を認識している状態にする。
  void recogWall();
  /// 横ずれエラーが起きている時間
  int slideErrorTime();
  /// 
  VirtualLeader* rearError(VirtualLeader* resultLeader);

protected: 
  /// 車両オブジェクト
  Vehicle* _vehicle;
  int _errortime;
  /// 予測エラーか
  bool _rearError;
  /// 予測エラー時の前方車との間隔
  double _rearErrorLength;
  ///　予測エラー時の前方車の速度
  double _rearErrorVelocity;
  ///　予測エラー継続時間
  int _rearErrorTime;
  /// 横ずれエラー時間
  int _slideErrorTime;
  ///　障害物エラーか
  int _isPassingError;
  ///　傲慢なエージェントか
  bool _isArrogance;
  ///  左折エラーかどうか
  bool _isLRError;
  /// 進路変更エラーかどうか
  bool _isShiftError;
  /// 横にずれるエラーかどうか
  bool _isHeadError;
  /// 進路変更終了か
  bool _isShiftEnd;
  /// 遅い車か
  bool _isSlow;
  ///  それぞれの制限速度
  double _vIde;
  ///  前の車との速度差
  double _velocityDifference;
  ///  先行者のid
  string _rearId;
  ///  事故が起こった瞬間か
  bool _accidentOccur;
  ///  事故が起こっているか
  bool _isAccident;
  ///  事故が起こってから何秒間か
  int _accidentTime;
  /// 交差点でクロスする車線が、障害物で見えていない状態であるかどうか。
  bool _isWall;
  /// 一度車線変更してから立った時間
  int _shiftTime;
 
}; 
#endif //__ERRORCONTROLLER_H_


