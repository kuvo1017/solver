#ifndef __VIRTUAL_LEADER_H__
#define __VIRTUAL_LEADER_H__

#include <string>

#define VL_DEBUG

/// 先行エージェントの情報を格納するクラス
/**
 * エージェントのrecongnize()の中で毎回更新される．
 *
 * 先行車だけでなく，信号などで停止したい位置の情報も含める．
 * MITRAMで言うところの「仮想先行車」．
 * VirtualLeaderの情報を元に速度決定
 */

using namespace std;

class VirtualLeader
{
  public:
    VirtualLeader();
#ifndef VL_DEBUG
    VirtualLeader(double distance, double velocity);
#else
    VirtualLeader(double distance, double velocity, std::string id);
#endif
    ~VirtualLeader();


    // 距離，速度情報をセットする
    void setInformation(double distance, double velocity);

    // 距離を返す
    double distance() const;

    // 速度を返す
    double velocity() const;

    // IDを返す
    const std::string& id() const;

     // リーダーの種類（先行車、信号など）を設定する
    void setType(string type);
 
    // リーダーの種類（先行車、信号など）を返す
    string getType();

#ifdef VL_DEBUG
    // 情報を表示する
    void print() const;
#endif

  private:
    /// このクラスを作成したエージェントからの距離
    double _distance;

    /// 速度
    double _velocity;

    // 加速度必要？
    //　種類
    string _type;


#ifdef VL_DEBUG
    // ID
    std::string _id;
#endif

};

#endif //__VIRTUAL_LEADER_H__

