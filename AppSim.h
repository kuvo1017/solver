#ifndef __APP_SIM_H__
#define __APP_SIM_H__

#include "AppMates.h"
#include "Visualizer.h"
#include <memory>

/// advmates-simアプリケーションクラス
/**
 * 計算と簡易可視化を同時に行う
 *
 * @ingroup Initialization
 */
class AppSim : public AppMates
{
public:
    AppSim(){};
    ~AppSim(){};

    /// 初期化とコマンドライン引数の処理
    virtual void init(int argc, char** argv, bool output);

protected:
    /// コマンドライン引数を処理する
    virtual void parseArgument(int argc, char** argv);

    /// 説明を出力する
    virtual void printUsage();

public:
    /// シミュレーションを開始する
    int run();

private:
    /// 可視化とGUIを担当するオブジェクト
    std::auto_ptr<Visualizer> _vis;
};

#endif //__APP_SIM_H__
