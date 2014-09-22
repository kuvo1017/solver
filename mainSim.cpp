#include <autogl.h>
#include <memory>
#include "AppSim.h"
#include <cstdlib>

using namespace std;

/// アプリケーションオブジェクト
auto_ptr<AppSim> app;

/// exit時に実行する関数
/**
 * appのデストラクタを明示的に呼び出す
 *
 * @sa App
 */
void exitFunc()
{
    app.reset();
}

/// advmates-simのmain関数に相当する関数
/**
 * AppSimulatorを作成して初期化を行い、シミュレーションを開始する．
 * 計算と結果の可視化（アニメーション）を同時に行う．
 */
void AutoGL_SetUp(int argc, char* argv[])
{
    /*
     * auto_ptrにより自動的にデストラクタが呼び出されるはずだが
     * 環境によりメモリリークが発生するため
     */
    atexit(exitFunc);

    app.reset(new AppSim());
    app->init(argc, argv, false);

    app->run();
}
