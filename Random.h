/* **************************************************
 * Copyright (C) 2014 ADVENTURE Project
 * All Rights Reserved
 **************************************************** */
#ifndef __RANDOM_H__
#define __RANDOM_H__

#include <iosfwd>
#include <functional>
#include <cstddef>
#include <vector>

/**
 * @addtogroup RandomNumber
 * @brief 乱数生成モジュール
 */

//######################################################################
// 乱数生成器

/**
 * @addtogroup RandomGenerator
 * @brief 擬似乱数生成器
 * @ingroup RandomNumber
 */

/// 乱数生成器の抽象基底クラス
/** 
 * @ingroup RandomGenerator
 */
class RandomGenerator
{
public:
    virtual ~RandomGenerator() = 0;
    /// 乱数の種を設定する
    /**
     * @param seed 乱数の種
     */
    virtual void setSeed(unsigned long seed) = 0;

    /// [0, maxInt()] を返す
    /**
     * @return 0以上maxInt()以下のunsigned long型乱数
     */
    virtual unsigned long nextInt() = 0;

    /// nextInt() で返せる最大値を返す
    virtual unsigned long maxInt() const = 0;

    /// [0, @p max) を返す
    /**
     * @param max 乱数の最大値を決める整数
     * @return 0以上@p max【未満】のunsigned long型乱数
     *
     * @attention 最大値は @p max - 1 であることに注意
     */
    virtual unsigned long nextInt(unsigned long max) = 0;

    /// [0, 1] を返す
    /**
     * @return 0以上1以下のdouble型乱数
     */

    virtual double nextReal() = 0;

    /// [@p min, @p max] を返す
    /**
     * @param max 乱数の最大値
     * @return 0以上@p max 以下のdouble型乱数
     */
    virtual double nextReal(double max) = 0;
};
  
/// 標準乱数による乱数生成器
/** 
 * 標準乱数生成関数 std::rand() のラッパ．
 *
 * @attention 
 * たいていのプラットフォームでは線形合同法による質のよくない．
 * また内部の static 変数を状態変数として利用するので注意が必要
 *
 * @ingroup RandomGenerator
 */
class StdRand : public RandomGenerator
{
public:
    virtual ~StdRand() {}
    virtual void setSeed(unsigned long seed);
    virtual unsigned long nextInt();
    virtual unsigned long nextInt(long max);
    virtual unsigned long maxInt() const;
    virtual double nextReal();
    virtual double nextReal(double max);
};

/// 線形合同法による乱数生成器
/**
 * std::rand() とは独自の実装．
 * 移植性のため作成． 
 *
 * @ingroup RandomGenerator
 */
class LinearCongruential : public RandomGenerator
{
public:
    LinearCongruential(long seed = 0) : _x(seed) {}
    virtual void setSeed(unsigned long seed)
    {
        _x = seed; 
    }
    virtual unsigned long nextInt();
    virtual unsigned long nextInt(unsigned long max);
    virtual unsigned long maxInt() const;
    virtual double nextReal();
    virtual double nextReal(double max);
    
private:
    unsigned long _x;
};

/// メルセンヌ・ツイスタ法による乱数生成器
/**
 * 松本眞 ・西村拓士による乱数生成アルゴリズム
 * http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/mt.html
 *
 * 詳細は lib/mt19937ar を参照．
 *
 * @ingroup RandomGenerator
 */
class MT19937 : public RandomGenerator
{
public:
    MT19937() : mti(N+1) {}
    virtual ~MT19937() {}
    virtual void setSeed(unsigned long seed);
    virtual unsigned long nextInt();
    virtual unsigned long nextInt(unsigned long max);
    virtual unsigned long maxInt() const;
    virtual double nextReal();
    virtual double nextReal(double max);

    /** 閉区間[0,0xffffffff].
     * used by another function */
    unsigned long nextInt32();
    /** 閉区間[0,0x7fffffff] */
    long nextInt31();
    /** 閉区間[0,1]の実数 */
    double nextReal1();
    /** 半閉区間[0,1)の実数 */
    double nextReal2();
    /** 開区間(0,1)の実数 */
    double nextReal3();
    /** 半閉区間[0,1)の実数．53bit の解像度を持つ */
    double nextRealRes53(void);
  
    friend std::istream& operator>>(std::istream& is, MT19937& gen);
    friend std::ostream& operator<<(std::ostream& os, MT19937& gen);
    
private:
    static const std::size_t N = 624;
    static const std::size_t M = 397;  
    /// constant vector a
    static const unsigned long MATRIX_A = 0x9908b0dfUL;
    /// most significant w-r bits
    static const unsigned long UPPER_MASK = 0x80000000UL;
    /// least significant r bits/
    static const unsigned long LOWER_MASK = 0x7fffffffUL;
    
    /// 内部状態を表す配列
    unsigned long mt[N];
    /// どうやって初期化されたかどうかを表す
    int mti;
};

#ifdef _OPENMP
/// 並列ストック乱数生成器
/** 
 * メルセンヌ・ツイスタ乱数生成器の値を並列ストックして順に返す
 * 並列処理でも乱数再現性を保つための処理
 * スレッド数、ストック最大数に関わらず再現可能、集中発生数を変えると再現しない
 * ストック最大数が 0 ならストック自体を行わない
 * 並列処理なら系列ごとに古いデータから順に返す
 * 非並列処理なら全体で古いデータから順に返す
 * 系列ごとに別の乱数を使うのは種の独立性を保のが難しい
 *
 * @note _OPENMP定義時のみ有効
 * @ingroup RandomGenerator
 */
class MultiStock : public RandomGenerator
{
public:
    MultiStock(MT19937& gen);
    virtual ~MultiStock() {}
    virtual void setSeed(unsigned long seed);
    virtual unsigned long nextInt();
    virtual unsigned long nextInt(unsigned long max);
    virtual unsigned long maxInt() const;
    virtual double nextReal();
    virtual double nextReal(double max);

    /// INT32 データ取得
    unsigned long nextInt32();

    /// 最大数設定、最初のみ呼ぶ
    void setMax(int stockMax);

    /// 準備、並列処理の前に呼ぶ
    void ready(int seriesMax);

    /// 並列開始
    void beginMulti(int series);

    /// 並列終了
    void endMulti();

private:
    /// データ作成
    void createData();

public:
    /// テスト表示
    void testPrint(const char* title, bool detail);

private:
    /// メルセンヌ・ツイスタ乱数生成器
    MT19937& _gen;

    /// ストック最大数、系列ごと
    int _stockMax;

    /// 使用数、系列ごと
    std::vector<int> _used;

    /// 全体使用数
    int _allUsed;

    /// 系列番号、スレッドごと、並列処理中のみ存在
    std::vector<int> _series;

    /// 並列ストックデータ、INT32、系列＊最大数
    /// 古いものから使うため系列の先頭から並べる
    ///   系列0 - 0 3 6  9
    ///   系列1 - 1 4 7 10
    ///   系列2 - 2 5 8 11
    /// 使用済を詰める時も古いデータから並べ直す
    /// 上の 1 を詰める時は 2 を 1 の場所に移す
    /// 非並列の場合は詰めた後で古い順に使う、使用済数は系列に入れる
    std::vector<unsigned long> _data;
};
#endif //_OPENMP

//######################################################################
// 確率分布に応じた乱数を生成するオブジェクト

/**
 * @addtogroup RandomDistribution
 * @brief 乱数の分布関数
 * @ingroup RandomNumber
 */

/// 正規分布関数オブジェクト
/** 
 * @f[
 *   f(x) = \frac{1}{\sqrt{2 \pi} \sigma} e^{-(x-\mu)^2 / 2 {\sigma}^2}
 * @f]
 *
 * Box-Muller 法（極座標法）による．
 * 独立した二系列の正規分布を生成する．
 *
 * @ingroup RandomDistribution
 */
class NormalDist : public std::binary_function<double, double, double>
{
public:
    explicit NormalDist(RandomGenerator& gen) : gen(gen), phase(false) {}
    /// 平均 @p mu, 標準偏差 @p sigma の分布
    double operator()(double mu, double sigma)
    {
        return mu + sigma * generate_0_1();
    }
    
private:
    /// 平均 0, 標準偏差 1 の分布
    double generate_0_1();
  
private:
    RandomGenerator& gen;
    double r1, r2;
    bool phase;
};

/// 二項分布関数オブジェクト
/**
 *  p の確率で起こる事象が n 回繰り返されるときに，それが x 回起こる確率．
 *  以下の数式で与えられる．
 * @f[
 *   f(x) = {}_n\mathrm{C}_x p^x (1-p)^{n-x},\; x=0,1,\ldots,n,\; 0 < p < 1
 * @f]
 *
 * @ingroup RandomDistribution
 */
class BinomialDist : public std::binary_function<int, double, int>
{
public:
    explicit BinomialDist(RandomGenerator& gen) : gen(gen) {}

    /// 試行回数@p nのうち，確率@p pの事象が発生する回数を返す
    int operator()(int n, double p);
    
private:
    RandomGenerator& gen;
};

/// ポアソン分布関数オブジェクト
/**
 * 確率関数は以下になる.
 * @f[ f(x) = \frac{e^{-\lambda} {\lambda}^x}{x!} @f]
 * 平均 @f$ \lambda @f$,分散 @f${\lambda}^2 @f$.
 *
 * @ingroup RandomDistribution
 */
class PoissonDist : public std::unary_function<double, int>
{
public:
    explicit PoissonDist(RandomGenerator& gen) : gen(gen) {}

    /// 平均 @p lambda の分布.
    int operator()(double lambda);
private:
    RandomGenerator& gen;
};

/// 指数分布関数オブジェクト
/**
 * 次の確率密度関数に従った分布.
 * @f[ f(x) = \lambda e^{- \lambda x} @f]
 * ただし,@f$ x \leq 0,\; \lambda > 0 @f$
 *
 * このとき平均 @f$ 1 / \lambda @f$,分散 @f$ 1 / {\lambda}^2 @f$ になる.
 *
 * @ingroup RandomDistribution
 */
class ExponentialDist : public std::unary_function<double, double>
{
public:
    explicit ExponentialDist(RandomGenerator& gen) : gen(gen) {}

    /// 平均 1/@p lambda の分布.
    double operator()(double lambda);
private:
    RandomGenerator& gen;
};

//######################################################################
// 乱数のインターフェース

/// さまざまな分布を扱える乱数クラス
/**
 *  利便性のため，このファイル中のモジュールをまとめて
 *  static メンバのみを持つクラスとして提供する．
 *  乱数生成器はデフォルトでメルセンヌ・ツイスタを使用．
 *
 * @ingroup RandomNumber
 */
class Random
{
public:
    /// 乱数の種を設定する
    /**
     * @param seed 乱数の種
     */
    static void setSeed(unsigned long seed);
  
    /** @name 様々な条件に従った乱数を返す */
    //@{

    /// [0, @p max) の一様分布に従う乱数を返す
    /**
     * @param max 乱数の最大値を決める整数
     * @return 0以上@p max【未満】の一様分布に従うint型乱数
     *
     * @attention RandomGenerator::nextInt(long) と異なりこちらは半開区間
     */
    static int uniform(int max);

    /// [@p min, @p max) の一様分布に従う乱数を返す
    /**
     * @param min 乱数の最小値
     * @param max 乱数の最大値を決める整数
     * @return @p min 以上 @p max 【未満】の一様分布に従うint型乱数
     */
    static int uniform(int min, int max);
  
    /// [0, 1) の一様分布にしたがう乱数を返す
    /**
     * @return 0以上1【未満】の一様分布に従うdouble型乱数
     */
    static double uniform();

    /// [@p min, @p max) の一様分布に従う乱数を返す
    /**
     * @param min 乱数の最小値
     * @param max 乱数の最大値を決める実数
     * @return @p min 以上 @p max 【未満】の一様分布に従うdouble型乱数
     */
    static double uniform(double min, double max);

    /// 平均 @p mu, 標準偏差 @p sigma のガウス分布に従う乱数を返す
    static double normal(double mu, double sigma);

    /// 平均 @p lamda のポアソン分布に従う乱数を返す
    static int poisson(double lambda);
  
    /// 平均 1/@p lambda の指数分布に従う乱数を返す
    static double exponential(double lambda);

    //@}

    /** @name ユーティリティ */
    //@{

    /// 確率 @p p のテスト
    /**
     * 「表が出る確率@p pのコインを投げて表が出たかどうか」に相当する
     *
     * @param p trueを返す確率
     */
    static bool biasedCoinFlip(double p);

    /// [0,max)の間の整数がランダムに入った数列を生成する
    /**
     * @p param max 並べ替える数の最大値を決める整数
     * @return ランダムに並べ替えられたvector
     */
    static std::vector<int> randomOrder(int max);

    //@}

#ifdef _OPENMP
    /** @name _OPENMPが定義時のみ有効 */
    //@{

    /// 並列ストック最大数設定、最初のみ呼ぶ
    static void multiStockSetMax(int stockMax);

    /// 並列ストック準備、並列処理の前に呼ぶ
    static void multiStockReady(int seriesMax);

    /// 並列ストック並列開始、各スレッドの最初に呼ぶ
    static void multiStockBeginMulti(int series);

    /// 並列ストック並列終了、シングルスレッドに戻った時に呼ぶ
    static void multiStockEndMulti();

    /// 並列ストックテスト表示
    static void multiStockTestPrint(const char* title);

    // @}
#endif //_OPENMP

    /// 並列ストックテスト
    static void multiStockTest();

private:
#ifndef _OPENMP
    /** @name _OPENMP未定義時のみ有効 */
    //@{

    /// 乱数生成オブジェクト
    static MT19937 _gen;

    //@}
#else //_OPENMP
    /** @name _OPENMP定義時のみ有効 */
    //@{

    /// 乱数生成オブジェクト
    static MT19937 _genOrginal;

    /// 並列ストック乱数オブジェクト
    static MultiStock _gen;

    //@}
#endif //_OPENMP

    static NormalDist _normal;
    static BinomialDist _binom;
    static PoissonDist _poisson;
    static ExponentialDist _exp;
};

#endif //__RANDOM_H__
