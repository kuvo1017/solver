/* **************************************************
 * Copyright (C) 2014 ADVENTURE Project
 * All Rights Reserved
 **************************************************** */
#include "Random.h"
#include <limits>
#include <iostream>
#include <istream>
#include <ostream>
#include <iterator>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <mt19937ar.h>
#ifdef _OPENMP
#include <omp.h>
#endif //_OPENMP

using namespace std;

//#define TEST_SERIAL_NUMBER
//**********************************************************************
RandomGenerator::~RandomGenerator() {}

//**********************************************************************
void StdRand::setSeed(unsigned long seed)
{
    srand(seed);
}

//----------------------------------------------------------------------
unsigned long StdRand::nextInt()
{
    return rand();
}

//----------------------------------------------------------------------
unsigned long StdRand::maxInt() const
{
    return RAND_MAX;
}

//----------------------------------------------------------------------
unsigned long StdRand::nextInt(long max)
{
    return static_cast<unsigned long>(nextReal() * max);
}

//----------------------------------------------------------------------
double StdRand::nextReal()
{
    return ((double)std::rand() / (double)(RAND_MAX));
}

double StdRand::nextReal(double max)
{
    return nextReal() * max;
}

//**********************************************************************
unsigned long LinearCongruential::maxInt() const
{
    return numeric_limits<unsigned long>::max();
} 

//----------------------------------------------------------------------
unsigned long LinearCongruential::nextInt()
{
    _x *= 1566083941UL;
    _x += 1;
    return _x;
}

//----------------------------------------------------------------------
unsigned long LinearCongruential::nextInt(unsigned long max)
{
    return (unsigned long)(max * nextReal());
}

//----------------------------------------------------------------------
double LinearCongruential::nextReal()
{
    return (double)(nextInt()) / (double)(maxInt());
}

//----------------------------------------------------------------------
double LinearCongruential::nextReal(double max)
{
    return max * nextReal();
}

//**********************************************************************
void MT19937::setSeed(unsigned long seed)
{
    init_genrand(seed);
}

//----------------------------------------------------------------------
unsigned long MT19937::nextInt()
{
    return nextInt31();
}

//----------------------------------------------------------------------
unsigned long MT19937::maxInt() const
{
    return 0x7fffffff;
}

//----------------------------------------------------------------------
unsigned long MT19937::nextInt(unsigned long max)
{
    return static_cast<unsigned long>(nextReal2() * max);
    // 半開区間版を利用する必要
}

//----------------------------------------------------------------------
double MT19937::nextReal()
{
    return nextReal1();
}

//----------------------------------------------------------------------
double MT19937::nextReal(double max)
{
    return nextReal1() * max;
}

//----------------------------------------------------------------------
unsigned long MT19937::nextInt32(void)
{
    return genrand_int32();
}

//----------------------------------------------------------------------
long MT19937::nextInt31(void)
{
    return genrand_int31();
}

//----------------------------------------------------------------------
double MT19937::nextReal1(void)
{
    return genrand_real1();
}

//----------------------------------------------------------------------
double MT19937::nextReal2(void)
{
    return genrand_real2();
}

//----------------------------------------------------------------------
double MT19937::nextReal3(void)
{
    return genrand_real3();
}

//----------------------------------------------------------------------
double MT19937::nextRealRes53(void) 
{ 
    return genrand_res53();
} 

//----------------------------------------------------------------------
istream& operator>>(istream& is, MT19937& gen)
{
    unsigned long* it = gen.mt;
    for (size_t i = 0; i < gen.N; i++) 
        is >> *(it++);
    return is;
}

//----------------------------------------------------------------------
ostream& operator<<(ostream& os, MT19937& gen)
{
    copy(gen.mt, gen.mt + gen.N,
         ostream_iterator<unsigned long>(os, " "));
    return os;
}

//**********************************************************************
#ifdef _OPENMP 
MultiStock::MultiStock(MT19937& gen) : _gen(gen)
{
    _gen = gen;
}

//----------------------------------------------------------------------
void MultiStock::setSeed(unsigned long seed)
{
    _gen.setSeed(seed);
}

//----------------------------------------------------------------------
unsigned long MultiStock::nextInt()
{
    // シングル版と異なる動作
    return static_cast<long>(nextInt32()>>1);
}

//----------------------------------------------------------------------
unsigned long MultiStock::maxInt() const
{
    return _gen.maxInt();
}

//----------------------------------------------------------------------
unsigned long MultiStock::nextInt(unsigned long max)
{
    // シングル版と異なる動作
    double real = nextInt32()*(1.0/4294967296.0); 
    /* divided by 2^32 */
    return static_cast<unsigned long>(real * max);
    // 半開区間版を利用する必要
}

//----------------------------------------------------------------------
double MultiStock::nextReal()
{
    // シングル版と異なる動作
    return nextInt32()*(1.0/4294967295.0); 
    /* divided by 2^32-1 */ 
}

//----------------------------------------------------------------------
double MultiStock::nextReal(double max)
{
    return nextReal() * max;
}

//----------------------------------------------------------------------
unsigned long MultiStock::nextInt32()
{
    int series;

    // 最大値 0 なら通常の乱数を返す
    if (_stockMax == 0)
    {
        return _gen.nextInt32();
    }

    // 非並列なら全体使用数から系列を取る
    // 全部使用済なら最初の系列を使って後の処理で追加する
    if (_series.size() == 0)
    {
        if (_allUsed == _used.size() * _stockMax)
        {
            series = 0;
        }
        else
        {
            series = _allUsed % _used.size();
        }
    }

    // 並列ならスレッド番号から系列を取る
    else
    {
        int thread = omp_get_thread_num();
        assert(thread < _series.size());
        series = _series[thread];
        assert(series >= 0 && series < _used.size());
    }

    // ストックデータを返す、最大値が足りない場合は追加する
    if (_used[series] >= _stockMax)
    {
        _stockMax++;
        createData();
    }
    int dataPos = _used.size() * _used[series] + series;
    _used[series] += 1;
    _allUsed++;
    return _data[dataPos];
}

//----------------------------------------------------------------------
void MultiStock::setMax(int stockMax)
{
    // 非並列用に１系列だけ作っておく
    _stockMax = stockMax;
    ready(1);
}

//----------------------------------------------------------------------
void MultiStock::ready(int seriesMax)
{
    if (_stockMax != 0)
    {
        // 系列を増やす
        int seriesMaxOld = _used.size();
        if (seriesMax > seriesMaxOld)
        {
            for (int i=seriesMaxOld; i<seriesMax; i++)
            {
                _used.push_back(0);
            }
        }

        // 乱数を作成する
        createData();
    }
}

//----------------------------------------------------------------------
void MultiStock::beginMulti(int series)
{
    if (_stockMax != 0)
    {
        int seriesSize = _series.size();
        int thread = omp_get_thread_num();
        for (int i = seriesSize; i <= thread; i++)
        {
            _series.push_back(-1);
        }
        _series[thread] = series;
    }
}

//----------------------------------------------------------------------
void MultiStock::endMulti()
{
    if (_stockMax != 0)
    {
        _series.clear();

        // 使用済データを詰める、その分だけ後ろに乱数を設定する
        if (_allUsed != 0)
        {
            int dataSize = _used.size() * _stockMax;
            int dataOld = 0;
            for (int dataNew = 0; dataNew < dataSize; dataNew++)
            {
                for (; dataOld < dataSize; dataOld++)
                {
                    if (_used[dataOld % _used.size()]
                        <= dataOld / _used.size())
                    {
                        break;
                    }
                }
                if (dataOld < dataSize)
                {
                    _data[dataNew] = _data[dataOld++];
                }
                else
                {
#ifdef TEST_SERIAL_NUMBER
                    _data[dataNew] = dataNew;
#else //TEST_SERIAL_NUMBER
                    _data[dataNew] = _gen.nextInt32();
                }
#endif //TEST_SERIAL_NUMBER
            }
            for (int i=0; i<_used.size(); i++)
            {
                _used[i] = 0;
            }
            _allUsed = 0;
        }
    }
}

//----------------------------------------------------------------------
void MultiStock::createData()
{
    int dataSizeCur = _data.size();
    int dataSize    = _used.size() * _stockMax;
    for (int i=dataSizeCur; i<dataSize; i++)
    {
#ifdef TEST_SERIAL_NUMBER
        _data.push_back(i);
#else //TEST_SERIAL_NUMBER
        _data.push_back(_gen.nextInt32());
    }
#endif //TEST_SERIAL_NUMBER
}

void MultiStock::testPrint(const char* title, bool detail)
{
    int seriesMax = _used.size();
    if (_allUsed != 0)
    {
        std::cout << "MultiStock " << title << " max " << _stockMax
                  << " all used " << _allUsed << endl;
    }
    if (detail)
    {
        std::cout << "Used and Data" << endl;
        for (int i=0; i<seriesMax; i++)
        {
            std::cout << i << ":" << _used[i];
            for (int j=0; j<_stockMax; j++)
            {
                std::cout << "\t" << _data[j * seriesMax + i];
            }
            std::cout << endl;
        }
        std::cout << "Series";
        for (int i=0; i<_series.size(); i++)
        {
            std::cout << " " << _series[i];
        }
        std::cout << endl;
    }
}
#endif //_OPENMP

/******************************************************************************/
int BinomialDist::operator()(int n, double p)
{
    // 定義どおりの実装．n 回繰り返す．
    int retval = 0;
    for (int i = 0; i < n; i++)
    {
        if (gen.nextReal() < p)
            retval++;
    }
    return retval;
}

/******************************************************************************/
int PoissonDist::operator()(double lambda)
{
    int k;
    double p = exp(lambda) * gen.nextReal();
    for (k = 0; p > 1.0; k++)
    {
        p *= gen.nextReal();
    }
    return k;
}

/******************************************************************************/
double ExponentialDist::operator()(double lambda)
{
    return - log(gen.nextReal()) / lambda;
}

/******************************************************************************/
double NormalDist::generate_0_1()
{
    double retval;
    if (!phase)
    {
        double s;
        do
        { 
            r1 = 2 * gen.nextReal() - 1;
            r2 = 2 * gen.nextReal() - 1;
            s = r1 * r1 + r2 * r2;
        } while(s >= 1 || s == 0);

        retval = r1 * sqrt(-2 * log(s) / s);
        phase = true;
    }
    else
    {
        double s = r1 * r1 + r2 * r2;
        retval = r2 * sqrt(-2 * log(s) / s);
        phase = false;
    }
    return retval;
}

/******************************************************************************/
#ifndef _OPENMP
MT19937 Random::_gen;
#else //_OPENMP
MT19937 Random::_genOrginal;
MultiStock Random::_gen(Random::_genOrginal);
#endif //_OPENMP
NormalDist Random::_normal(Random::_gen);
BinomialDist Random::_binom(Random::_gen);
PoissonDist Random::_poisson(Random::_gen);
ExponentialDist Random::_exp(Random::_gen);

void Random::setSeed(unsigned long seed)
{
#ifdef _OPENMP
#pragma omp critical (Random)
    {
#endif //_OPENMP
        _gen.setSeed(seed);
#ifdef _OPENMP
    }
#endif //_OPENMP
}

int Random::uniform(int max)
{
    int ret;
#ifdef _OPENMP
#pragma omp critical (Random)
    {
#endif //_OPENMP
        ret = _gen.nextInt(max);
#ifdef _OPENMP
    }
#endif //_OPENMP
    return ret;
}

int Random::uniform(int min, int max)
{
    assert(min <= max);
    return min + uniform(max - min);
}

double Random::uniform()
{
    double ret;
#ifdef _OPENMP
#pragma omp critical (Random)
    {
#endif //_OPENMP
        ret = _gen.nextReal();
#ifdef _OPENMP
    }
#endif //_OPENMP
    return ret;
}

double Random::uniform(double min, double max)
{
    double ret;
#ifdef _OPENMP
#pragma omp critical (Random)
    {
#endif //_OPENMP
        ret = min + _gen.nextReal(max - min);
#ifdef _OPENMP
    }
#endif //_OPENMP
    return ret;
}

bool Random::biasedCoinFlip(double p)
{
    return (uniform() < p);
}

double Random::normal(double mu, double sigma)
{
    double ret;
#ifdef _OPENMP
#pragma omp critical (Random)
    {
#endif //_OPENMP
        ret = _normal(mu, sigma);
#ifdef _OPENMP
    }
#endif //_OPENMP
    return ret;
}

int Random::poisson(const double lambda)
{
    int ret;
#ifdef _OPENMP
#pragma omp critical (Random)
    {
#endif //_OPENMP
        ret = _poisson(lambda);
#ifdef _OPENMP
    }
#endif //_OPENMP
    return ret;
}

double Random::exponential(double lambda)
{
    double ret;
#ifdef _OPENMP
#pragma omp critical (Random)
    {
#endif //_OPENMP
        ret = _exp(lambda);
#ifdef _OPENMP
    }
#endif //_OPENMP
    return ret;
}

std::vector<int> Random::randomOrder(int max)
{
    std::vector<int> order;
    order.reserve(max);
    for (int i=0; i<max; i++)
    {
        order.push_back(i);
    }
    if (order.size()>1)
    {
        std::pointer_to_unary_function<int, int> rnd(&uniform);
        std::random_shuffle(order.begin(), order.end(), rnd);
    }
    return order;
}

#ifdef _OPENMP

void Random::multiStockSetMax(int stockMax)
{
#pragma omp critical (Random)
    {
        _gen.setMax(stockMax);
    }
}

void Random::multiStockReady(int seriesMax)
{
#pragma omp critical (Random)
    {
        _gen.ready(seriesMax);
    }
}

void Random::multiStockBeginMulti(int series)
{
#pragma omp critical (Random)
    {
        _gen.beginMulti(series);
    }
}

void Random::multiStockEndMulti()
{
#pragma omp critical (Random)
    {
        _gen.endMulti();
    }
}

void Random::multiStockTestPrint(const char* title)
{
#pragma omp critical (Random)
    {
        _gen.testPrint(title, false);
    }
}

#endif //_OPENMP

void Random::multiStockTest()
{
#ifdef _OPENMP
    for (int i = 0; i < 2; i++)
    {
        multiStockReady(5);
        _gen.testPrint("Ready 5", true);
#pragma omp parallel for
        for (int j = 0; j < 3; j++)
        {
            multiStockBeginMulti(j);
            cout << "Loop " << j << " thread " << omp_get_thread_num() << endl;
            cout << "Get " << _gen.nextInt32() << " in loop " << j << endl;
            cout << "Get " << _gen.nextInt32() << " in loop " << j << endl;
            if (j == 1)
            {
                cout << "Get " << _gen.nextInt32() << " in loop " << j << endl;
                cout << "Get " << _gen.nextInt32() << " in loop " << j << endl;
                cout << "Get " << _gen.nextInt32() << " in loop " << j << endl;
                cout << "Get " << _gen.nextInt32() << " in loop " << j << endl;
            }
        }
        _gen.testPrint("Multi", true);
        multiStockEndMulti();
        _gen.testPrint("End Multi", true);
        cout << "Get " << _gen.nextInt32() << " in single" << endl;
        cout << "Get " << _gen.nextInt32() << " in single" << endl;
        cout << "Get " << _gen.nextInt32() << " in single" << endl;
        cout << "Get " << _gen.nextInt32() << " in single" << endl;
        cout << "Get " << _gen.nextInt32() << " in single" << endl;
        cout << "Get " << _gen.nextInt32() << " in single" << endl;
        cout << "Get " << _gen.nextInt32() << " in single" << endl;
        _gen.testPrint("Single", true);
    }
#endif //_OPENMP
}
