#include "GVManager.h"
#include "AmuStringOperator.h"
#include "AmuConverter.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <cassert>

using namespace std;

map <string, string> GVManager::_strings;
map <string, double> GVManager::_numerics;
map <string, bool>   GVManager::_flags;
unsigned long GVManager::_maxTime;

//======================================================================
void GVManager::init()
{
    _maxTime = 0;
}

//======================================================================
bool GVManager::getVariable(const std::string& key, std::string* value)
{
    assert(_isStringKeyFound(key));
    *value = _strings[key];
    return true;
}

//======================================================================
string GVManager::getString(const std::string& key)
{
    assert(_isStringKeyFound(key));
    return _strings[key];
}

//======================================================================
bool GVManager::getVariable(const std::string& key, double* value)
{
    //cout << key << endl;
    assert(_isNumericKeyFound(key));
    *value = _numerics[key];
    return true;
}

//======================================================================
double GVManager::getNumeric(const std::string& key)
{
//    cout << key <<endl;
    assert(_isNumericKeyFound(key));
    return _numerics[key];
}

//======================================================================
bool GVManager::getFlag(const std::string& key)
{
    assert(_isFlagKeyFound(key));
    return _flags[key];
}

//======================================================================
unsigned long GVManager::getMaxTime()
{
    return _maxTime;
}

//======================================================================
bool GVManager::setNewString(const std::string& key, 
                             const std::string& value)
{
    // キーを探して、重複がなければ設定
    if (!_isStringKeyFound(key))
    {
        _strings[key] = value;
        return true;
    }
    else
    {
        return false;
    }
}

//======================================================================
bool GVManager::setNewNumeric(const std::string& key, 
                              const double value)
{
    // キーを探して、重複がなければ設定
    if(!_isNumericKeyFound(key))
    {
        _numerics[key] = value;
        return true;
    }
    else
    {
        return false;
    }
}

//======================================================================
bool GVManager::setNewNumeric(const std::string& key, 
                              const int value)
{
    return setNewNumeric(key, (double)value);
}

//======================================================================
bool GVManager::setNewFlag(const std::string& key,
                           const bool value)
{
    // キーを探して、重複がなければ設定
    if(!_isFlagKeyFound(key))
    {
        _flags[key] = value;
        return true;
    }
    else
    {
        return false;
    }
}

//======================================================================
bool GVManager::resetString(const std::string& key,
                            const std::string& value)
{
    _strings[key] = value;
    return true;
}

//======================================================================
bool GVManager::resetNumeric(const std::string& key,
                             const double value)
{
    _numerics[key] = value;
    return true;
}

//======================================================================
bool GVManager::resetNumeric(const std::string& key,
                             const int value)
{
    return resetNumeric(key, (double)value);
}

//======================================================================
bool GVManager::resetFlag(const std::string& key,
                          const bool value)
{
    _flags[key] = value;
    return true;
}

//======================================================================
bool GVManager::setVariablesFromFile(const std::string& fileName)
{
    // ファイルを読み込む
    ifstream ifs(fileName.c_str(), ios::in);
    if (!ifs)
    {
        cout << "no init file: " << fileName << endl;
        return false;
    }

    string str;
    while (ifs.good())
    {
        // 1行ごとの処理
        getline(ifs, str);
        AmuStringOperator::getAdjustString(&str);
        if (!str.empty())
        {
            vector<string> tokens;
            AmuStringOperator::getTokens(&tokens, str, '=');
            if (tokens.size()!=2)
            {
                cerr << "invalid format of global variable: "
                     << str << endl;
                cerr << "format is <symbol = value>" << endl;
            }
            else
            {
                if (!tokens[0].empty() && !tokens[1].empty())
                {
                    if (tokens[0]=="MAX_TIME")
                    {
                        _maxTime = AmuConverter::strtoul(tokens[1]);
                        continue;
                    }
                    // 数値か文字列か判断し，
                    // 数値であれば_numericsに，
                    // 文字列であれば_stringsに登録する
                    bool isNumeric = false;
                    if (isdigit(tokens[1][0])
                        || ( tokens[1].size()>=2
                             && tokens[1][0]=='-'
                             && isdigit(tokens[1][1])))
                    {
                        stringstream ss;
                        double d;
                        string s="";
                        ss << tokens[1];
                        ss >> d >> s;
                        if (s=="")
                        {
                            // 数値の登録
                            // キーが無ければ作成し，既にあれば上書きする
                            isNumeric = true;
                            resetNumeric(tokens[0], d);
                        }
                    }
                    if (!isNumeric)
                    {
                        // 文字列の登録
                        // キーが無ければ作成し，既にあれば上書きする
                        resetString(tokens[0], tokens[1]);
                    }
                }
            }
        }
    }
    return true;
}

//======================================================================
void GVManager::print()
{
    cout << "*** Global Variables ***" << endl;
    if (_maxTime>=100)
    {
        cout << "MAX_TIME=" << _maxTime << endl;
    }
    for (map<string, bool>::const_iterator where=_flags.begin();
         where!=_flags.end();
         where++)
    {
        cout << (*where).first << "="
             << ((*where).second?"true":"false") << endl;
    }
    for (map<string, double>::const_iterator where=_numerics.begin();
         where!=_numerics.end();
         where++)
    {
        cout << (*where).first << "=" << (*where).second << endl;
    }
    for (map<string, string>::const_iterator where=_strings.begin();
         where!=_strings.end();
         where++)
    {
        cout << (*where).first << ":" << (*where).second << endl;
    }
}

//======================================================================
bool GVManager::_isStringKeyFound(const std::string& key)
{
    map<string, string>::iterator where = _strings.find(key);
    if (where != _strings.end())
    {
        return true;
    }
    else
    {
        return false;
    }
}

//======================================================================
bool GVManager::_isNumericKeyFound(const std::string& key)
{
    map<string, double>::iterator where = _numerics.find(key);
    if (where != _numerics.end())
    {
        return true;
    }
    else
    {
        return false;
    }
}

//======================================================================
bool GVManager::_isFlagKeyFound(const std::string& key)
{
    map<string, bool>::iterator where = _flags.find(key);
    if (where != _flags.end())
    {
        return true;
    }
    else
    {
        return false;
    }
}
