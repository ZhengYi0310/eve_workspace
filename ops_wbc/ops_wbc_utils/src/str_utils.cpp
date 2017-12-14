/*************************************************************************
	> File Name: str_utils.cpp
	> Author: Yi Zheng 
	> Mail: hchengcq@gmail.com
	> Created Time: Wed 13 Dec 2017 05:17:05 PM PST
 ************************************************************************/

#include <cctype>
#include <iostream>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include "utils/str_utils.hpp"

using namespace std;
using namespace ops_wbc_utils;

bool StrUtils::have(const std::string& str, char ch)
{
    uint32_t length = str.length();
    for (uint32_t i = 0; i < length; i++)
    {
        if (str[i] == ch)
            return true;
    }

    return false;
}

bool StrUtils::have(const std::string& str, const std::vector<char>& chs)
{
    uint32_t size = chs.size();

    for (uint32_t i = 0; i < size; i++)
    {
        if (!StrUtils::have(str, chs[i]))
            return false;
    }

    return true;
}
