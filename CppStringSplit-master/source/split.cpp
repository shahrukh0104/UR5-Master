/*****************************************************************************
* Copyright (C) 2016 Sourabh Anand
*
* Author(s): Sourabh Anand <sourabhanand.cs@gmail.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the MIT License
*
* You should have received a copy of the MIT License along with this program;
* if not, visit https://opensource.org/licenses/MIT
*
*****************************************************************************/

#include "../include/split.h"
#include <iostream>

using namespace std;

using namespace stringhelper;

/*
 * Splits a string according to the given delimiter and returns a vector of
 * tokens.
 * 
 * @param fullStr (string): the string to be splitted
 * @param delimiter (string): delimiter accroding to which string is to be splitted
 * @param splitBehaviour (Behaviour): two acceptable enum values
 *
 * @return vector of splitted tokens
 *
 */
std::vector<std::string> stringhelper::split(std::string &fullStr,
                                             char delimiter,
                                             Behaviour splitBehaviour)
{
    std::vector<std::string> tokens;
    size_t idx, start = 0;
    std::string _temp;
    while ((idx = fullStr.find(delimiter, start)) != std::string::npos)
    {
        _temp = fullStr.substr(start, idx-start);
        if (_temp.length() == 0)
        {
            if (splitBehaviour == KEEP_EMPTY_PARTS)
            {
                tokens.push_back(_temp);
            }
        }
        else
        {
            tokens.push_back(_temp);
        }
        start = idx + 1;
    }
    _temp =  fullStr.substr(start);
    if (_temp.length() == 0)
    {
        if (splitBehaviour == KEEP_EMPTY_PARTS)
        {
            tokens.push_back(_temp);
        }
    }
    else
    {
        tokens.push_back(_temp);
    }

    return tokens;
}
