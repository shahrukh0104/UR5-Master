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

#include <iostream>
#include "split.h"

int example(int argc, char *argv[])
{
    std::string input;
    std::cin >> input;

    std::vector<std::string> tokens = stringhelper::split(input, ':',
                                                stringhelper::KEEP_EMPTY_PARTS);

    for (size_t i = 0; i < tokens.size(); ++i)
    {
        std::cout << tokens[i] << "\n";
    }
    std::cin.ignore();
    std::cin.get();
    return 0;
}
