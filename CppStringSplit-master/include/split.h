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

#ifndef __SPLIT_H__
#define __SPLIT_H__

#include <vector>
#include <string>

namespace stringhelper {
    enum Behaviour { KEEP_EMPTY_PARTS, SKIP_EMPTY_PARTS };

    std::vector<std::string> split(std::string &fullStr, char delimiter,
                                   Behaviour = KEEP_EMPTY_PARTS);

} // namespace stringhelper

#endif // __SPLIT_H__
