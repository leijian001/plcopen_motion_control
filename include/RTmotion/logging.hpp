/*
 * Copyright (c) 2020 Intel Corporation
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file logging.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

namespace RTmotion
{
#ifdef DEBUG
#define DEBUG_TEST 1
#else
#define DEBUG_TEST 0
#endif

// clang-format off
#define DEBUG_PRINT(fmt, ...) \
            do { if (DEBUG_TEST) fprintf(stderr, "%s:%d:%s(): \n" fmt, __FILE__, __LINE__, __func__, ##__VA_ARGS__); } while (0)

#define INFO_PRINT(fmt, ...) \
            do { fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)
// clang-format on

}  // namespace RTmotion