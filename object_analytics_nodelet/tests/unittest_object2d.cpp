/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <cassert>
#include <gtest/gtest.h>
#include "object_analytics_nodelet/model/object2d.h"
#include "tests/unittest_util.h"

TEST(UnitTestObject2D, getRoi)
{
  ObjectInBox oib = getObjectInBox(0, 0, 100, 100, "table", 0.99);
  Object2D obj(oib);
  RegionOfInterest left = obj.getRoi();
  RegionOfInterest right = getRoi(0, 0, 100, 100);
  EXPECT_TRUE(left == right);
}

TEST(UnitTestObject2D, getObject)
{
  ObjectInBox oib = getObjectInBox(0, 0, 100, 100, "table", 0.99);
  Object2D obj(oib);
  Object left = obj.getObject();
  Object right = getObject("table", 0.99);
  EXPECT_TRUE(left == right);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
