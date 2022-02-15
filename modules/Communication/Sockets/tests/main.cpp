/*
 * @Author: Pawel Ptasznik
 * @Copyright CERN 2017
 */

#include "gmock/gmock.h"

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}

