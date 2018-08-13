#include "gtest/gtest.h"
#include "../src/EntryPoint.h"

TEST(KalmanFilter, ableToComplieTest) {
  int resutl1 = entryPoint2D();
  EXPECT_EQ(resutl1, 1);
};

TEST(KalmanFilter, twoDimentionalProcessFromFileTest) {
  int resutl1 = processFile("/Users/james/git/Self-Driving-Car-Extended-Kalman-Filter-2D/tests/obj_pose-laser-radar-synthetic-input.txt");
  EXPECT_EQ(resutl1, 0);
};

TEST(KalmanFilter, jacobianMatrixTest) {
  jacobianMatrix();
};

TEST(KalmanFilter, rmseTest) {
  calcRMSE();
};