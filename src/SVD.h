#pragma once

#include "global.h"

/// struct for SVD decomposition results
struct SVDResult {
  SVDResult(Mat3f U, Mat3f Sigma, Mat3f V) : U(U), Sigma(Sigma), V(V) {}
  Mat3f U, Sigma, V;
};

/**
 * Calculate the result for the SVD decomposition
 * @param m the matrix to decomposite
 * @return SVDResult
 */
SVDResult SVDDecompose(const Mat3f &m);