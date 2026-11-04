#pragma once

// 2D trilateration: weighted least squares + Gauss-Newton polish.
//   xs, ys = receiver positions (m), ds = distances (m), n = how many (1..8)
//   Outputs x, y (m) and rms = how much the receivers disagree (m). Small = trustworthy.
//   1 receiver  -> its own position
//   2 receivers -> a point on the line between them
//   3+          -> full trilateration
bool trilaterate(const float* xs, const float* ys, const float* ds, int n,
                 float& x, float& y, float& rms);