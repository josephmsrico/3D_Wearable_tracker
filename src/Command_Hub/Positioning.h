#pragma once

// 2D trilateration: weighted least squares + Gauss-Newton polish.
//   xs, ys = receiver positions (m), ds = distances (m), n = how many (1..8)
//   Outputs x, y (m) and rms = how much the receivers disagree (m). Small = trustworthy.
//   1 receiver  -> its own position
//   2 receivers -> a point on the line between them
//   3+          -> full trilateration
bool trilaterate(const float* xs, const float* ys, const float* ds, int n,
                 float& x, float& y, float& rms);

// 3D trilateration: same idea, but receivers have a height (zs) and the answer does too.
//   zHint     = the Tracker's height from its barometer (m, same frame as zs)
//   haveHint  = false if there's no barometer reading (z is then only loosely guessed)
//   Outputs x, y, z (m) and rms (m, measured in 3D).
//   Receivers mounted at the same height can't tell "above" from "below", so the
//   barometer is what really pins z down. 4+ receivers at different heights help RSSI
//   contribute to z as well.
bool trilaterate3D(const float* xs, const float* ys, const float* zs, const float* ds, int n,
                   float zHint, bool haveHint,
                   float& x, float& y, float& z, float& rms);