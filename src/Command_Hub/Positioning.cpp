#include <Arduino.h>
#include "Positioning.h"

static const int MAX_SOLVE_RX = 8;   // most receivers one solve can take

bool trilaterate(const float* xsIn, const float* ysIn, const float* dsIn, int n,
                 float& x, float& y, float& rms) {
  if (n < 1 || n > MAX_SOLVE_RX) return false;

  // ---- Sort nearest first: the nearest reading is the most accurate, so it's the reference ----
  float xs[MAX_SOLVE_RX], ys[MAX_SOLVE_RX], ds[MAX_SOLVE_RX], w[MAX_SOLVE_RX];
  for (int i = 0; i < n; i++) {
    int j = i;
    while (j > 0 && ds[j - 1] > dsIn[i]) { xs[j] = xs[j-1]; ys[j] = ys[j-1]; ds[j] = ds[j-1]; j--; }
    xs[j] = xsIn[i]; ys[j] = ysIn[i]; ds[j] = dsIn[i];
  }

  // ---- Weights: far readings are noisier, so trust them less ----
  for (int i = 0; i < n; i++) w[i] = 1.0f / (ds[i] * ds[i]);

  if (n == 1) {
    // Only one receiver: all we know is "near it"
    x = xs[0]; y = ys[0];

  } else if (n == 2) {
    // Two receivers: on the line between them, split by distance
    float f = ds[0] / (ds[0] + ds[1]);
    x = xs[0] + (xs[1] - xs[0]) * f;
    y = ys[0] + (ys[1] - ys[0]) * f;

  } else {
    // ---- Step 1: first guess ----
    // Subtract the nearest receiver's circle from the others: x² and y² cancel,
    // leaving straight-line equations we can solve directly (weighted least squares).
    float a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
    float k0 = xs[0] * xs[0] + ys[0] * ys[0];
    for (int i = 1; i < n; i++) {
      float ax = 2 * (xs[i] - xs[0]);
      float ay = 2 * (ys[i] - ys[0]);
      float b  = (xs[i] * xs[i] + ys[i] * ys[i] - k0) - (ds[i] * ds[i] - ds[0] * ds[0]);
      a11 += w[i] * ax * ax;  a12 += w[i] * ax * ay;  a22 += w[i] * ay * ay;
      b1  += w[i] * ax * b;   b2  += w[i] * ay * b;
    }
    float det = a11 * a22 - a12 * a12;
    if (det > 1e-3f * a11 * a22) {
      x = (a22 * b1 - a12 * b2) / det;
      y = (a11 * b2 - a12 * b1) / det;
    } else {
      // Receivers in a straight line: can't tell which side, use the weighted centre
      float sw = 0; x = y = 0;
      for (int i = 0; i < n; i++) { x += w[i] * xs[i]; y += w[i] * ys[i]; sw += w[i]; }
      x /= sw; y /= sw;
    }

    // ---- Step 2: polish (Gauss-Newton) ----
    // Each receiver pushes the point away if it's too close, pulls it in if too far.
    // Combine the pushes, take a step, repeat until it moves less than 1 mm.
    for (int it = 0; it < 8; it++) {
      float h11 = 0, h12 = 0, h22 = 0, g1 = 0, g2 = 0;
      for (int i = 0; i < n; i++) {
        float dx = x - xs[i], dy = y - ys[i];
        float r  = fmaxf(sqrtf(dx * dx + dy * dy), 1e-3f);   // current distance
        float jx = dx / r, jy = dy / r;                       // direction away from receiver
        float res = r - ds[i];                                // too far (+) / too close (-)
        h11 += w[i] * jx * jx;  h12 += w[i] * jx * jy;  h22 += w[i] * jy * jy;
        g1  += w[i] * jx * res; g2  += w[i] * jy * res;
      }
      float hd = h11 * h22 - h12 * h12;
      if (fabsf(hd) < 1e-9f) break;
      float sx = -(h22 * g1 - h12 * g2) / hd;
      float sy = -(h11 * g2 - h12 * g1) / hd;
      x += sx;  y += sy;
      if (sx * sx + sy * sy < 1e-6f) break;
    }
  }

  // ---- Residual: how well the answer fits every circle ----
  float sum = 0;
  for (int i = 0; i < n; i++) {
    float res = hypotf(x - xs[i], y - ys[i]) - ds[i];
    sum += res * res;
  }
  rms = sqrtf(sum / n);
  return true;
}