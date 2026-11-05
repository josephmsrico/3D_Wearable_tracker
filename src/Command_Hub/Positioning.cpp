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

// =====================================================================
// 3D trilateration
// =====================================================================

// How noisy each input is, used to balance RSSI against the barometer.
// RSSI distance error grows with distance: ~3 dB of noise at n = 2.5 is about ±30% of d.
// The barometer is good to about ±0.5 m over a few minutes.
static const float RANGE_REL_ERR = 0.30f;   // RSSI distance error as a fraction of d
static const float BARO_ERR_M    = 0.5f;    // barometer height error (m)

// Solve the 3x3 system A·s = b (Cramer's rule). Returns false if it has no unique answer.
static bool solve3(const float A[3][3], const float b[3], float s[3]) {
  float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
            - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
            + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (fabsf(det) < 1e-12f) return false;
  for (int c = 0; c < 3; c++) {
    float M[3][3];
    for (int r = 0; r < 3; r++)
      for (int k = 0; k < 3; k++) M[r][k] = (k == c) ? b[r] : A[r][k];
    s[c] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
          - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
          + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
  }
  return true;
}

// Total weighted error at a point: every sphere's miss², plus the barometer's miss²
static float cost3D(float x, float y, float z, const float* xs, const float* ys, const float* zs,
                    const float* ds, const float* w, int n, float zRef, float wz) {
  float c = 0;
  for (int i = 0; i < n; i++) {
    float dx = x - xs[i], dy = y - ys[i], dz = z - zs[i];
    float res = sqrtf(dx * dx + dy * dy + dz * dz) - ds[i];
    c += w[i] * res * res;
  }
  float rz = z - zRef;
  return c + wz * rz * rz;
}

bool trilaterate3D(const float* xs, const float* ys, const float* zs, const float* ds, int n,
                   float zHint, bool haveHint,
                   float& x, float& y, float& z, float& rms) {
  if (n < 1 || n > MAX_SOLVE_RX) return false;

  // ---- Where should z start, and how much do we trust that? ----
  float zRef, wz;
  if (haveHint) {
    zRef = zHint;
    wz   = (RANGE_REL_ERR * RANGE_REL_ERR) / (BARO_ERR_M * BARO_ERR_M);
  } else {
    // No barometer: guess "about receiver height" and only nudge gently
    zRef = 0;
    for (int i = 0; i < n; i++) zRef += zs[i];
    zRef /= n;
    wz = (n >= 4) ? 0.01f : 0.2f;
  }

  // ---- Step 1: first guess, using the 2D solver ----
  // A sphere seen at height zRef is a circle on the floor plan. Pythagoras turns each
  // slant distance into a flat (horizontal) one: flat² = slant² - heightGap².
  float dh[MAX_SOLVE_RX];
  for (int i = 0; i < n; i++) {
    float gap = zRef - zs[i];
    dh[i] = sqrtf(fmaxf(ds[i] * ds[i] - gap * gap, 0.09f));   // never below 0.3 m
  }
  float rms2;
  if (!trilaterate(xs, ys, dh, n, x, y, rms2)) return false;
  z = zRef;

  // ---- Step 2: polish in 3D (Levenberg-Marquardt = Gauss-Newton with a safety brake) ----
  // Same push/pull as before, but along x, y AND z, plus the barometer pulling z toward zRef.
  // If a step makes things worse we throw it away and try a smaller, more careful one.
  if (n >= 3) {
    float w[MAX_SOLVE_RX];
    for (int i = 0; i < n; i++) w[i] = 1.0f / (ds[i] * ds[i]);   // same weighting as 2D

    float lambda = 0.001f;
    float cost = cost3D(x, y, z, xs, ys, zs, ds, w, n, zRef, wz);
    for (int it = 0; it < 15; it++) {
      float H[3][3] = {{0}}, g[3] = {0};
      for (int i = 0; i < n; i++) {
        float dx = x - xs[i], dy = y - ys[i], dz = z - zs[i];
        float r  = fmaxf(sqrtf(dx * dx + dy * dy + dz * dz), 1e-3f);
        float J[3] = { dx / r, dy / r, dz / r };    // direction away from receiver
        float res  = r - ds[i];                     // too far (+) / too close (-)
        for (int a = 0; a < 3; a++) {
          g[a] += w[i] * J[a] * res;
          for (int b = 0; b < 3; b++) H[a][b] += w[i] * J[a] * J[b];
        }
      }
      H[2][2] += wz;                 // barometer only cares about z
      g[2]    += wz * (z - zRef);

      float A[3][3], bvec[3], s[3];
      for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) A[a][b] = H[a][b];
        A[a][a] += lambda * H[a][a] + 1e-6f;
        bvec[a] = -g[a];
      }
      if (!solve3(A, bvec, s)) { lambda *= 10; continue; }

      float nx = x + s[0], ny = y + s[1], nz = z + s[2];
      float nc = cost3D(nx, ny, nz, xs, ys, zs, ds, w, n, zRef, wz);
      if (nc <= cost) {                       // better: keep it, and be bolder next time
        x = nx; y = ny; z = nz; cost = nc;
        lambda = fmaxf(lambda * 0.3f, 1e-6f);
        if (s[0] * s[0] + s[1] * s[1] + s[2] * s[2] < 1e-6f) break;   // moved < 1 mm
      } else {                                // worse: undo, and be more careful
        lambda *= 10;
      }
    }
  }

  // ---- Residual: how well the answer fits every sphere ----
  float sum = 0;
  for (int i = 0; i < n; i++) {
    float dx = x - xs[i], dy = y - ys[i], dz = z - zs[i];
    float res = sqrtf(dx * dx + dy * dy + dz * dz) - ds[i];
    sum += res * res;
  }
  rms = sqrtf(sum / n);
  return true;
}