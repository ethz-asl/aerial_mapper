/*
 *    Filename: utils-color-palette.h
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef UTILS_COLOR_PALETTE_H_
#define UTILS_COLOR_PALETTE_H_

#define DEG2RAD 0.01745329

// SYSTEM
#include <math.h>
#include <vector>

struct color {
  unsigned char rgbBlue;
  unsigned char rgbGreen;
  unsigned char rgbRed;
  color() { rgbBlue = rgbGreen = rgbRed = 0; }
};

struct palette {
  enum palettetypes {
    Linear_red_palettes,
    GammaLog_red_palettes,
    Inversion_red_palette,
    Linear_palettes,
    GammaLog_palettes,
    Inversion_palette,
    False_color_palette1,
    False_color_palette2,
    False_color_palette3,
    False_color_palette4
  };
  color colors[256];
};

static palette GetPalette(palette::palettetypes pal) {
  palette ret;

  int i, r, g, b;
  float f;

  switch (pal) {
    case palette::Linear_red_palettes:
      /*
       * Linear red palettes.
       */
      for (i = 0; i < 256; i++) {
        ret.colors[i].rgbBlue = 0;
        ret.colors[i].rgbGreen = 0;
        ret.colors[i].rgbRed = i;
      }
      break;
    case palette::GammaLog_red_palettes:
      /*
       * GammaLog red palettes.
       */
      for (i = 0; i < 256; i++) {
        f = log10(pow((i / 255.0), 1.0) * 9.0 + 1.0) * 255.0;
        ret.colors[i].rgbBlue = 0;
        ret.colors[i].rgbGreen = 0;
        ret.colors[i].rgbRed = f;
      }
      break;
    case palette::Inversion_red_palette:
      /*
       * Inversion red palette.
       */
      for (i = 0; i < 256; i++) {
        ret.colors[i].rgbBlue = 0;
        ret.colors[i].rgbGreen = 0;
        ret.colors[i].rgbRed = 255 - i;
      }
      break;
    case palette::Linear_palettes:
      /*
       * Linear palettes.
       */
      for (i = 0; i < 256; i++) {
        ret.colors[i].rgbBlue = ret.colors[i].rgbGreen = ret.colors[i].rgbRed =
            i;
      }
      break;
    case palette::GammaLog_palettes:
      /*
       * GammaLog palettes.
       */
      for (i = 0; i < 256; i++) {
        f = log10(pow((i / 255.0), 1.0) * 9.0 + 1.0) * 255.0;
        ret.colors[i].rgbBlue = ret.colors[i].rgbGreen = ret.colors[i].rgbRed =
            f;
      }
      break;
    case palette::Inversion_palette:
      /*
       * Inversion palette.
       */
      for (i = 0; i < 256; i++) {
        ret.colors[i].rgbBlue =
            ret.colors[i].rgbGreen =
            ret.colors[i].rgbRed =
            255 - i;
      }
      break;
    case palette::False_color_palette1:
      /*
       * False color palette #1.
       */
      for (i = 0; i < 256; i++) {
        r = (sin((i / 255.0 * 360.0 - 120.0 > 0 ? i / 255.0 * 360.0 - 120.0
                                                : 0) *
                 DEG2RAD) *
                 0.5 +
             0.5) *
            255.0;
        g = (sin((i / 255.0 * 360.0 + 60.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        b = (sin((i / 255.0 * 360.0 + 140.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        ret.colors[i].rgbBlue = b;
        ret.colors[i].rgbGreen = g;
        ret.colors[i].rgbRed = r;
      }
      break;
    case palette::False_color_palette2:
      /*
       * False color palette #2.
       */
      for (i = 0; i < 256; i++) {
        r = (sin((i / 255.0 * 360.0 + 120.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        g = (sin((i / 255.0 * 360.0 + 240.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        b = (sin((i / 255.0 * 360.0 + 0.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        ret.colors[i].rgbBlue = b;
        ret.colors[i].rgbGreen = g;
        ret.colors[i].rgbRed = r;
      }
      break;
    case palette::False_color_palette3:
      /*
       * False color palette #3.
       */
      for (i = 0; i < 256; i++) {
        r = (sin((i / 255.0 * 360.0 + 240.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        g = (sin((i / 255.0 * 360.0 + 0.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        b = (sin((i / 255.0 * 360.0 + 120.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        ret.colors[i].rgbBlue = b;
        ret.colors[i].rgbGreen = g;
        ret.colors[i].rgbRed = r;
      }
      break;

    case palette::False_color_palette4:
      /*
       * False color palette #4. Matlab JET
       */

      enum { nsep = 64, nvals = 192, n = 256 };

      std::vector<double> vals;
      vals.resize(nvals, 0);

      int idx = 0;
      for (int i = 0; i < nsep; ++i) {
        vals.at(idx++) = (i / (double)nsep);
      }

      for (int i = 0; i < nsep; ++i) {
        vals.at(idx + i) = 1.;
      }

      idx += nsep;
      for (int i = nsep - 1; i >= 0; --i) {
        vals.at(idx++) = i / (double)nsep;
      }

      std::vector<int> r;
      r.resize(nvals);
      std::vector<int> g;
      g.resize(nvals);
      std::vector<int> b;
      b.resize(nvals);
      for (std::size_t i = 0; i < nvals; ++i) {
        g.at(i) = ceil(nsep / 2) - 1 + i;
        r.at(i) = g.at(i) + nsep;
        b.at(i) = g.at(i) - nsep;
      }

      int idxr = 0;
      int idxg = 0;

      for (int i = 0; i < nvals; ++i) {
        if (r.at(i) >= 0 && r.at(i) < n)
          ret.colors[r.at(i)].rgbRed = vals.at(idxr++) * 255.;

        if (g.at(i) >= 0 && g.at(i) < n)
          ret.colors[g.at(i)].rgbGreen = vals.at(idxg++) * 255.;
      }

      int idxb = 0;
      int cntblue = 0;
      for (int i = 0; i < nvals; ++i) {
        if (b.at(i) >= 0 && b.at(i) < n) cntblue++;
      }

      for (int i = 0; i < nvals; ++i) {
        if (b.at(i) >= 0 && b.at(i) < n)
          ret.colors[b.at(i)].rgbBlue =
              vals.at(nvals - 1 - cntblue + idxb++) * 255.;
      }
      break;
  }
  return ret;
}

#endif // UTILS_COLOR_PALETTE_H_
