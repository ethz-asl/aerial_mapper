/* signum function */
int sgn(int x) { return (x > 0) ? 1 : (x < 0) ? -1 : 0; }

void gbham(int xstart, int ystart, int xend, int yend)
/*--------------------------------------------------------------
 * Bresenham-Algorithmus: Linien auf Rastergeräten zeichnen
 *
 * Eingabeparameter:
 *    int xstart, ystart        = Koordinaten des Startpunkts
 *    int xend, yend            = Koordinaten des Endpunkts
 *
 * Ausgabe:
 *    void SetPixel(int x, int y) setze ein Pixel in der Grafik
 *         (wird in dieser oder aehnlicher Form vorausgesetzt)
 *---------------------------------------------------------------
 */
{
  int x, y, t, dx, dy, incx, incy, pdx, pdy, ddx, ddy, es, el, err;

  /* Entfernung in beiden Dimensionen berechnen */
  dx = xend - xstart;
  dy = yend - ystart;

  /* Vorzeichen des Inkrements bestimmen */
  incx = sgn(dx);
  incy = sgn(dy);
  if (dx < 0) dx = -dx;
  if (dy < 0) dy = -dy;

  /* feststellen, welche Entfernung größer ist */
  if (dx > dy) {
    /* x ist schnelle Richtung */
    pdx = incx;
    pdy = 0; /* pd. ist Parallelschritt */
    ddx = incx;
    ddy = incy; /* dd. ist Diagonalschritt */
    es = dy;
    el = dx; /* Fehlerschritte schnell, langsam */
  } else {
    /* y ist schnelle Richtung */
    pdx = 0;
    pdy = incy; /* pd. ist Parallelschritt */
    ddx = incx;
    ddy = incy; /* dd. ist Diagonalschritt */
    es = dx;
    el = dy; /* Fehlerschritte schnell, langsam */
  }

  /* Initialisierungen vor Schleifenbeginn */
  x = xstart;
  y = ystart;
  err = el / 2;
  std::cout << "x,y " << x << ", " << y << std::endl;

  /* Pixel berechnen */
  for (t = 0; t < el; ++t) /* t zaehlt die Pixel, el ist auch Anzahl */
  {
    /* Aktualisierung Fehlerterm */
    err -= es;
    if (err < 0) {
      /* Fehlerterm wieder positiv (>=0) machen */
      err += el;
      /* Schritt in langsame Richtung, Diagonalschritt */
      x += ddx;
      y += ddy;
    } else {
      /* Schritt in schnelle Richtung, Parallelschritt */
      x += pdx;
      y += pdy;
    }
    std::cout << "x,y " << x << ", " << y << std::endl;
  }
} /* gbham() */

void OrthomosaicBackprojection::Bresenham(int x1, int y1, int const x2,
                                          int const y2) {
#ifdef adsf
  int delta_x(x2 - x1);
  // if x1 == x2, then it does not matter what we set here
  signed char const ix((delta_x > 0) - (delta_x < 0));
  delta_x = std::abs(delta_x) << 1;

  int delta_y(y2 - y1);
  // if y1 == y2, then it does not matter what we set here
  signed char const iy((delta_y > 0) - (delta_y < 0));
  delta_y = std::abs(delta_y) << 1;

  plot(x1, y1);

  if (delta_x >= delta_y) {
    // error may go below zero
    int error(delta_y - (delta_x >> 1));

    while (x1 != x2) {
      if ((error >= 0) && (error || (ix > 0))) {
        error -= delta_x;
        y1 += iy;
      }
      // else do nothing

      error += delta_y;
      x1 += ix;

      plot(x1, y1);
    }
  } else {
    // error may go below zero
    int error(delta_x - (delta_y >> 1));

    while (y1 != y2) {
      if ((error >= 0) && (error || (iy > 0))) {
        error -= delta_y;
        x1 += ix;
      }
      // else do nothing

      error += delta_x;
      y1 += iy;

      plot(x1, y1);
    }
  }
#endif
