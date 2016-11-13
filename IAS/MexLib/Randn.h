
/*
 * Randn.h
 *
 *  Created on: Mar 29, 2014
 *      Author: rueckert
 *      Original (May 2014): http://phoxis.org/2013/05/04/generating-random-numbers-from-normal-distribution-in-c
 */

#ifndef RANDN_H_
#define RANDN_H_

double randn (double mu, double sigma) {
  double U1, U2, W, mult;
  static double X1, X2;
  static int call = 0;

  if (call == 1)
    {
      call = !call;
      return (mu + sigma * (double) X2);
    }

  do
    {
      U1 = -1 + ((double) rand () / RAND_MAX) * 2;
      U2 = -1 + ((double) rand () / RAND_MAX) * 2;
      W = pow (U1, 2) + pow (U2, 2);
    }
  while (W >= 1 || W == 0);

  mult = sqrt ((-2 * log (W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;

  call = !call;

  return (mu + sigma * (double) X1);
}


#endif /* RANDN_H_ */
