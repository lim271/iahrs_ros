#ifndef _GP_H_
#define _GP_H_
#include <math.h>
#include <vector>



namespace iahrs_ros
{

class GaussianProcess
{

public:

  GaussianProcess()
  {
    _x = {
      -2.81,
      -2.21,
      -1.53,
      -0.87,
       -0.2,
       0.58,
       1.62,
       2.66
    };
    _alpha = {
      -30.81798107,
       30.41501325,
      -19.61070601,
        9.09562676,
       -0.55722606,
        8.89265069,
       -4.72808793,
        4.24544208,
    };
    _magnitude = 0.182 * 0.182;
  }

  double predict(const double& xstar)
  {
    double ystar = xstar;
    std::vector<double>::const_iterator x_it = _x.begin();
    std::vector<double>::const_iterator alpha_it = _alpha.begin();
    for (x_it, alpha_it; x_it < _x.end(); x_it++, alpha_it++)
    {
      ystar += _magnitude * exp(cos(*x_it - xstar) - 1.0) * (*alpha_it);
    }

    return ystar;
  }

private:

  unsigned int _n;
  double _magnitude;
  double _noise;
  std::vector<double> _x;
  std::vector<double> _alpha;

};


}

#endif // _GP_H_