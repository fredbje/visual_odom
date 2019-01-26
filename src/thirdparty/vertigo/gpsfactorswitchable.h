#ifndef GPSFACTORSWITCHABLE_H
#define GPSFACTORSWITCHABLE_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include "switchVariableLinear.h"

namespace vertigo {

class GpsFactorSwitchableLinear : public gtsam::NoiseModelFactor2<gtsam::Pose3, SwitchVariableLinear>
    {
    public:
        GpsFactorSwitchableLinear();

        GpsFactorSwitchableLinear(gtsam::Key key1, gtsam::Key key2, const gtsam::Point3& measured, const gtsam::SharedNoiseModel& model)
            : gtsam::NoiseModelFactor2<gtsam::Pose3, SwitchVariableLinear>(model, key1, key2),
          gpsFactor(key1, measured, model) {}

        gtsam::Vector evaluateError(const gtsam::Pose3& p1, const SwitchVariableLinear& s,
            boost::optional<gtsam::Matrix&> H1 = boost::none,
            boost::optional<gtsam::Matrix&> H2 =  boost::none) const
          {

            // calculate error
            gtsam::Vector error = gpsFactor.evaluateError(p1, H1);
            error *= s.value();

            // handle derivatives
            if (H1) *H1 = *H1 * s.value();
            if (H2) *H2 = error;

            return error;
          };

private:
        gtsam::GPSFactor gpsFactor;
    };

}
#endif // GPSFACTORSWITCHABLE_H
