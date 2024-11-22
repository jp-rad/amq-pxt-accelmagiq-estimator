#include "pxt.h"
#include "lib/AccelMagiQLibQuaternionEstimator.h"

using namespace accelmagiqlib;

namespace accelmagiq
{

    static QuaternionEstimator *_pQuaternionEstimator = nullptr;
    QuaternionEstimator &instance()
    {
        if (nullptr == _pQuaternionEstimator)
            _pQuaternionEstimator = new QuaternionEstimator();
        return *_pQuaternionEstimator;
    }

    //%
    void setCoordinateSystem(int system)
    {
        instance().setCoordinateSystem(system);
    }

    //%
    void setLowPassFilterAlpha(TNumber alpha)
    {
        instance().setLowPassFilterAlpha(toDouble(alpha));
    }

    //%
    void estimate_()
    {
        instance().estimate();
    }

    //%
    TNumber getW_()
    {
        return fromDouble(instance().getW());
    }

    //%
    TNumber getX_()
    {
        return fromDouble(instance().getX());
    }

    //%
    TNumber getY_()
    {
        return fromDouble(instance().getY());
    }

    //%
    TNumber getZ_()
    {
        return fromDouble(instance().getZ());
    }

}
