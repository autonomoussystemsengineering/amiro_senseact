#ifndef VCNL4020Models
#define VCNL4020Models

#include <math.h>

namespace VCNL4020Models {

	/** \brief Alpha constant of the obstacle model */
	static const float VCNL4020MODELS_OBSTACLEMODEL_ALPHA = 0.942693757414292;
	/** \brief Beta constant of the obstacle model */
	static const float VCNL4020MODELS_OBSTACLEMODEL_BETA = -16.252241893638708;
	/** \brief Delta constant of the obstacle model */
	static const float VCNL4020MODELS_OBSTACLEMODEL_DELTA = 0.0;
	/** \brief Xi constant of the obstacle model */
	static const float VCNL4020MODELS_OBSTACLEMODEL_XI = 1.236518540376969;
	/** \brief Measurement variance of the obstacle model */
	static const float VCNL4020MODELS_OBSTACLEMODEL_MEAS_VARIANCE = 20.886268537074187;

	/** \brief Gradient m of the edge model */
	static const float VCNL4020MODELS_EDGEMODEL_M = 6.359946153158588;
	/** \brief Axis intercept b of the edge model */
	static const float VCNL4020MODELS_EDGEMODEL_B = 0.401918238192352;

	/** \brief Calculates the distance [m] to an obstacle, given the obstacle's angle [rad] relative to the sensor's normal and the sensor value [ticks] */
	static float obstacleModel(float angle, float sensorValue) {
		float cosxi = cos(VCNL4020MODELS_OBSTACLEMODEL_XI * angle);
		if (cosxi < 0) {
			cosxi *= -1;
		}
		float divi = sensorValue - VCNL4020MODELS_OBSTACLEMODEL_BETA;
		if (divi <= 0) {
			divi = 1;
		}
		return sqrt(VCNL4020MODELS_OBSTACLEMODEL_ALPHA * cosxi / divi + VCNL4020MODELS_OBSTACLEMODEL_DELTA * cosxi);
	}

	/** \brief Calculates the standard deviation [m] of the calculated distance of the obstacle model, given the calculated distance [m] to the obstacle and the obstacle's angle [rad] relative to the sensor's normal */
	static float obstacleErrorModel(float dist, float angle) {
		float sig = sqrt(VCNL4020MODELS_OBSTACLEMODEL_MEAS_VARIANCE);
		float cosxi = cos(VCNL4020MODELS_OBSTACLEMODEL_XI * angle);
		if (cosxi < 0) {
			cosxi *= -1;
		}
		float diffdistdelta = dist * dist / cosxi - VCNL4020MODELS_OBSTACLEMODEL_DELTA;
		float error = (diffdistdelta * diffdistdelta) / (2 * dist * VCNL4020MODELS_OBSTACLEMODEL_ALPHA * sqrt(1 / cosxi)) * sig;
		return error;
	}

	/** \brief Calculates the distance [m] to the next edge, given the normalized edge value factored by 10000 (as integer) */
	static float edgeModel(int senValue) {
		return (VCNL4020MODELS_EDGEMODEL_M * ((float)senValue)/10000.0 + VCNL4020MODELS_EDGEMODEL_B) / 100;
	}
}

#endif // VCNL4020Models
