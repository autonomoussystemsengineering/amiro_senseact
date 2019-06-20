#ifndef TwbTracking
#define TwbTracking

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/threading/SynchronizedQueue.h>

// types
#include <types/enum.pb.h>
#include <types/loc.pb.h>
#include <types/pose.pb.h>
#include <types/rotation.pb.h>
#include <types/shapes.pb.h>
#include <types/vertex.pb.h>

namespace twbTrackingProcess {

	/** \brief Tracking Position, including
	  *  - x: x-coordinate of position (Float) [meter]
	  *  - y: y-coordinate of position (Float) [meter]
	  *  - theta: angle of position (Float) [rad]
	  */
	typedef struct {
		float x;
		float y;
		float theta;
	} TrackingPos;

	/** \brief Tracking Object, including
	  *  - id: Marker ID (Integer)
	  *  - pos: Tracking Position (TrackingPos)
	  */
	typedef struct {
		int id;
		TrackingPos pos;
	} TrackingObject;



	/** \brief Constant for tracking timeout in milliseconds */
	static const uint32_t TRACKING_TIMEOUT = 3000;

	/** \brief Constant for the translation of the distance to meter */
	static const float TRACKING_TO_METER = 1.0;

	/** \brief Constant for the translation of the angle to rad */
	static const float TRACKING_TO_RAD = M_PI/180.0;

	/** \brief Constant for angle normalization */
	static const float TRACKING_ANGLE_NORM = 2.0*M_PI;


	/** \brief Checks, if an Tracking Object is an error */
	static bool isErrorTracking(TrackingObject obj) {
		return obj.id < 0;
	}

	/** \brief Creates an tracking error object */
	static TrackingObject createErrorTracking() {
		TrackingObject obj;
		obj.id = -1;
		obj.pos.x = 0.0;
		obj.pos.y = 0.0;
		obj.pos.theta = 0.0;
		return obj;
	}

	/** \brief Returns the Tracking Object for the given marker ID in the given ObjectList */
	static TrackingObject readTracking(boost::shared_ptr<twbTracking::proto::ObjectList> data, int trackingMarkerID) {
		twbTracking::proto::Pose pose2D;
		bool found = false;
		for (int i = 0; i < data->object_size(); i++) {
			if (trackingMarkerID == data->object(i).id()) {
				pose2D = data->object(i).position();
				found = true;
				break;
			}
		}
		if (found) {
			TrackingObject obj;
			obj.id = trackingMarkerID;
			obj.pos.x = pose2D.translation().x()*TRACKING_TO_METER;
			obj.pos.y = pose2D.translation().y()*TRACKING_TO_METER;
			float angle = pose2D.rotation().z()*TRACKING_TO_RAD;
			angle = fmod(angle, TRACKING_ANGLE_NORM);
			if (angle < 0) angle += TRACKING_ANGLE_NORM;
			obj.pos.theta = angle;
			return obj;
		} else {
			return createErrorTracking();
		}
	}

	/** \brief Checks, if the tracking data contains the data of the given marker */
	static bool isBeingTracked(boost::shared_ptr<twbTracking::proto::ObjectList> data, int trackingMarkerID) {
		bool isIn = false;
		for (int i = 0; i < data->object_size(); i++) {
			if (trackingMarkerID == data->object(i).id()) {
				isIn = true;
				break;
			}
		}
		return isIn;
	}

	/** \brief Registers the ObjectList for RSB */
	static void registerTracking() {
		rsb::converter::ProtocolBufferConverter<twbTracking::proto::ObjectList>::Ptr converterObjectList(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::ObjectList>);
		rsb::converter::converterRepository<std::string>()->registerConverter(converterObjectList);
	}

	/** \brief Creates the participant RSB config for the tracking. */
	static rsb::ParticipantConfig getTrackingRSBConfig() {
		rsb::ParticipantConfig trackingPartConf = rsb::getFactory().getDefaultParticipantConfig(); {
			// disable socket transport
			rsc::runtime::Properties trackingPropSocket  = trackingPartConf.mutableTransport("socket").getOptions();
			trackingPropSocket["enabled"] = boost::any(std::string("0"));

			// Get the options for spread transport, because we want to change them
			rsc::runtime::Properties trackingPropSpread  = trackingPartConf.mutableTransport("spread").getOptions();

			// enable socket transport
			trackingPropSpread["enabled"] = boost::any(std::string("1"));

			// Change the config
			trackingPropSpread["host"] = boost::any(std::string("alpia.techfak.uni-bielefeld.de"));

			// Change the Port
			trackingPropSpread["port"] = boost::any(std::string("4803"));

			// Write the tranport properties back to the participant config
			trackingPartConf.mutableTransport("socket").setOptions(trackingPropSocket);
			trackingPartConf.mutableTransport("spread").setOptions(trackingPropSpread);
		}
		return trackingPartConf;
	}

	/** \brief Gives the tracking scope */
	static std::string getTrackingScope() {
		return "/tracking/merger";
	}

	/** \brief Waits with the given synchronized queue for new tracking data for the given marker. It returns an tracking error object, if the TRACKING_TIMEOUT has been reached. */
	static TrackingObject getNextTrackingObject(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>> trackingQueue, int trackingMarkerID) {
		boost::shared_ptr<twbTracking::proto::ObjectList> data;
		try {
			data = boost::static_pointer_cast<twbTracking::proto::ObjectList>(trackingQueue->pop(TRACKING_TIMEOUT));
			return readTracking(data, trackingMarkerID);
		} catch (rsc::threading::QueueEmptyException ex) {
			return createErrorTracking();
		}
	}

	/** \brief Waits with the given synchronized queue for any new tracking data. It returns an tracking error object in the vector, if the TRACKING_TIMEOUT has been reached. */
	static std::vector<TrackingObject> getNextTrackingObjects(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>> trackingQueue) {
		boost::shared_ptr<twbTracking::proto::ObjectList> data;
		std::vector<TrackingObject> positions;
		try {
			data = boost::static_pointer_cast<twbTracking::proto::ObjectList>(trackingQueue->pop(TRACKING_TIMEOUT));
			for (int i = 0; i < data->object_size(); i++) {
				TrackingObject tracking = readTracking(data, data->object(i).id());
				positions.push_back(tracking);
			}
		} catch (rsc::threading::QueueEmptyException ex) {
			positions.clear();
			TrackingObject obj = createErrorTracking();
			positions.push_back(obj);
		}
		return positions;
	}
}

#endif // TwbTracking


/* ***** !!!!! OLD TRACKING VERSIONS !!!!! *****


	\brief Waits with the given synchronized queue for new tracking data for the given marker. It returns an tracking error object, if the TRACKING_TIMEOUT has been reached.
	static TrackingObject getNextTrackingObject(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>> trackingQueue, int trackingMarkerID) {
		boost::shared_ptr<twbTracking::proto::ObjectList> data;
		int restTime = TRACKING_TIMEOUT; // ms
		do {
			if (!trackingQueue->empty()) {
				data = boost::static_pointer_cast<twbTracking::proto::ObjectList>(trackingQueue->pop());
				if (isBeingTracked(data, trackingMarkerID)) {
					break;
				}
			} else if (restTime <= 0) {
				return createErrorTracking();
			} else {
				// sleep for 10 ms
				usleep(10000);
				restTime -= 10;
			}
		} while (true);
		return readTracking(data, trackingMarkerID);
	}

	\brief Waits with the given synchronized queue for any new tracking data. It returns an tracking error object in the vector, if the TRACKING_TIMEOUT has been reached.
	static std::vector<TrackingObject> getNextTrackingObjects(boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<twbTracking::proto::ObjectList>>> trackingQueue) {
		boost::shared_ptr<twbTracking::proto::ObjectList> data;
		int restTime = TRACKING_TIMEOUT; // ms
		do {
			if (!trackingQueue->empty()) {
				data = boost::static_pointer_cast<twbTracking::proto::ObjectList>(trackingQueue->pop());
				break;
			} else if (restTime <= 0) {
				std::vector<TrackingObject> positions;
				TrackingObject obj = createErrorTracking();
				positions.push_back(obj);
				return positions;
			} else {
				// sleep for 10 ms
				usleep(10000);
				restTime -= 10;
			}
		} while (true);
		std::vector<TrackingObject> positions;
		for (int i = 0; i < data->object_size(); i++) {
			TrackingObject tracking = readTracking(data, data->object(i).id());
			positions.push_back(tracking);
		}
		return positions;
	}
*/
