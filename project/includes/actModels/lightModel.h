#ifndef LightModel
#define LightModel

#include <Color.h>

namespace LightModel {

	/** \brief Initial colors */
	static std::vector<Color> initColors = {amiro::Color(amiro::Color::RED),
						amiro::Color(amiro::Color::GREEN),
						amiro::Color(amiro::Color::BLUE),
						amiro::Color(amiro::Color::WHITE),
						amiro::Color(amiro::Color::RED),
						amiro::Color(amiro::Color::GREEN),
						amiro::Color(amiro::Color::BLUE),
						amiro::Color(amiro::Color::WHITE)};

	/** \brief Type of lighting of the LEDs
	  * SINGLE_INIT: Sets the initial colors (see vector "initColors").
	  * SINGLE_SHINE: Sets the given color(s) and lets it just shine.
	  * SINGLE_BLINK: Sets the given color(s) and blinks with all LEDs turned on or off.
	  * SINGLE_WARNING: Sets the given color(s) and blinks with always the left or right half turned on or off.
	  * SINGLE_CROSSED: Sets the given color(s) and blinks with only 4 crossed LEDs at once turned on.
	  * SINGLE_CIRCLELEFT: Sets the given color(s) and lets circle two LEDs turned on to the left (counter clockwise).
	  * SINGLE_CIRCLERIGHT: Sets the given color(s) and lets circle two LEDs turned on to the right (clockwise).
	  * CHANGE_INIT: Changes the colors without turning off the LEDs. The color set equals the initial colors (see vector "initColors").
	  * CHANGE_SHINE: Changes the given colors without turning off the LEDs.
	  * CHANGE_BLINK: Changes the given colors and blinks with all LEDs turned on or off.
	  */
	enum LightType {SINGLE_INIT,
			SINGLE_SHINE,
			SINGLE_BLINK,
			SINGLE_WARNING,
			SINGLE_CROSSED,
			SINGLE_CIRCLELEFT,
			SINGLE_CIRCLERIGHT,
			CHANGE_INIT,
			CHANGE_SHINE,
			CHANGE_BLINK};

	/** \brief String array for lighting type names (order equals the lighting type enumeration) */
	static const std::string LightTypeName[] = {	"INIT (single color)", // SINGLE_INIT
							"SHINE (single color)", // SINGLE_SHINE
							"BLINK (single color)", // SINGLE_BLINK
							"WARNING (single color)", // SINGLE_WARNING
							"CROSSED (single color)", // SINGLE_CROSSED
							"CIRCLE LEFT (single color)", // SINGLE_CIRCLELEFT
							"CIRCLE RIGHT (single color)", // SINGLE_CIRCLERIGHT
							"INIT (changing colors)", // CHANGE_INIT
							"SHINE (changing colors)", // CHANGE_SHINE
							"BLINK (changing colors)"}; // CHANGE_BLINK

	/** \brief Minimal period time for each lighting type (order equals the lighting type enumeration) */
	static const int LightTypeMinPeriodTime[] = {	0, // SINGLE_INIT
							0, // SINGLE_SHINE
							200, // SINGLE_BLINK
							400, // SINGLE_WARNING
							400, // SINGLE_CROSSED
							800, // SINGLE_CIRCLELEFT
							800, // SINGLE_CIRCLERIGHT
							100, // CHANGE_INIT
							100, // CHANGE_SHINE
							200}; // CHANGE_BLINK

	static const int LightTypeCount = 10;


	/** \brief Check, if the lighting type is known. */
	bool lightTypeIsKnown(int lightType) {
		return lightType >= 0 && lightType < LightTypeCount;
	}

	/** \brief Checks, if the given lighting type is a lighting type as CHANGE. */
	bool lightTypeIsChange(int lightType) {
		return lightType >= LightType::CHANGE_INIT;
	}

	/** \brief Checks, if the given lighting type is a lighting type with initial colors. */
	bool lightTypeIsInit(int lightType) {
		return lightType == LightType::SINGLE_INIT || lightType == LightType::CHANGE_INIT;
	}

	/** \brief Creates integer vector containing lighting type, period time and only one color for the setLights tool. */
	std::vector<int> setLight2Vec(int lightingType, amiro::Color color, int periodTime) {
		std::vector<int> commandVector(5,0);
		commandVector[0] = lightingType;
		commandVector[1] = (int)(color.getRed());
		commandVector[2] = (int)(color.getGreen());
		commandVector[3] = (int)(color.getBlue());
		commandVector[4] = periodTime;
		return commandVector;
	}

	/** \brief Creates integer vector containing lighting type, period time and colors (color count depends on lighting type) for the setLights tool (if there is a problem, there is only one entry with -1). */
	std::vector<int> setLights2Vec(int lightingType, std::vector<amiro::Color> colors, int periodTime) {
		std::vector<int> commandVector;
		if ((!lightTypeIsChange(lightingType) && (colors.size() == 8 || colors.size() == 1))
		 || (lightTypeIsChange(lightingType) && colors.size() > 0)
		 || (lightTypeIsInit(lightingType))
		) {
			commandVector.push_back(lightingType);
			for (uint led=0; led<colors.size(); led++) {
				commandVector.push_back((int)(colors[led].getRed()));
				commandVector.push_back((int)(colors[led].getGreen()));
				commandVector.push_back((int)(colors[led].getBlue()));
			}
			commandVector.push_back(periodTime);
		} else {
			commandVector.push_back(-1);
		}
		return commandVector;
	}
}

#endif // LightModel
