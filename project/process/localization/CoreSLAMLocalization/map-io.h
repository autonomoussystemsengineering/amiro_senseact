#ifndef MAPIO_H
#define MAPIO_H

#ifdef __cplusplus
extern "C"{
#endif
#include <libs/CoreSLAM/CoreSLAM.h>
#ifdef __cplusplus
}
#endif
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <MSG.h>

/**
 * @brief saveMapAsPGM Saves a given map as a 16bit PGM.
 * @param map pointer to tinySLAM map struct.
 * @param path path in the file system where the image should be saved.
 */
void saveMapAsPGM(ts_map_t *map, std::string path);

/**
 * @brief loadMapFromPGM loads a 16bit grayscale PGM and converts it into a tinySLAM map struct. This function can be used to load PGMs created with saveMapAsPGM(). In contrast to loadMapFromImage() PGM are loaded with 16bit depth.
 * @param map pointer to a tinySLAM map struct.
 * @param path path in the file system to the PGM file.
 * @return true on success, otherwise false
 */
bool loadMapFromPGM(ts_map_t *map, std::string path);
/**
 * @brief loadMapFromImage loads a grayscale image using OpenCV and converts it into a tinySLAM map struct.
 * @param map pointer to a tinySLAM map struct.
 * @param path path in the file system to the image file.
 * @return true on success, otherwise false
 */
bool loadMapFromImage(ts_map_t *map, std::string path, bool flipHorizontal);

#endif // MAPIO_H
