#include "map-io.h"

/**
 * @brief saveMapAsPGM Saves a given map as a 16bit PGM.
 * @param map pointer to tinySLAM map struct.
 * @param path path in the file system where the image should be saved.
 */
void saveMapAsPGM(ts_map_t *map, std::string path) {
    std::ofstream file(path);

    // Magic Number for Portable Graymap in ASCII
    file << "P2" << std::endl;

    // Dimensions of the PGM
    file << map->size << " " << map->size << std::endl;

    // Image depth
    file << TS_NO_OBSTACLE << std::endl;

    // Actual image data
    int x, y;
    ts_map_pixel_t *ptr;
    for (ptr = map->map, y = 0; y < map->size; y++) {
        for (x = 0; x < map->size; x++, ptr++) {
            file << *ptr << " ";
        }
        file << std::endl;
    }

    file.close();
}

/**
 * @brief loadMapFromPGM loads a 16bit grayscale PGM and converts it into a tinySLAM map struct. This function can be used to load PGMs created with saveMapAsPGM(). In contrast to loadMapFromImage() PGM are loaded with 16bit depth.
 * @param map pointer to a tinySLAM map struct.
 * @param path path in the file system to the PGM file.
 * @return true on success, otherwise false
 */
bool loadMapFromPGM(ts_map_t *map, std::string path) {
    // read image from file
    cv::Mat image;
    if (!path.empty()) {
      image = cv::imread(path, CV_LOAD_IMAGE_UNCHANGED); // load unchanged to keep 16bit
    } else {
      ERROR_MSG("Empty path to map");
      return false;
    }

    // check if loading image failed
    if (!image.data) {
        ERROR_MSG("Could not open file: " << path);
        return false;
    }

    // check if image is square
    if (image.cols != image.rows) {
        ERROR_MSG("Height of image must be equal to width! Maybe this PGM was not created with CoreSLAM? Use loadMapFromImage instead.");
        return false;
    }

    // initalize struct
    map->size = image.cols;
    map->map = (ts_map_pixel_t *) malloc(sizeof(ts_map_pixel_t) * map->size * map->size);

    // copy image into map structure
    int x, y;
    ts_map_pixel_t *ptr = map->map;
    for (y = 0; y < image.rows; y++) {
        for (x = 0; x < image.cols; x++) {
            *ptr = image.at<uint16_t>(y,x);
            ptr++;
        }
    }

    return true;
}

/**
 * @brief loadMapFromImage loads a grayscale image using OpenCV and converts it into a tinySLAM map struct.
 * @param map pointer to a tinySLAM map struct.
 * @param path path in the file system to the image file.
 * @return true on success, otherwise false
 */
bool loadMapFromImage(ts_map_t *map, std::string path, bool flipHorizontal) {
    // read image from file
    cv::Mat image = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);

    // check if loading image failed
    if (!image.data) {
        ERROR_MSG("Could not open file: " << path);
        return false;
    }

    // maybe flip
    if (flipHorizontal) {
        cv::flip(image, image, 0); // 0 is OpenCV's magic flip code for flipping around x axis
    }

    // check if image is square
    if (image.cols != image.rows) {
        INFO_MSG("Loaded map image is not square. Padding it...");
    }

    // initalize struct
    map->size = std::max(image.cols, image.rows);
    map->map = (ts_map_pixel_t *) malloc(sizeof(ts_map_pixel_t) * map->size * map->size);

    // convert image to map
    int x, y;
    ts_map_pixel_t *ptr = map->map;
    for (y = 0; y < map->size; y++) {
        for (x = 0; x < map->size; x++) {
            if (x < image.cols && y < image.rows) {
                *ptr = image.at<uchar>(y,x) * (TS_NO_OBSTACLE / 255);
            } else {
                // mark as unknown
                *ptr = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2;
            }
            ptr++;
        }
    }

    return true;
}
