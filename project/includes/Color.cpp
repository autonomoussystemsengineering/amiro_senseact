
#include <Color.h>

using namespace amiro;

Color::Color()
    : red(0),
      green(0),
      blue(0) {
}

Color::Color(const Color& color)
    : red(color.red),
      green(color.green),
      blue(color.blue) {
}

Color::Color(int r, int g, int b)
    : red(r),
      green(g),
      blue(b) {
}

Color::Color(GlobalColor color) {
  static const struct {
    unsigned char red;
    unsigned char green;
    unsigned char blue;
  } global_colors[] = {
      { 240, 248, 255 },
      { 250, 235, 215 },
      { 0, 255, 255 },
      { 127, 255, 212 },
      { 240, 255, 255 },
      { 245, 245, 220 },
      { 255, 228, 196 },
      { 0, 0, 0 },
      { 255, 235, 205 },
      { 0, 0, 255 },
      { 138, 43, 226 },
      { 165, 42, 42 },
      { 222, 184, 135 },
      { 95, 158, 160 },
      { 127, 255, 0 },
      { 210, 105, 30 },
      { 255, 127, 80 },
      { 100, 149, 237 },
      { 255, 248, 220 },
      { 220, 20, 60 },
      { 0, 255, 255 },
      { 0, 0, 139 },
      { 0, 139, 139 },
      { 184, 134, 11 },
      { 169, 169, 169 },
      { 0, 100, 0 },
      { 169, 169, 169 },
      { 189, 183, 107 },
      { 139, 0, 139 },
      { 85, 107, 47 },
      { 255, 140, 0 },
      { 153, 50, 204 },
      { 139, 0, 0 },
      { 233, 150, 122 },
      { 143, 188, 143 },
      { 72, 61, 139 },
      { 47, 79, 79 },
      { 47, 79, 79 },
      { 0, 206, 209 },
      { 148, 0, 211 },
      { 255, 20, 147 },
      { 0, 191, 255 },
      { 105, 105, 105 },
      { 105, 105, 105 },
      { 30, 144, 255 },
      { 178, 34, 34 },
      { 255, 250, 240 },
      { 34, 139, 34 },
      { 255, 0, 255 },
      { 220, 220, 220 },
      { 248, 248, 255 },
      { 255, 215, 0 },
      { 218, 165, 32 },
      { 128, 128, 128 },
      { 128, 128, 128 },
      { 0, 128, 0 },
      { 173, 255, 47 },
      { 240, 255, 240 },
      { 255, 105, 180 },
      { 205, 92, 92 },
      { 75, 0, 130 },
      { 255, 255, 240 },
      { 240, 230, 140 },
      { 230, 230, 250 },
      { 255, 240, 245 },
      { 124, 252, 0 },
      { 255, 250, 205 },
      { 173, 216, 230 },
      { 240, 128, 128 },
      { 224, 255, 255 },
      { 250, 250, 210 },
      { 211, 211, 211 },
      { 144, 238, 144 },
      { 211, 211, 211 },
      { 255, 182, 193 },
      { 255, 160, 122 },
      { 32, 178, 170 },
      { 135, 206, 250 },
      { 119, 136, 153 },
      { 119, 136, 153 },
      { 176, 196, 222 },
      { 255, 255, 224 },
      { 0, 255, 0 },
      { 50, 205, 50 },
      { 250, 240, 230 },
      { 255, 0, 255 },
      { 128, 0, 0 },
      { 102, 205, 170 },
      { 0, 0, 205 },
      { 186, 85, 211 },
      { 147, 112, 219 },
      { 60, 179, 113 },
      { 123, 104, 238 },
      { 0, 250, 154 },
      { 72, 209, 204 },
      { 199, 21, 133 },
      { 25, 25, 112 },
      { 245, 255, 250 },
      { 255, 228, 225 },
      { 255, 228, 181 },
      { 255, 222, 173 },
      { 0, 0, 128 },
      { 253, 245, 230 },
      { 128, 128, 0 },
      { 107, 142, 35 },
      { 255, 165, 0 },
      { 255, 69, 0 },
      { 218, 112, 214 },
      { 238, 232, 170 },
      { 152, 251, 152 },
      { 175, 238, 238 },
      { 219, 112, 147 },
      { 255, 239, 213 },
      { 255, 218, 185 },
      { 205, 133, 63 },
      { 255, 192, 203 },
      { 221, 160, 221 },
      { 176, 224, 230 },
      { 128, 0, 128 },
      { 255, 0, 0 },
      { 188, 143, 143 },
      { 65, 105, 225 },
      { 139, 69, 19 },
      { 250, 128, 114 },
      { 244, 164, 96 },
      { 46, 139, 87 },
      { 255, 245, 238 },
      { 160, 82, 45 },
      { 192, 192, 192 },
      { 135, 206, 235 },
      { 106, 90, 205 },
      { 112, 128, 144 },
      { 112, 128, 144 },
      { 255, 250, 250 },
      { 0, 255, 127 },
      { 70, 130, 180 },
      { 210, 180, 140 },
      { 0, 128, 128 },
      { 216, 191, 216 },
      { 255, 99, 71 },
      { 64, 224, 208 },
      { 238, 130, 238 },
      { 245, 222, 179 },
      { 255, 255, 255 },
      { 245, 245, 245 },
      { 255, 255, 0 },
      { 154, 205, 50 }, };
  this->setRedGreenBlue(global_colors[color].red, global_colors[color].green,
                        global_colors[color].blue);
}

Color Color::darker(int factor) {
  if (factor <= 0)
    return *this;
  else if (factor < 100)
    return this->lighter(10000 / factor);

  int red = 100 * this->red / factor;
  int green = 100 * this->green / factor;
  int blue = 100 * this->blue / factor;

  return Color(red, green, blue);
}

void Color::getRedGreenBlue(int* r, int* g, int* b) {
  *r = this->red;
  *g = this->green;
  *b = this->blue;
}

unsigned char Color::getRed() {
  return this->red;
}

unsigned char Color::getGreen() {
  return this->green;
}

unsigned char Color::getBlue() {
  return this->blue;
}

Color Color::lighter(int factor) {
  if (factor <= 0)
    return *this;
  else if (factor < 100)
    return this->darker(10000 / factor);

  int red = factor * this->red / 100;
  int green = factor * this->green / 100;
  int blue = factor * this->blue / 100;

  red = (red < 0) ? 0 : (red > 255) ? 255 : red;
  green = (green < 0) ? 0 : (green > 255) ? 255 : green;
  blue = (blue < 0) ? 0 : (blue > 255) ? 255 : blue;

  return Color(red, green, blue);
}

void Color::setRedGreenBlue(int red, int green, int blue) {
  this->red = red;
  this->green = green;
  this->blue = blue;
}

Color& Color::operator=(Color const& color) {
  if (this != &color) {
    this->red = color.red;
    this->green = color.green;
    this->blue = color.blue;
  }
  return *this;
}

Color& Color::operator=(GlobalColor color) {
  return operator=(Color(color));
}
