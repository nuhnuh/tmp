//

#include "mylib.h"



namespace mylib {

int getX() {
  return ++x;
}

double toDouble(int x) {
  return static_cast<double>(x);
}

std::vector<int> get1ArrayOfIntegers() {
  std::vector<int> vector {1, 2, 3, 4};
  return vector;
}

std::vector<std::vector<int>> get2ArrayOfIntegers() {
  std::vector<std::vector<int>> vector {{1, 11}, {2, 22}, {3, 33}, {4, 44}};
  return vector;
}

} // namespace mylib
