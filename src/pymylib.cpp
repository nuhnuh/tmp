
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <mylib.h>

#include <Eigen/Dense>



namespace py = pybind11;


// helpers
namespace {

typedef py::array_t<double, py::array::c_style | py::array::forcecast> pyarray_d;

template <typename T>
py::array_t<T> py_array_from_data(const T *data, size_t shape0) {
  py::array_t<T> res({shape0});
  std::copy(data, data + shape0, res.mutable_data());
  return res;
}

template <typename T>
py::array_t<T> py_array_from_data(const T *data, size_t shape0, size_t shape1) {
  py::array_t<T> res({shape0, shape1});
  std::copy(data, data + shape0 * shape1, res.mutable_data());
  return res;
}

template <typename T>
py::array_t<T> py_array_from_vector(const std::vector<T> &v) {
  const T *data = v.size() ? &v[0] : NULL;
  return py_array_from_data(data, v.size());
}

}


namespace {

py::object py_to_cpp_adapter(pyarray_d &v) {
  std::cout << "[DBG] py_to_cpp_adapter()" << v << std::endl;
  // input
  std::cout << "  v:" << v << std::endl;
  std::cout << "  v.size():" << v.size() << std::endl;
  std::cout << "  v.ndim():" << v.ndim() << std::endl;
  if (v.ndim() > 0)
    std::cout << "  v.shape(0):" << v.shape(0) << std::endl;
  if (v.ndim() > 1)
    std::cout << "  v.shape(1):" << v.shape(1) << std::endl;
  if (v.ndim() > 2)
    std::cout << "  v.shape(2):" << v.shape(2) << std::endl;
  if (v.ndim() == 3 && v.size() > 0)
    std::cout << "  *v.data(0,0,0):" << *v.data(0,0,0) << std::endl;
  // output
  Eigen::Matrix<double, 3, 4, Eigen::RowMajor> t_row_major;
  t_row_major << 1., 2., 3., 4., 5., 6., 7., 8., 9., 8., 7., 6.;
  return py_array_from_data(t_row_major.data(), 3, 4);  // returns a numpy.array()
}

} // namespace



PYBIND11_MODULE(pymylib, m) {

  m.doc() = "pybind11 example plugin"; // optional module docstring

  m.def("getX", mylib::getX, "returns the value of mylib::x");
  m.def("toDouble", mylib::toDouble, "transforms an int into a double");
  m.def("get1ArrayOfIntegers", mylib::get1ArrayOfIntegers, "returns a 1-vector of integers");
  m.def("get2ArrayOfIntegers", mylib::get2ArrayOfIntegers, "returns a 2-vector of integers");
  m.def("py_to_cpp_adapter", py_to_cpp_adapter, "input: np.array; output: np.array");

} // PYBIND11_MODULE

