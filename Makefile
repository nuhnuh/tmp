#


#default: demo
default: docker-run-ubuntu20


################################################################################

BUILD_DIR := /tmp/snippets-pybind11/build



################################################################################


cmake:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake $(MKFILE_DIR) \
			-DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
			-DCMAKE_BUILD_TYPE=Debug \
			-DCMAKE_C_COMPILER_LAUNCHER="ccache" -DCMAKE_CXX_COMPILER_LAUNCHER="ccache" \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=1

build: cmake
	@if [ ! -d $(BUILD_DIR) ]; then \
		echo "Run first 'make cmake'"; \
		exit 1; \
	fi
	cd $(BUILD_DIR) && cmake --build . -- -j$(shell nproc)

demo: build
	@#$(BUILD_DIR)/dbg
	PYTHONPATH=$(BUILD_DIR) python3 -c "import pymylib; print('pymylib.getX():', pymylib.getX())"
	PYTHONPATH=$(BUILD_DIR) python3 -c "import pymylib; print('pymylib.toDouble(33):', pymylib.toDouble(33))"
	PYTHONPATH=$(BUILD_DIR) python3 -c "import pymylib; print('pymylib.get1ArrayOfIntegers():', pymylib.get1ArrayOfIntegers())"
	PYTHONPATH=$(BUILD_DIR) python3 -c "import pymylib; print('type(pymylib.get1ArrayOfIntegers())', type(pymylib.get1ArrayOfIntegers()))"
	PYTHONPATH=$(BUILD_DIR) python3 -c "import pymylib; print('pymylib.get2ArrayOfIntegers():', pymylib.get2ArrayOfIntegers())"
	PYTHONPATH=$(BUILD_DIR) python3 -c "import pymylib; import numpy; print('pymylib.py_to_cpp_adapter([]):', pymylib.py_to_cpp_adapter([]))"
	PYTHONPATH=$(BUILD_DIR) python3 -c "import pymylib; import numpy; print('pymylib.py_to_cpp_adapter([[[1, 11], [2, 22]]]):', pymylib.py_to_cpp_adapter([[[1, 11], [2, 22]]]))"


clean:
	rm -rf $(BUILD_DIR)


################################################################################
# docker

docker-create_image-ubuntu20:
	docker build -t manu/ubuntu20:sudo -f Dockerfile.ubuntu20 .

docker-run-ubuntu20:
	docker run -ti --rm \
	   -e DISPLAY=$(DISPLAY) -v /tmp/.X11-unix:/tmp/.X11-unix \
		 -v $(HOME):/SHARED:ro \
		 -v $(MKFILE_DIR):/home/ubuntu20/snippets-pybind11 \
	   --name ubuntu20_wsudo \
		 manu/ubuntu20:sudo bash

docker-connect-ubuntu20:
	docker exec -ti ubuntu20_wsudo bash

docker-delete-ubuntu20:
	docker rmi manu/ubuntu20:sudo


################################################################################
# utils

# asigned at runtime
MKFILE_DIR = $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
## asigned when declared
#MKFILE_DIR := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
