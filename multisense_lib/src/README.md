# LibMultiSense

LibMultiSense is a C++ library used to interface with the MultiSense S
family of sensors from Carnegie Robotics. For more information on the
various MultiSense products please visit
http://carnegierobotics.com/products/

### Installation

#### Linux

LibMultiSense uses CMake for its build system.

To build the standalone LibMultiSense library and demonstration applications.

    > git clone https://github.com/carnegierobotics/LibMultiSense.git
    > cd LibMultiSense
    > mkdir build
    > cd build && cmake -DCMAKE_INSTALL_PREFIX=../install ..
    > make

Integrating LibMultiSense into an existing CMake project is easy. Simply 
clone the LibMultiSense repository into the existing project's source tree.
 In the main CMakeLists.txt file of the project, add the following lines:

     include_directories(LibMultiSense/source/LibMultiSense)
     add_subdirectory(LibMultiSense/source/LibMultiSense)


#### Windows

LibMultiSense uses CMake to create a Microsoft Visual Studio project file used
to build the LibMultiSense DLL.

Download and install CMake on Windows (http://www.cmake.org/download/), making
sure CMake is included included in the system PATH.

Clone LibMultiSense to the Windows machine using a Windows Mercurial client.
TortiseHg works well for this application http://tortoisehg.bitbucket.org/

Open a command prompt and execute the following commands:

    > cd <LibMultiSense_checkout_directory>
    > mkdir build
    > cd build
    > cmake ..

This will create a LibMultiSense.sln Visual Studio Solution file in the build directory.
Open the solution file with Visual Studio (http://msdn.microsoft.com/en-us/vstudio/aa718325.aspx)
and build the Solution.


### Documentation and Examples

LibMultiSense builds as a single shared library which can be linked into
any existing project.

The two header files MultiSenseChannel.hh and MultiSenseTypes.hh contain
all the declarations necessary to interface with a MultiSense sensor.

Doxygen documentation can be built for LibMultisense by runnning the Doxygen
configuration file located in the docs directory

    > cd LibMultiSense/docs
    > doxygen Doxyfile

Html and LaTex documentation will be generated in the docs directory.

Usage examples are included in the Doxygen documentation.

### Support

Please direct all issues, questions, and feature requests to
support@carnegierobotics.com




