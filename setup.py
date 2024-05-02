from distutils.core import setup, Extension
import numpy

module = Extension('vehiclemodel',
                   sources=['vehicle_model.cpp'],
                   include_dirs=[numpy.get_include(), '/usr/include', '/usr/local/include'],
                   libraries=['boost_python38', 'boost_numpy38'],  # Update as per your Boost version and availability
                   library_dirs=['/usr/lib', '/usr/local/lib'],
                   extra_compile_args=["-std=c++11"])

setup(name='VehicleModel',
      version='1.0',
      description='Vehicle Simulation Module',
      ext_modules=[module])
