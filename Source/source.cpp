/*
 ==============================================================================
 
 source.cpp
 Created: 18 Feb 2017 6:37:01pm
 Author:  tollie
 
 ==============================================================================
 */

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "PluginHost.h"

//==========================================================================
// Converts a C++ vector to a Python list. All following functions
// are essentially cheap ripoffs from this one.
// https://gist.github.com/octavifs/5362272
template <class T>
boost::python::list vectorToList (std::vector<T> vector)
{
    typename std::vector<T>::iterator iter;
    boost::python::list list;
    for (iter = vector.begin(); iter != vector.end(); ++iter)
    {
        list.append(*iter);
    }
    return list;
}

//==========================================================================
// Yeah this is lazy. I know.
template <class T>
boost::python::list arrayToList (std::array<T, 13> array)
{
    typename std::array<T, 13>::iterator iter;
    boost::python::list list;
    for (iter = array.begin(); iter != array.end(); ++iter)
    {
        list.append(*iter);
    }
    return list;
}

//==========================================================================
// Converts a std::pair which is used as a parameter in C++
// into a tuple with the respective types int and float for
// use in Python.
boost::python::tuple parameterToTuple (std::pair<int, float> parameter)
{
    boost::python::tuple parameterTuple;
    parameterTuple = boost::python::make_tuple (parameter.first,
                                                parameter.second);
    return parameterTuple;
}

//==============================================================================
BOOST_PYTHON_MODULE(libbass)
{
    using namespace boost::python;
    
    // Vectors
    class_<std::vector<double>>("DoubleArray")
    .def(vector_indexing_suite<std::vector<double>>());
    class_<std::vector<float>>("FloatArray")
    .def(vector_indexing_suite<std::vector<float>>());
    class_<std::vector<std::string>>("StringArray")
    .def(vector_indexing_suite<std::vector<std::string>>());
    class_<std::vector<int>>("IntArray")
    .def(vector_indexing_suite<std::vector<int>>());
    class_<std::vector<bool>>("BoolArray")
    .def(vector_indexing_suite<std::vector<bool>>());
    
    // 2D Vectors
    class_<std::vector<std::vector<double>>>("DoubleArray2D")
    .def(vector_indexing_suite<std::vector<std::vector<double>>>());
    class_<std::vector<std::vector<float>>>("FloatArray2D")
    .def(vector_indexing_suite<std::vector<std::vector<float>>>());
    
    // Plugin Host
    class_<PluginHost>("PluginHost")
    .def("loadPlugin", &PluginHost::loadPlugin)
    .def("getParameterValue", &PluginHost::getParameterValue)
    .def("setParameterValue", &PluginHost::setParameterValue)
    .def("getNumParameters", &PluginHost::getNumParameters)
    .def("getPluginName", &PluginHost::getPluginName)
    .def("processAudioMono", &PluginHost::processAudioMono)
    .def("prepareToPlay", &PluginHost::prepareToPlay)
    ;
}
