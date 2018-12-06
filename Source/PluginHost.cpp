/*
 ==============================================================================
 
 PluginHost.cpp
 Created: 19 Feb 2017 9:47:15pm
 Author:  tollie
 
 ==============================================================================
 */

#include "PluginHost.h"


//------------------------------------------------------------------------------
//
PluginHost::PluginHost():
_plugin(nullptr),
_sampleRate(44100.0),
_bufferSize(512)
{
};

//------------------------------------------------------------------------------
//
PluginHost::~PluginHost()
{
    if (_plugin != nullptr)
    {
        _plugin->releaseResources();
        delete _plugin;
    }
};

//------------------------------------------------------------------------------
//
bool PluginHost::loadPlugin(const std::string& path)
{
    OwnedArray<PluginDescription> pluginDescriptions;
    KnownPluginList pluginList;
    AudioPluginFormatManager pluginFormatManager;
    
    pluginFormatManager.addDefaultFormats();
    
    for (int i = pluginFormatManager.getNumFormats(); --i >= 0;)
    {
        pluginList.scanAndAddFile (String (path),
                                   true,
                                   pluginDescriptions,
                                   *pluginFormatManager.getFormat(i));
    }
    
    // If there is a problem here first check the preprocessor definitions
    // in the projucer are sensible - is it set up to scan for plugin's?
    jassert (pluginDescriptions.size() > 0);
    
    String errorMessage;
    
    if (_plugin != nullptr)
    {
        delete _plugin;
    }
    
    _plugin = pluginFormatManager.createPluginInstance (*pluginDescriptions[0],
                                                        _sampleRate,
                                                        _bufferSize,
                                                        errorMessage);
    if (_plugin != nullptr)
    {
        // Success so set up plugin, then set up features and get all available
        // parameters from this given plugin.
        _plugin->prepareToPlay (_sampleRate, _bufferSize);
        _plugin->setNonRealtime (true);
        
        return true;
    }
    
    std::cout << "PluginHost::loadPlugin error: " << errorMessage.toStdString() << std::endl;
    
    return false;
}

//------------------------------------------------------------------------------
//
float
PluginHost::getParameterValue(const int index)
{
    return _plugin->getParameter(index);
};

//------------------------------------------------------------------------------
//
void
PluginHost::setParameterValue(const int index, const float newValue)
{
    _plugin->setParameter(index, newValue);
};

//------------------------------------------------------------------------------
//
const std::string
PluginHost::getPluginName()
{
    return _plugin->getName().toStdString();
};

//------------------------------------------------------------------------------
//
const int
PluginHost::getNumParameters()
{
    return _plugin->getNumParameters();
};

//------------------------------------------------------------------------------
//
void
PluginHost::prepareToPlay(double sampleRate, int numSamples)
{
    _plugin->prepareToPlay(sampleRate, numSamples);
}

//------------------------------------------------------------------------------
//
void
PluginHost::processAudioMono(std::vector<float>& buffer)
{
    // Convert std::vector to juce::AudioBuffer
    float* dataPtrs[1] = {buffer.data()};
    AudioBuffer<float> audioBuffer(dataPtrs, 1, buffer.size());
    MidiBuffer midiBuffer;
    
    // Process audio through plugin
    _plugin->processBlock(audioBuffer, midiBuffer);
};
