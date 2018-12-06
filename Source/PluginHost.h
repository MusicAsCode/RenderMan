/*
  ==============================================================================

    PluginHost.h
    Created: 19 Feb 2017 9:47:15pm
    Author:  tollie

  ==============================================================================
*/

#pragma once

#include <random>
#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include "../JuceLibraryCode/JuceHeader.h"

using namespace juce;

class PluginHost
{
public:
    PluginHost();
    virtual ~PluginHost();

    // Plugin info
    bool loadPlugin (const std::string& path);
    const std::string getPluginName();
    
    // Parameters
    const int getNumParameters();
    float getParameterValue(const int index);
    void setParameterValue(const int index, const float newValue);
    
    // Processing
    void prepareToPlay(double sampleRate, int numSamples);
    void processAudioMono(std::vector<float>& buffer);

private:

    double               _sampleRate;
    int                  _bufferSize;
    AudioPluginInstance* _plugin;
};
