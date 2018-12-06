/*
  ==============================================================================

    RenderEngine.h
    Created: 19 Feb 2017 9:47:15pm
    Author:  tollie

  ==============================================================================
*/

#ifndef RENDERENGINE_H_INCLUDED
#define RENDERENGINE_H_INCLUDED

#include <random>
#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include "Maximilian/maximilian.h"
#include "Maximilian/libs/maxiFFT.h"
#include "Maximilian/libs/maxiMFCC.h"
#include "../JuceLibraryCode/JuceHeader.h"

using namespace juce;

typedef std::vector<std::pair<int, float>>  PluginPatch;
typedef std::vector<std::array<double, 13>> MFCCFeatures;

class RenderEngine
{
public:
    RenderEngine (int sr,
                  int bs,
                  int ffts) :
    _sampleRate(sr),
    _bufferSize(bs),
    _fftSize(ffts),
    _plugin(nullptr)
    {
        maxiSettings::setup (_sampleRate, 1, _bufferSize);
    }

    virtual ~RenderEngine()
    {
        if (_plugin != nullptr)
        {
            _plugin->releaseResources();
            delete _plugin;
        }
    }


    bool loadPlugin (const std::string& path);

    void setPatch (const PluginPatch patch);

    const PluginPatch getPatch();

    void renderPatch (const uint8  midiNote,
                      const uint8  midiVelocity,
                      const double noteLength,
                      const double renderLength);

    const MFCCFeatures getMFCCFrames();

    const MFCCFeatures getNormalisedMFCCFrames (const std::array<double, 13>& mean,
                                                const std::array<double, 13>& variance);

    const std::vector<double> getRMSFrames();

    const size_t getPluginParameterSize();

    const String getPluginParametersDescription();

    bool overridePluginParameter (const int   index,
                                  const float value);

    bool removeOverridenParameter (const int index);
    const std::vector<double> getAudioFrames();
    
    bool writeToWav(const std::string& path);
    
    
    // Stuart's methods
    float getParameterValue(const int index);
    void setParameterValue(const int index, const float newValue);
    const std::string getPluginName();
    void processAudioMono(std::vector<float>& buffer);

private:
    void fillAudioFeatures (const AudioSampleBuffer& data,
                            maxiFFT&                 fft);

    void ifTimeSetNoteOff (const double& noteLength,
                           const double& sampleRate,
                           const int&    bufferSize,
                           const uint8&  midiChannel,
                           const uint8&  midiPitch,
                           const uint8&  midiVelocity,
                           const int&    currentBufferIndex,
                           MidiBuffer&   bufferToNoteOff);

    void fillAvailablePluginParameters (PluginPatch& params);

    double               _sampleRate;
    int                  _bufferSize;
    int                  _fftSize;
    maxiMFCC             _mfcc;
    AudioPluginInstance* _plugin;
    PluginPatch          _pluginParameters;
    PluginPatch          _overridenParameters;
    MFCCFeatures         _mfccFeatures;
    std::vector<double>  _processedMonoAudioPreview;
    std::vector<double>  _rmsFrames;
    double               _currentRmsFrame;
};


#endif  // RENDERENGINE_H_INCLUDED
