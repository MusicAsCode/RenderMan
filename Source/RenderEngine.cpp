/*
  ==============================================================================

    RenderEngine.cpp
    Created: 19 Feb 2017 9:47:15pm
    Author:  tollie

  ==============================================================================
*/

#include "RenderEngine.h"

//==============================================================================
bool RenderEngine::loadPlugin (const std::string& path)
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

    if (_plugin != nullptr) delete _plugin;

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

        _mfcc.setup (512, 42, 13, 20, int (_sampleRate / 2), _sampleRate);

        // Resize the pluginParameters patch type to fit this plugin and init
        // all the values to 0.0f!
        fillAvailablePluginParameters (_pluginParameters);

        return true;
    }

    std::cout << "RenderEngine::loadPlugin error: "
    << errorMessage.toStdString()
    << std::endl;

    return false;
}

//==============================================================================
void RenderEngine::renderPatch (const uint8  midiNote,
                                const uint8  midiVelocity,
                                const double noteLength,
                                const double renderLength)
{
    // Get the overriden patch and set the vst parameters with it.
    PluginPatch overridenPatch = getPatch();
    for (const auto& parameter : overridenPatch)
        _plugin->setParameter (parameter.first, parameter.second);

    // Get the note on midiBuffer.
    MidiMessage onMessage = MidiMessage::noteOn (1,
                                                 midiNote,
                                                 midiVelocity);
    onMessage.setTimeStamp(0);
    MidiBuffer midiNoteBuffer;
    midiNoteBuffer.addEvent (onMessage, onMessage.getTimeStamp());

    // Setup fft here so it is destroyed when rendering is finished and
    // the stack unwinds so it doesn't share frames with a new patch.
    maxiFFT fft;
    fft.setup (_fftSize, _fftSize / 2, _fftSize / 4);

    // Data structure to hold multi-channel audio data.
    AudioSampleBuffer audioBuffer (_plugin->getTotalNumOutputChannels(),
                                   _bufferSize);

    int numberOfBuffers = int (std::ceil (renderLength * _sampleRate / _bufferSize));

    // Clear and reserve memory for the audio storage!
    _processedMonoAudioPreview.clear();
    _processedMonoAudioPreview.reserve (numberOfBuffers * _bufferSize);

    // Number of FFT, MFCC and RMS frames.
    int numberOfFFT = int (std::ceil (renderLength * _sampleRate / _fftSize)) * 4;
    _rmsFrames.clear();
    _rmsFrames.reserve (numberOfFFT);
    _currentRmsFrame = 0.0;
    _mfccFeatures.clear();
    _mfccFeatures.reserve (numberOfFFT);

    _plugin->prepareToPlay (_sampleRate, _bufferSize);

    for (int i = 0; i < numberOfBuffers; ++i)
    {
        // Trigger note off if in the correct audio buffer.
        ifTimeSetNoteOff (noteLength,
                          _sampleRate,
                          _bufferSize,
                          1,
                          midiNote,
                          midiVelocity,
                          i,
                          midiNoteBuffer);

        // Turn Midi to audio via the vst.
        _plugin->processBlock (audioBuffer, midiNoteBuffer);

        // Get audio features and fill the datastructure.
        fillAudioFeatures (audioBuffer, fft);
    }
}

//=============================================================================
void RenderEngine::fillAudioFeatures (const AudioSampleBuffer& data,
                                      maxiFFT&                 fft)
{
    // Keep it auto as it may or may not be double precision.
    const auto readptrs = data.getArrayOfReadPointers();
    for (int i = 0; i < data.getNumSamples(); ++i)
    {
        // Mono the frame.
        int channel = 0;
        auto currentFrame = readptrs[channel][i];
        const int numberChannels = data.getNumChannels();

        while (++channel < numberChannels)
            currentFrame += readptrs[channel][i];

        currentFrame /= numberChannels;

        // Save the audio for playback and plotting!
        _processedMonoAudioPreview.push_back (currentFrame);

        // RMS.
        _currentRmsFrame += (currentFrame * currentFrame);

        // Extract features.
        if (fft.process (currentFrame))
        {
            // This isn't real-time so I can take the luxuary of allocating
            // heap memory here.
            double* mfccs = new double[13];
            _mfcc.mfcc (fft.magnitudes, mfccs);

            std::array<double, 13> mfccsFrame;
            std::memcpy (mfccsFrame.data(), mfccs, sizeof (double) * 13);

            // Add the mfcc frames here.
            _mfccFeatures.push_back (mfccsFrame);
            delete[] mfccs;

            // Root Mean Square.
            _currentRmsFrame /= _fftSize;
            _currentRmsFrame = sqrt (_currentRmsFrame);
            _rmsFrames.push_back (_currentRmsFrame);
            _currentRmsFrame = 0.0;
        }
    }
}

//=============================================================================
void RenderEngine::ifTimeSetNoteOff (const double& noteLength,
                                     const double& sampleRate,
                                     const int&    bufferSize,
                                     const uint8&  midiChannel,
                                     const uint8&  midiPitch,
                                     const uint8&  midiVelocity,
                                     const int&    currentBufferIndex,
                                     MidiBuffer&   bufferToNoteOff)
{
    double eventFrame = noteLength * sampleRate;
    bool bufferBeginIsBeforeEvent = currentBufferIndex * bufferSize < eventFrame;
    bool bufferEndIsAfterEvent = (currentBufferIndex + 1) * bufferSize >= eventFrame;
    bool noteOffEvent = bufferBeginIsBeforeEvent && bufferEndIsAfterEvent;
    if (noteOffEvent)
    {
        MidiBuffer midiOffBuffer;
        MidiMessage offMessage = MidiMessage::noteOff (midiChannel,
                                                       midiPitch,
                                                       midiVelocity);
        offMessage.setTimeStamp(eventFrame);
        midiOffBuffer.addEvent(offMessage, offMessage.getTimeStamp());
        bufferToNoteOff = midiOffBuffer;
    }
}



//==============================================================================
bool RenderEngine::overridePluginParameter (const int   index,
                                            const float value)
{
    int biggestParameterIndex = _pluginParameters.size() - 1;

    if (biggestParameterIndex < 0)
    {
        std::cout << "RenderEngine::overridePluginParameter error: " <<
                     "No patch set. Is the plugin loaded?" << std::endl;
        return false;
    }
    else if (index > _pluginParameters[biggestParameterIndex].first)
    {
        std::cout << "RenderEngine::overridePluginParameter error: " <<
                     "Overriden parameter index is greater than the biggest parameter index." <<
                     std::endl;
        return false;
    }
    else if (index < 0)
    {
        std::cout << "RenderEngine::overridePluginParameter error: " <<
                     "Overriden parameter index is less than the smallest parameter index." <<
                     std::endl;
        return false;
    }
    else if (value < 0.0 || value > 1.0)
    {
        std::cout << "RenderEngine::overridePluginParameter error: " <<
                     "Keep the overriden value between 0.0 and 1.0." <<
                     std::endl;
        return false;
    }

    auto iterator = std::find_if (_overridenParameters.begin(),
                                  _overridenParameters.end(),
                                  [&index] (const std::pair<int, float>& parameter)
                                  {
                                      return parameter.first == index;
                                  });

    bool exists = (iterator != _overridenParameters.end());

    if (exists)
        iterator->second = value;
    else
        _overridenParameters.push_back(std::make_pair(index, value));

    return true;
}

//==============================================================================
bool RenderEngine::removeOverridenParameter (const int index)
{
    int biggestParameterIndex = _pluginParameters.size() - 1;

    if (biggestParameterIndex < 0)
    {
        std::cout << "RenderEngine::removeOverridenParameter error: " <<
                     "No patch set. Is the plugin loaded?" << std::endl;
        return false;
    }
    else if (index > _pluginParameters[biggestParameterIndex].first)
    {
        std::cout << "RenderEngine::removeOverridenParameter error: " <<
                     "Overriden parameter index is greater than the biggest parameter index." <<
                     std::endl;
        return false;
    }
    else if (index < 0)
    {
        std::cout << "RenderEngine::removeOverridenParameter error: " <<
                     "Overriden parameter index is less than the smallest parameter index." <<
                     std::endl;
        return false;
    }

    auto iterator = std::find_if (_overridenParameters.begin(),
                                  _overridenParameters.end(),
                                  [&index] (const std::pair<int, float>& parameter)
                                  {
                                      return parameter.first == index;
                                  });

    bool exists = (iterator != _overridenParameters.end());

    if (exists)
    {
        _overridenParameters.erase(iterator);
        return true;
    }

    std::cout << "RenderEngine::removeOverridenParameter error: " <<
                 "Overriden parameter does not exist." <<
                 std::endl;
    return false;
}

//==============================================================================
void RenderEngine::fillAvailablePluginParameters (PluginPatch& params)
{
    params.clear();
    params.reserve (_plugin->getNumParameters());

    int usedParameterAmount = 0;
    for (int i = 0; i < _plugin->getNumParameters(); ++i)
    {
        // Ensure the parameter is not unused.
        if (_plugin->getParameterName(i) != "Param")
        {
            ++usedParameterAmount;
            params.push_back (std::make_pair (i, 0.0f));
        }
    }
    params.shrink_to_fit();
}

//==============================================================================
const String RenderEngine::getPluginParametersDescription()
{
    String parameterListString ("");

    if (_plugin != nullptr)
    {
        std::ostringstream ss;

        for (const auto& pair : _pluginParameters)
        {
            ss << std::setw (3) << std::setfill (' ') << pair.first;

            const String name = _plugin->getParameterName (pair.first);
            const String index (ss.str());

            parameterListString = parameterListString +
                                  index + ": " + name +
                                  "\n";
            ss.str ("");
            ss.clear();
        }
    }
    else
    {
        std::cout << "Please load the plugin first!" << std::endl;
    }

    return parameterListString;
}

//==============================================================================
void RenderEngine::setPatch (const PluginPatch patch)
{
    const size_t currentParameterSize = _pluginParameters.size();
    const size_t newPatchParameterSize = patch.size();

    if (currentParameterSize == newPatchParameterSize)
    {
        _pluginParameters = patch;
    }
    else
    {
        std::cout << "RenderEngine::setPatch error: Incorrect patch size!" <<
        "\n- Current size:  " << currentParameterSize <<
        "\n- Supplied size: " << newPatchParameterSize << std::endl;
    }
}

//==============================================================================
const PluginPatch RenderEngine::getPatch()
{
    if (_overridenParameters.size() == 0)
        return _pluginParameters;

    PluginPatch overridenPluginParameters = _pluginParameters;
    std::pair<int, float> copy;

    for (auto& parameter : overridenPluginParameters)
    {
        // Should we have overriden this parameter's index...
        if (std::any_of(_overridenParameters.begin(), _overridenParameters.end(),
                        [parameter, &copy] (std::pair<int, float> p)
                        {
                            copy = p;
                            return p.first == parameter.first;
                        }))
        {
            parameter = copy;
        }
    }
    return overridenPluginParameters;
}

//==============================================================================
const size_t RenderEngine::getPluginParameterSize()
{
    return _pluginParameters.size();
}

//==============================================================================
const MFCCFeatures RenderEngine::getMFCCFrames()
{
    return _mfccFeatures;
}

//==============================================================================
const MFCCFeatures RenderEngine::getNormalisedMFCCFrames(const std::array<double, 13>& mean,
                                                         const std::array<double, 13>& variance)
{
    MFCCFeatures normalisedMFCCFrames;
    normalisedMFCCFrames.resize(_mfccFeatures.size());

    for (size_t i = 0; i < normalisedMFCCFrames.size(); ++i)
    {
        for (size_t j = 0; j < 13; ++j)
        {
            normalisedMFCCFrames[i][j] =_mfccFeatures[i][j] - mean[j];
            normalisedMFCCFrames[i][j] /= variance[j];
        }
    }
    return normalisedMFCCFrames;
}

//==============================================================================
const std::vector<double> RenderEngine::getAudioFrames()
{
    return _processedMonoAudioPreview;
}

//==============================================================================
const std::vector<double> RenderEngine::getRMSFrames()
{
    return _rmsFrames;
}

//==============================================================================
bool RenderEngine::writeToWav(const std::string& path)
{
    const auto size = _processedMonoAudioPreview.size();
    if (size == 0)
        return false;

    maxiRecorder recorder;
    recorder.setup (path);
    recorder.startRecording();
    const double* data = _processedMonoAudioPreview.data();
    recorder.passData (data, size);
    recorder.stopRecording();
    recorder.saveToWav();
    return true;
}



float
RenderEngine::getParameterValue(const int index)
{
    return _plugin->getParameter(index);
};

void
RenderEngine::setParameterValue(const int index, const float newValue)
{
    _plugin->setParameter(index, newValue);
};

const std::string
RenderEngine::getPluginName()
{
    return _plugin->getName().toStdString();
};

void
RenderEngine::processAudioMono(std::vector<float>& buffer)
{
    // Convert std::vector to juce::AudioBuffer
    float* dataPtrs[1] = {buffer.data()};
    AudioBuffer<float> audioBuffer(dataPtrs, 1, buffer.size());
    MidiBuffer midiBuffer;
    
    // Process audio through plugin
    _plugin->processBlock(audioBuffer, midiBuffer);
};
