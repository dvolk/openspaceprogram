// audio.cpp -- see audio.h for the module contract (three kinds of
// tracks, silent degradation, the camera as listener).

#include "audio.h"
#include "camera.h"

#include <cstdio>
#include <fcntl.h>
#include <unistd.h>

bool Audio::init() {
    if(mixer_ != nullptr) { return true; }
    if(!MIX_Init()) {
        printf("audio: unavailable (%s) -- running silent\n", SDL_GetError());
        return false;
    }
    // Open the device with stderr muted: on a machine without a sound
    // card the ALSA backend dumps its own diagnostics to fd 2 (the e2e
    // battery runs under Xvfb, and the smoke case forbids stray "error:"
    // lines on boot). Restore stderr right after -- this only ever
    // affects this process.
    const int savedErr = dup(2);
    const int devnull = open("/dev/null", O_WRONLY);
    if(savedErr != -1 && devnull != -1) { dup2(devnull, 2); }
    if(devnull != -1) { close(devnull); }

    // spec = NULL: take the device's native format; the mixer converts.
    MIX_Mixer *m = MIX_CreateMixerDevice(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, nullptr);

    if(savedErr != -1) {
        dup2(savedErr, 2);
        close(savedErr);
    }
    if(m == nullptr) {
        printf("audio: no playback device (%s) -- running silent\n", SDL_GetError());
        MIX_Quit();
        return false;
    }
    mixer_ = m;
    return true;
}

void Audio::shutdown() {
    if(mixer_ == nullptr) { return; }
    // The tracks die with the mixer; the loaded audio is ours to free.
    for(auto &kv : audios_) { MIX_DestroyAudio(kv.second); }
    audios_.clear();
    oneShots_.clear();
    loop_ = nullptr;
    music_ = nullptr;
    MIX_DestroyMixer(mixer_);
    mixer_ = nullptr;
    MIX_Quit();
}

MIX_Audio *Audio::loadAudio(const std::string &path) {
    auto it = audios_.find(path);
    if(it != audios_.end()) { return it->second; }
    // predecode = false: keep the (compressed) file data in RAM and decode
    // on playback -- the 10-minute music track stays ~9 MB, not ~50.
    MIX_Audio *a = MIX_LoadAudio(mixer_, path.c_str(), false);
    if(a == nullptr) {
        printf("audio: cannot load %s: %s\n", path.c_str(), SDL_GetError());
        return nullptr;
    }
    audios_[path] = a;
    return a;
}

void Audio::setMixerPos(MIX_Track *t, const glm::dvec3 &world, const Camera &cam) {
    const glm::dvec3 p = listenerRelative(cam.pos, cam.up, cam.forward, world);
    MIX_Point3D mp = { (float)p.x, (float)p.y, (float)p.z };
    MIX_SetTrack3DPosition(t, &mp);
}

void Audio::playOnce(const std::string &path, const glm::dvec3 &worldPos) {
    if(mixer_ == nullptr) { return; }
    MIX_Audio *a = loadAudio(path);
    if(a == nullptr) { return; }
    if(oneShots_.size() >= MAX_ONESHOTS) { return; }
    MIX_Track *t = MIX_CreateTrack(mixer_);
    if(t == nullptr) { return; }
    if(!MIX_SetTrackAudio(t, a)) { MIX_DestroyTrack(t); return; }
    MIX_SetTrackGain(t, sfxVol_);
    if(!MIX_PlayTrack(t, 0)) { MIX_DestroyTrack(t); return; }
    oneShots_.push_back({ t, worldPos });
}

void Audio::setLoop(const std::string &path, bool active, float gain, const glm::dvec3 &worldPos) {
    if(mixer_ == nullptr) { return; }
    if(loop_ == nullptr || loopPath_ != path) {
        // A different loop: retire the old track (its audio stays cached).
        if(loop_ != nullptr) { MIX_DestroyTrack(loop_); }
        MIX_Audio *a = loadAudio(path);
        if(a == nullptr) { return; }
        loop_ = MIX_CreateTrack(mixer_);
        if(loop_ == nullptr) { return; }
        if(!MIX_SetTrackAudio(loop_, a)) { MIX_DestroyTrack(loop_); loop_ = nullptr; return; }
        MIX_SetTrackLoops(loop_, -1);
        loopPath_ = path;
        loopActive_ = false;
    }
    loopGain_ = gain;
    loopWorld_ = worldPos;
    if(!active) {
        if(MIX_TrackPlaying(loop_)) {
            const Sint64 fade = MIX_TrackMSToFrames(loop_, 120);
            MIX_StopTrack(loop_, fade);
        }
        loopActive_ = false;
        return;
    }
    if(!MIX_TrackPlaying(loop_)) {
        // a short fade-in so ignition doesn't pop
        SDL_PropertiesID o = SDL_CreateProperties();
        SDL_SetNumberProperty(o, MIX_PROP_PLAY_LOOPS_NUMBER, -1);
        SDL_SetNumberProperty(o, MIX_PROP_PLAY_FADE_IN_MILLISECONDS_NUMBER, 60);
        MIX_PlayTrack(loop_, o);
        SDL_DestroyProperties(o);
    }
    MIX_SetTrackGain(loop_, gain * sfxVol_);
    loopActive_ = true;
}

void Audio::setMusic(const std::string &path) {
    if(mixer_ == nullptr) { return; }
    if(music_ != nullptr && musicPath_ == path && MIX_TrackPlaying(music_)) { return; }
    MIX_Audio *a = loadAudio(path);
    if(a == nullptr) { return; }
    if(music_ == nullptr) {
        music_ = MIX_CreateTrack(mixer_);
        if(music_ == nullptr) { return; }
        if(!MIX_SetTrackAudio(music_, a)) { MIX_DestroyTrack(music_); music_ = nullptr; return; }
        MIX_SetTrackLoops(music_, -1);
        musicPath_ = path;
    } else if(!MIX_SetTrackAudio(music_, a)) {
        MIX_DestroyTrack(music_);
        music_ = nullptr;
        return;
    }
    if(MIX_TrackPlaying(music_)) { MIX_StopTrack(music_, 0); }
    SDL_PropertiesID o = SDL_CreateProperties();
    SDL_SetNumberProperty(o, MIX_PROP_PLAY_LOOPS_NUMBER, -1);
    SDL_SetNumberProperty(o, MIX_PROP_PLAY_FADE_IN_MILLISECONDS_NUMBER, 500);
    MIX_PlayTrack(music_, o);
    SDL_DestroyProperties(o);
    MIX_SetTrackGain(music_, musicVol_);
}

void Audio::stopMusic() {
    if(mixer_ == nullptr || music_ == nullptr) { return; }
    if(MIX_TrackPlaying(music_)) { MIX_StopTrack(music_, 0); }
}

void Audio::setSfxVolume(float v) {
    sfxVol_ = (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v);
    if(mixer_ != nullptr && loop_ != nullptr && MIX_TrackPlaying(loop_)) {
        MIX_SetTrackGain(loop_, loopGain_ * sfxVol_);
    }
}

void Audio::setMusicVolume(float v) {
    musicVol_ = (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v);
    if(mixer_ != nullptr && music_ != nullptr && MIX_TrackPlaying(music_)) {
        MIX_SetTrackGain(music_, musicVol_);
    }
}

void Audio::update(const Camera &cam) {
    if(mixer_ == nullptr) { return; }
    for(size_t i = oneShots_.size(); i-- > 0;) {
        Shot &s = oneShots_[i];
        if(!MIX_TrackPlaying(s.t)) {
            MIX_DestroyTrack(s.t);
            oneShots_.erase(oneShots_.begin() + (int)i);
            continue;
        }
        setMixerPos(s.t, s.world, cam);
    }
    if(loopActive_ && loop_ != nullptr && MIX_TrackPlaying(loop_)) {
        setMixerPos(loop_, loopWorld_, cam);
    }
}
