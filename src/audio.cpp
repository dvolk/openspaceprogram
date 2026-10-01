// audio.cpp -- see audio.h for the module contract.
#include "audio.h"

#include "resdir.h"

#include <cstdio>

bool Audio::init() {
    if(mixer_ != nullptr) { return true; }
    dbg_ = (getenv("AUDIO_DEBUG") != nullptr);
    if(!MIX_Init()) {
        printf("audio: unavailable (%s) -- running silent\n", SDL_GetError());
        return false;
    }
    // Generous device buffer: the ALSA backend is not forgiving of a briefly
    // delayed callback (the "crack" when the engine track starts mixing).
    // Extra latency is fine for ambient game SFX/music.
    SDL_SetHint(SDL_HINT_AUDIO_DEVICE_SAMPLE_FRAMES, "8192");

    // spec = NULL: take the device's native format; the mixer converts.
    MIX_Mixer *m = MIX_CreateMixerDevice(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, nullptr);
    if(m == nullptr) {
        printf("audio: no playback device (%s) -- running silent\n", SDL_GetError());
        MIX_Quit();
        return false;
    }
    mixer_ = m;
    if(dbg_) {
        SDL_AudioSpec spec;
        if(MIX_GetMixerFormat(mixer_, &spec)) {
            printf("[aud] init: driver=%s device OK %dHz/%dch/%s  mixerGain=%.2f\n",
                   SDL_GetCurrentAudioDriver(),
                   (int)spec.freq, (int)spec.channels,
                   spec.format == SDL_AUDIO_F32 ? "F32"
                   : (spec.format == SDL_AUDIO_S16 ? "S16"
                   : (spec.format == SDL_AUDIO_S32 ? "S32" : "?")),
                   (double)MIX_GetMixerGain(mixer_));
        } else {
            printf("[aud] init: driver=%s device OK (format query failed: %s)\n",
                   SDL_GetCurrentAudioDriver(), SDL_GetError());
        }
        fflush(stdout);
    }
    return true;
}

int Audio::deviceRate() const {
    if(mixer_ == nullptr) { return 0; }
    SDL_AudioSpec spec;
    if(!MIX_GetMixerFormat(mixer_, &spec)) { return 0; }
    return (int)spec.freq;
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

MIX_Audio *Audio::loadAudio(const std::string &path, bool predecode) {
    // Cache on the resolved path so "res/x" and "./res/x" cannot double-load.
    const std::string key = resdir::path(path);
    auto it = audios_.find(key);
    if(it != audios_.end()) { return it->second; }
    // predecode: SFX are tiny WAVs (free); music streams (a full decode stalls boot).
    MIX_Audio *a = MIX_LoadAudio(mixer_, key.c_str(), predecode);
    if(a == nullptr) {
        printf("audio: cannot load %s: %s\n", path.c_str(), SDL_GetError());
        return nullptr;
    }
    audios_[key] = a;
    if(dbg_) {
        SDL_AudioSpec spec;
        const char *fmt = "?";
        if(MIX_GetAudioFormat(a, &spec)) {
            fmt = (spec.format == SDL_AUDIO_F32) ? "F32"
                 : (spec.format == SDL_AUDIO_S16) ? "S16"
                 : (spec.format == SDL_AUDIO_S32) ? "S32" : "?";
            // MIX_GetAudioDuration is in SAMPLE FRAMES (not ms).
            const Sint64 frames = MIX_GetAudioDuration(a);
            printf("[aud] load: %s  %dHz/%dch/%s  dur=%.1fs\n",
                   path.c_str(), (int)spec.freq, (int)spec.channels, fmt,
                   (frames >= 0 && spec.freq > 0)
                       ? (double)frames / (double)spec.freq
                       : -1.0);
        }
        fflush(stdout);
    }
    return a;
}

void Audio::playOnce(const std::string &path, float balance) {
    if(mixer_ == nullptr) { return; }
    MIX_Audio *a = loadAudio(path, true);
    if(a == nullptr) { return; }
    if(oneShots_.size() >= MAX_ONESHOTS) { return; }
    MIX_Track *t = MIX_CreateTrack(mixer_);
    if(t == nullptr) { return; }
    if(!MIX_SetTrackAudio(t, a)) { MIX_DestroyTrack(t); return; }
    const float gain = sfxVol_ * balance;   // balance trims a hot one-shot down
    MIX_SetTrackGain(t, gain);
    const bool ok = MIX_PlayTrack(t, 0);
    if(dbg_) {
        printf("[aud] playOnce: %s gain=%.2f (balance=%.2f) playing=%d\n",
               path.c_str(), (double)gain, (double)balance,
               (int)MIX_TrackPlaying(t));
        fflush(stdout);
    }
    if(!ok) { MIX_DestroyTrack(t); return; }
    Shot s; s.t = t; s.born_ms = SDL_GetTicks();
    oneShots_.push_back(s);
}

void Audio::primeTrack(MIX_Track *t) {
    SDL_PropertiesID o = SDL_CreateProperties();
    SDL_SetNumberProperty(o, MIX_PROP_PLAY_LOOPS_NUMBER, -1);
    const bool ok = MIX_PlayTrack(t, o);
    SDL_DestroyProperties(o);
    if(!ok) { return; }
    MIX_StopTrack(t, 0);   // the callback ran at least once: buffers are grown
}

void Audio::setLoop(const char *path, bool active, float gain) {
    if(mixer_ == nullptr) { return; }
    if(!active) {
        // Fade out (no mid-wave cut). Arm the fade exactly ONCE: re-arming
        // every frame would never let it complete. The track is kept warm
        // for the next ignition.
        loopActive_ = false;
        if(loop_ == nullptr) { return; }
        if(!loopStopping_) {
            loopStopping_ = true;
            if(dbg_) { printf("[aud] setLoop: STOP (fading out)\n"); fflush(stdout); }
            MIX_StopTrack(loop_, MIX_TrackMSToFrames(loop_, 150));
        }
        return;
    }
    // Re-ignition during a stop-fade: start fresh (cancels the fade cleanly).
    if(loopStopping_) {
        if(loop_ != nullptr) { MIX_DestroyTrack(loop_); loop_ = nullptr; loopPath_.clear(); }
        loopStopping_ = false;
    }
    if(loop_ == nullptr || loopPath_ != path) {
        // A different loop: retire the old track (its audio stays cached).
        if(loop_ != nullptr) { MIX_DestroyTrack(loop_); }
        MIX_Audio *a = loadAudio(path, true);   // engine hum: tiny WAV, predecode is free
        if(a == nullptr) { return; }
        loop_ = MIX_CreateTrack(mixer_);
        if(loop_ == nullptr) { return; }
        if(!MIX_SetTrackAudio(loop_, a)) { MIX_DestroyTrack(loop_); loop_ = nullptr; return; }
        primeTrack(loop_);   // warm the buffers off the real-time thread
        loopPath_ = path;
        loopActive_ = false;
    }
    loopGain_ = gain;
    if(!MIX_TrackPlaying(loop_)) {
        // A fresh track starts at the gain set below (a fade-in reads as a
        // "click"). loops=-1: PlayTrack with no options would reset the loop
        // count and the engine would cut out after one pass.
        SDL_PropertiesID o = SDL_CreateProperties();
        SDL_SetNumberProperty(o, MIX_PROP_PLAY_LOOPS_NUMBER, -1);
        const bool ok = MIX_PlayTrack(loop_, o);
        SDL_DestroyProperties(o);
        if(!ok) {
            MIX_DestroyTrack(loop_);
            loop_ = nullptr;
            return;
        }
    }
    MIX_SetTrackGain(loop_, gain * sfxVol_);
    loopActive_ = true;
    if(dbg_) {
        printf("[aud] setLoop: active gain=%.2f playing=%d\n",
               (double)gain, (int)MIX_TrackPlaying(loop_));
        fflush(stdout);
    }
}

void Audio::setMusic(const std::string &path) {
    if(mixer_ == nullptr) { return; }
    if(music_ != nullptr && musicPath_ == path && MIX_TrackPlaying(music_)) { return; }
    MIX_Audio *a = loadAudio(path, false);   // music: stream it (a full decode stalls boot)
    if(a == nullptr) { return; }
    if(music_ == nullptr) {
        music_ = MIX_CreateTrack(mixer_);
        if(music_ == nullptr) { return; }
        if(!MIX_SetTrackAudio(music_, a)) { MIX_DestroyTrack(music_); music_ = nullptr; return; }
        MIX_SetTrackLoops(music_, -1);
    } else if(!MIX_SetTrackAudio(music_, a)) {
        MIX_DestroyTrack(music_);
        music_ = nullptr;
        return;
    }
    musicPath_ = path;   // keep the guard above honest on a path swap
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

void Audio::update() {
    if(mixer_ == nullptr) { return; }
    for(size_t i = oneShots_.size(); i-- > 0;) {
        Shot &s = oneShots_[i];
        if(!MIX_TrackPlaying(s.t)) {
            if(dbg_) {
                printf("[aud] reap: one-shot %p lived %d ms\n",
                       (void *)s.t,
                       (int)(SDL_GetTicks() - s.born_ms));
                fflush(stdout);
            }
            MIX_DestroyTrack(s.t);
            oneShots_.erase(oneShots_.begin() + (int)i);
        }
    }
    // Stop-fade finished: keep the track warm (a fresh track would reallocate
    // its mix buffers inside the real-time callback -- the "crack on a tap").
    if(loopStopping_ && loop_ != nullptr && !MIX_TrackPlaying(loop_)) {
        if(dbg_) { printf("[aud] loop fade done -> kept warm\n"); fflush(stdout); }
        loopStopping_ = false;
    }
}
