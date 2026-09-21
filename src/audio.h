// audio.h -- the game's sound, over SDL_mixer:
//
//   one-shots  a track per fire (capped, reaped when done) -- the
//              decoupler pop; positional (the listener is the camera)
//   the loop   one persistent track, infinite loops -- the engine hum;
//              gain tracks the throttle, repositioned every frame
//   the music  one persistent track, infinite loops -- ambient, and
//              deliberately NOT positional (it would duck with distance)
//
// Everything degrades to SILENCE: no audio device (headless, the e2e
// battery under Xvfb) or a missing file just means that sound never
// happens -- init() / loadAudio() report it once and every call after
// is a no-op, so the game runs exactly as it did before audio existed.
//
// The mixer's positional system (like OpenAL's) is right-handed with the
// listener at the origin: +x right, +y up, -z forward. It does distance
// attenuation + spatialization; no doppler, no rolloff curve of our
// choosing (the library's "good enough" 3D, per its own docs).
#pragma once

#include <SDL3/SDL.h>
#include <SDL3_mixer/SDL_mixer.h>

#include <map>
#include <string>
#include <vector>

#include <glm/glm.hpp>

class Camera;   // update() reads the live one as the listener

/* A source's WORLD position converted to the mixer's listener frame
   (the listener at the origin, looking down -z). The camera supplies
   the listener's forward (view direction) and screen up; the world is
   y-up. Pure math -- unit-testable without a mixer (test_audio). */
inline glm::dvec3 listenerRelative(const glm::dvec3 &listener,
                                   const glm::dvec3 &listenerUp,
                                   const glm::dvec3 &listenerFwd,
                                   const glm::dvec3 &source)
{
    const glm::dvec3 f = glm::normalize(listenerFwd);
    glm::dvec3 r = glm::cross(f, listenerUp);
    if(glm::length(r) < 1e-9) { r = glm::dvec3(1.0, 0.0, 0.0); }   // up || forward: any right axis works
    r = glm::normalize(r);
    const glm::dvec3 u = glm::cross(r, f);
    const glm::dvec3 rel = source - listener;
    return glm::dvec3(glm::dot(rel, r), glm::dot(rel, u), -glm::dot(rel, f));
}

class Audio {
public:
    /* MIX_Init + open the default playback device. false (no device)
       -> the object stays silent forever; every method below is a no-op. */
    bool init();
    void shutdown();

    bool enabled() const { return mixer_ != nullptr; }

    /* One-shot SFX at a world position (the decoupler pop). Capped at
       MAX_ONESHOTS concurrent; finished tracks are reaped on update(). */
    void playOnce(const std::string &path, const glm::dvec3 &worldPos);

    /* The looping SFX (the engine hum). active starts/stops it (a short
       fade on the stop so the cutoff doesn't click), gain in [0,1]
       (the throttle), worldPos repositioned every frame while active. */
    void setLoop(const std::string &path, bool active, float gain, const glm::dvec3 &worldPos);

    /* Ambient music: load once (decoded on playback, not pre-expanded),
       loop forever. A missing file is a logged no-op. */
    void setMusic(const std::string &path);
    void stopMusic();

    /* Master levels in [0,1] (the Settings window drives these). */
    void setSfxVolume(float v);
    void setMusicVolume(float v);

    /* Per frame, with the live camera as listener: reaps finished
       one-shots and repositions the loop. No-op when disabled/idle. */
    void update(const Camera &cam);

private:
    static const size_t MAX_ONESHOTS = 8;

    MIX_Audio *loadAudio(const std::string &path);   // cached per path; null on miss
    void setMixerPos(MIX_Track *t, const glm::dvec3 &world, const Camera &cam);

    struct Shot {
        MIX_Track *t;
        glm::dvec3 world;
    };

    MIX_Mixer *mixer_ = nullptr;

    float sfxVol_ = 1.0f;     // master for the one-shots + the loop
    float musicVol_ = 0.5f;   // ambient music (background, not the star)

    std::map<std::string, MIX_Audio *> audios_;      // loaded once per path

    std::vector<Shot> oneShots_;                     // capped; reaped in update()

    MIX_Track *loop_ = nullptr;
    std::string loopPath_;
    bool loopActive_ = false;
    float loopGain_ = 0.0f;
    glm::dvec3 loopWorld_ = glm::dvec3(0.0);   // repositioned in update()

    MIX_Track *music_ = nullptr;
    std::string musicPath_;
};
