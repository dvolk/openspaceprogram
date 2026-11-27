# Windows code signing — options, and what Steam does for you

Date: 2026-11-26
Status: research (no code or pipeline changes made)
Revision: corrected after an adversarial source pass; "Verified vs inferred" at
the end says which claims are which.

## TL;DR

- **Steam does not code-sign your executable.** SteamPipe chunks, compresses
  and encrypts depot content and ships it; the only thing its build process
  "signs" is `InstallScript` files, and that's a Steam `kvsignatures` VDF
  signature, not Authenticode. Valve *does* Authenticode-sign its own
  redistributables (`steam_api64.dll`, `steamclient*.dll`), which is why a
  Steam install directory is never wholly unsigned — but that's their DLL,
  not your game.
- **Steam usually spares you the SmartScreen dialog, and does not spare you
  Smart App Control.** Steam neither tags downloads with Mark-of-the-Web nor
  launches games through the shell, so the app-reputation prompt generally
  doesn't fire (Valve documents neither; this is inference, not a cited
  fact). SAC ignores all of that and has hard-blocked shipped Steam games —
  MENACE, November 2025, unsigned `Menace.exe`, error `0x11C7` — with no
  "Run anyway" escape hatch.
- **If you want a signature**, the two realistic routes are SignPath
  Foundation (free, AGPLv3-eligible, publisher name becomes "SignPath
  Foundation") and Azure Artifact Signing (ex-"Trusted Signing", $9.99/mo).
  The second one does *not* need a Windows runner: `jsign --storetype
  TRUSTEDSIGNING` signs PE files from Linux.
- **No EV cert buys instant trust any more.** Microsoft's SmartScreen docs
  say so outright.

## Current state (verified 2026-11-26)

- `.github/workflows/release.yml`: manual-dispatch, one `ubuntu-24.04`
  runner, `OS=windows ./bootstrap.sh` mingw cross-build, `make artifacts`,
  then `gh release` upload of `linux.tar.xz`, `windows.zip`, `AppImage`
  plus `SHA256SUMS`.
- The Windows artifact is a **single fully static PE** (`-static -mwindows`,
  no runtime DLLs), zipped with `res/`, docs and licence. One executable to
  sign; nothing else in the zip is a PE.
- Remotes: `origin` = Gitea (`git.oxfordfun.com`), `github` =
  `github.com/dvolk/openspaceprogram`.
- License: AGPLv3 (code). `res/` carries third-party assets under their own
  attribution files.
- No signing anywhere in the tree today (`signtool`/`osslsigncode`/
  Authenticode: zero hits in `.github/`, `utils/`, `release/`).

## What actually triggers the scary dialogs

Two different mechanisms, often conflated:

**SmartScreen** — Microsoft's [reputation
doc](https://learn.microsoft.com/en-us/windows/apps/package-and-deploy/smartscreen-reputation)
says it evaluates a file's hash and its publisher certificate. Their table:

| How it's signed | First-download behaviour |
|---|---|
| Microsoft Store (re-signed by MS) | no warning, ever |
| OV/EV cert | warning, but publisher name shown; hash reputation accrues |
| unsigned | "Windows protected your PC"; "Run anyway" available |
| self-signed | identical to unsigned |

Quotables:

- "**Even when signed, a newly created binary could still show a
  SmartScreen warning** until its hash or publisher certificate accumulates
  sufficient evidence of positive reputation."
- "**Unsigned files must build reputation anew with every update.**"
- "Several weeks and hundreds of clean installs." No manual submission path
  for consumer endpoints.
- "**EV certificates no longer bypass SmartScreen.** … Paying a premium for
  EV solely to avoid SmartScreen warnings is no longer justified."

The prompt itself is a *shell* check on files carrying Mark-of-the-Web, so
what matters is who launches the process and whether the file was tagged.

**Smart App Control (SAC)** — [overview](https://learn.microsoft.com/en-us/windows/apps/develop/smart-app-control/overview),
[FAQ](https://support.microsoft.com/en-us/windows/security/threat-malware-protection/smart-app-control-frequently-asked-questions).
Windows 11 only, on by default "in most cases" on new installs (and now
enableable without a clean install). In enforcement mode a binary runs only
if Microsoft's app-intelligence service predicts it's safe, *or* it's signed
by a CA in the Trusted Root Program. "Unknown, unsigned code are blocked by
default." No MOTW requirement, no per-app bypass, and once it's off the only
way back is a Windows reset.

Field evidence that this is not hypothetical: MENACE shipped on Steam in
2025, unsigned, and players got

> Failed to start process for this game: An Application Control policy has
> blocked this file. (0x11C7)

with a CodeIntegrity event naming the SAC policy. Reboots cleared it until
the next patch. Players were told the durable fix is a signature.

**Also:** since 2023-06-01 the CA/B Forum Code Signing Baseline Requirements
(6.2.7.4.2) cover *all* publicly-trusted code-signing certs — EV and OV —
and require the private key in a FIPS 140-2 Level 2 (or CC EAL4+) hardware
crypto module. So "buy cert, keep a .pfx, sign with `osslsigncode -key`" is
dead for new certs.

## Options

| Route | Cost | Gate | Signs where | Publisher shown |
|---|---|---|---|---|
| **SignPath Foundation** | free | OSI license, released project, verifiable build on a supported CI, *verifiable reputation*, MFA, per-release approval | their service, GitHub Actions integration | "SignPath Foundation" |
| **Azure Artifact Signing** | $9.99/mo Basic (5k sigs/mo) | paid Azure sub, identity validation, country gate, cert CN = your legal name | Linux via `jsign --storetype TRUSTEDSIGNING`; Windows via `signtool` + dlib | your legal name |
| **Commercial OV** | ~$200–600/yr (unverified range) | business/ID validation | key in HSM; Linux via `jsign --storetype ESIGNER` or SSL.com CodeSignTool | your legal name |
| **Commercial EV** | more, no SmartScreen benefit | as above | as above | your legal name |
| **Self-signed** | free | none | anywhere | "Unknown publisher" still |

### SignPath Foundation

From their [terms](https://signpath.org/terms.html):

- OSI-approved license, **no commercial dual-licensing**, no proprietary
  components (system libraries excepted).
- Project must already be **released** in the form you want signed, and
  documented on its download page.
- Binaries must be **verifiably built from the repo**, and the signing
  request must come from a [trusted build
  system](https://docs.signpath.io/trusted-build-systems) — GitHub, GitLab,
  Azure DevOps, Jenkins, TeamCity, AppVeyor. A Gitea-only pipeline doesn't
  qualify, so the *release build* has to run on GitHub Actions, not merely be
  mirrored there. Yours already does.
- **"Verifiable reputation for executable programs"** is an explicit
  condition — this is the real gate for a young game, not a formality.
- Every release is **manually approved** by a trusted team member
  (Author/Reviewer/Approver roles); MFA required on both SignPath and the
  source repo.
- A **code-signing policy** statement on the project home page with their
  prescribed wording and a privacy-policy link.
- Metadata restrictions on signed binaries (product name = project name, one
  version per build).

Gotchas: the cert is issued to **SignPath Foundation** and lent to you, so
the dialog says "SignPath Foundation", not "Open Space Program". Acceptance
is discretionary, revocation can be retroactive, liability is disclaimed, and
no review/queue time is published — plan in weeks.

Open issue for us: their license clause covers "all components". `res/`
includes DejaVu (Bitstream Vera license), CC-BY skyboxes/heightmaps and
freesound audio under their own attribution files. That is not obviously
"OSI-approved for all components"; worth settling before applying, because a
rejection is a wasted round trip.

### Azure Artifact Signing (ex-Trusted Signing)

- Basic SKU $9.99/mo, 5,000 signatures/month (Premium SKU exists at 100k
  signatures). Billing starts at account creation and is **not pro-rated**.
  Free/trial/sponsored subscriptions are rejected; pay-as-you-go or EA only.
- **Country gate:** Public Trust certs for organisations are limited to the
  US, Canada, EU, UK, Australia, NZ, Japan, South Korea, Singapore,
  Switzerland, Norway, Israel. **Individual developers must be located in the
  US or Canada.** Outside that list, Public Trust isn't available at all.
- **Privacy:** no custom CN or O — CN is the validated legal name. For
  individual validation the certificate always displays city, state/province
  and country/region from your ID/billing address; street and postal code are
  opt-in via the certificate profile.
- **Signing client:** Microsoft's supported path is Windows SDK
  `signtool.exe` ≥ 10.0.2261.755 + .NET 8 + the Artifact Signing dlib, driven
  by a `metadata.json` whose `Endpoint` must match the account's region or you
  get a 403 / `SignerSign()` failure. **But** [jsign](https://ebourg.github.io/jsign/)
  has a first-class `TRUSTEDSIGNING` store type that signs PE from Linux:

  ```
  jsign --storetype TRUSTEDSIGNING --keystore weu.codesigning.azure.net \
        --storepass "$TOKEN" --alias <account>/<profile> osp.exe
  ```

  So this can ride on the existing Linux release runner — no
  `windows-latest`, no second toolchain.
- Certs live **3 days**: timestamping is mandatory, jsign enables it
  automatically, and Microsoft pushes `http://timestamp.acs.microsoft.com`.
- There is an official GitHub Action too, if we ever do want a Windows job.

### Commercial OV/EV

Sensible only if you want your own publisher name on the dialog and a company
to call. On Linux you'd drive the CA's cloud signer — jsign's `ESIGNER` store
type (SSL.com) or their CodeSignTool; DigiCert KeyLocker ships `smctl` and a
PKCS#11 module. Certum has historically sold a discounted OV product for
open-source projects; terms and pricing unverified here.

## Steam: what Valve does and doesn't do

### SteamPipe doesn't sign your exe

From [SteamPipe upload](https://partner.steamgames.com/doc/sdk/uploading):
builds come from `steamcmd +run_app_build`; files split into ~1 MB chunks;
unchanged chunks reused; new chunks "compressed and encrypted" and uploaded;
manifest + BuildID assigned. Nothing touches Authenticode. The only "sign"
in the doc is `InstallScript`, which is a Steam-side VDF signature for
installer scripts.

So your exe arrives on disk byte-identical to what you uploaded — unless you
enable the DRM wrapper. Valve is not your publisher of record for Authenticode
and Steam's reputation doesn't transfer to the file.

### Why Steam players usually skip the SmartScreen dialog

The app-reputation prompt is run by the shell at launch for files tagged with
Mark-of-the-Web — the `Zone.Identifier` ADS that browsers, mail clients and
Explorer's download manager attach. Steam's downloader doesn't tag files, and
the Steam client launches games with `CreateProcess` rather than through the
shell. Either factor plausibly suppresses the prompt; the commonly reported
experience is that Steam-launched games don't show it.

Mark this as **inference**: Valve documents none of it, and I found no
authoritative statement. What is not inference:

- Direct downloads (GitHub releases page, itch, a Discord link) are exactly
  the case where an unsigned exe hurts, and where a signature changes
  "Unknown publisher" to a name.
- Steam doesn't shield you from scanners. Defender's on-execute inspection,
  cloud-delivered protection, EDR and Smart App Control all run regardless of
  MOTW. Defender heuristics are the usual reason a static mingw exe gets
  quarantined, and that's channel-independent.
- SAC is the sharp edge: a shipped, well-known game was blocked in the field
  for being unsigned. That risk arrives with the Steam audience, not before.
- If you later ship the same build outside Steam, signing once stops you
  answering the same question twice.

### The Steam DRM wrapper vs. a signature

Steam's optional DRM wrapper is applied **server-side**: you post the exe,
Valve's back end injects a loader (and optional obfuscation), you get a
wrapped exe back. `drm_wrap` does this from a build script; there's also a
manual Security-tab flow. Valve's [DRM
FAQ](https://partner.steamgames.com/doc/features/drm) admits the consequence:
"the DRM injection process is modifying your binary in the same way a virus
might do," and the wrapper "has some obfuscation" that can be disabled in
compatibility mode.

Consequences if you ever enable it:

1. **Build → `drm_wrap` → sign → upload**, every build. The docs say "each
   build you intend to publish needs to be wrapped", so a one-time manual
   wrap doesn't carry forward.
2. Signing *before* wrapping doesn't just waste effort — the injection
   rewrites the PE, so the signature arrives broken, which is a worse signal
   to scanners than no signature at all.
3. Don't chain `drm_wrap` into a build script that also signs; you'll re-wrap
   the signed file and break it. Pick one order and keep it.
4. The wrapper is optional. `SteamAPI_RestartAppIfNecessary` covers the
   "must be launched through Steam" case without mutating the binary.

### Steam mechanics that matter for this repo

- `steamcmd` has a Linux build and the app build script is a `.vdf` with
  `ContentRoot` / `Depots` / `SetLive`. The existing Linux release runner can
  upload depots; no Windows host needed for the uncontroversial path.
- Branches are the QA story: upload to a password-protected beta branch, run
  your wine e2e smoke against the installed depot, then promote with
  `SetLive`. The default branch can't be set live by a script.
- The build account needs "Edit App Metadata" + "Publish App Changes To
  Steam", and a `config.vdf` login token for CI. To set a build live for a
  released app it also needs a phone number or the Steam Mobile App, and
  there's a **3-day lockout after any security change** on that account —
  that will bite a solo dev exactly once, at the worst time.
- Content is hashed/chunked, so "Verify integrity of game files" re-downloads
  a modified exe; signing after packaging and before upload is also the stable
  order for delta patches.
- Steam's Windows client is 64-bit only as of January 2026, which your build
  already assumes.

## Recommendation

Sequence, cheapest first:

1. **Now (free, one afternoon):** change nothing technical. Add a line to the
   Windows download notes: SmartScreen shows "More info → Run anyway", and
   users should only proceed after checking `SHA256SUMS`. Say explicitly that
   this is SmartScreen; a SAC block (`0x11C7`) has *no* "Run anyway" button —
   the only fixes are a reboot, a Defender folder exclusion, or turning SAC
   off. Keep `SHA256SUMS`, and file a [WDSI](https://www.microsoft.com/wdsi)
   false-positive report if Defender ever quarantines a build.
2. **When a stranger's first launch matters:** SignPath Foundation. It
   matches the license and release shape, costs nothing, and keeps the key
   off your machines. Budget for onboarding, a policy paragraph on the
   README, "SignPath Foundation" as the publisher string, and a manual
   approval click per release. The reputational gate is the risk; if they
   decline, fall through.
3. **Fallback if SignPath says no:** Azure Artifact Signing on the existing
   Linux runner via `jsign`, subject to the US/Canada individual gate and
   accepting your legal name plus city/country on the cert.
4. **Steam (whenever that happens):** signing is *not* a launch prerequisite,
   but it is the only durable answer to SAC. Skip the DRM wrapper, keep builds
   identical across channels, and if you do sign, apply it to the
   already-wrapped binary.

## Verified vs inferred

Verified against primary sources: SmartScreen behaviour and EV's demotion;
SAC's rules and default-on status; the MENACE `0x11C7` block; the CA/B HSM
requirement and its date; SteamPipe mechanics and the absence of exe signing;
the DRM wrapper's server-side mutation and per-build rule; build-account 2FA
and lockout; Azure's price, quota, country gate, cert fields, 3-day validity
and timestamping; jsign's `TRUSTEDSIGNING` support; SignPath's terms.

Inferred, not sourced: that Steam's lack of MOTW tagging (and its
`CreateProcess` launch) is *why* the SmartScreen prompt usually doesn't
appear; that a broken signature is worse than none; that Defender false
positives are the usual failure mode for static mingw exes. The first one is
the one to re-check if a player ever reports a SmartScreen dialog on a Steam
build.

## Open questions / risks

- **SignPath's reputation gate.** "Verifiable reputation for executable
  programs" is a judgement call by a reviewer; a young AGPL space game with a
  few hundred downloads may not clear it. Ask before doing the policy-page
  work, or do the cheap parts first.
- **Asset licensing vs "all components OSI".** `res/` mixes DejaVu, CC-BY and
  freesound audio. Either that's fine as "system libraries"/content, or it
  blocks the application. Unresolved.
- **`drm_wrap` on Linux.** Docs describe a `steamcmd` command using
  `drmtoolp`; whether the Linux `steamcmd` ships it is unverified. The winvm
  (`10.235.219.101`) is the fallback if we ever need to wrap.
- **jsign's quota accounting.** It makes an extra call to fetch the cert
  chain, which reportedly counts against the 5,000/month unless files are
  batched into one invocation. Cheap to design for, worth knowing.
- **`VERSIONINFO` consistency.** SignPath requires one product version across
  the signed binaries *in a build* — per-release bumps are normal. The thing
  to check is that `release/osp.rc`'s numeric MAJ.MIN.PATCH and the string
  version agree; today only the first three components move.
- **SAC on the winvm.** If we ever want to reproduce a `0x11C7` block, SAC is
  per-device and Windows 11-only; worth checking the VM's state before
  assuming a repro is possible.

## Sources

- SmartScreen reputation — <https://learn.microsoft.com/en-us/windows/apps/package-and-deploy/smartscreen-reputation>
- Smart App Control overview — <https://learn.microsoft.com/en-us/windows/apps/develop/smart-app-control/overview>
- Smart App Control FAQ — <https://support.microsoft.com/en-us/windows/security/threat-malware-protection/smart-app-control-frequently-asked-questions>
- MENACE `0x11C7` report — <https://steamcommunity.com/app/2432860/discussions/0/568176211023434607/>
- CA/B Forum code signing requirements — <https://cabforum.org/working-groups/code-signing/requirements/>
- Azure Artifact Signing — <https://learn.microsoft.com/en-us/azure/artifact-signing/overview>,
  [FAQ](https://learn.microsoft.com/en-us/azure/artifact-signing/faq),
  [integrations](https://learn.microsoft.com/en-us/azure/artifact-signing/how-to-signing-integrations),
  [quickstart](https://learn.microsoft.com/en-us/azure/artifact-signing/quickstart)
- jsign (TRUSTEDSIGNING / ESIGNER) — <https://ebourg.github.io/jsign/>
- SignPath Foundation terms — <https://signpath.org/terms.html>,
  <https://docs.signpath.io/trusted-build-systems>
- SteamPipe upload — <https://partner.steamgames.com/doc/sdk/uploading>
- Steam DRM wrapper — <https://partner.steamgames.com/doc/features/drm>
