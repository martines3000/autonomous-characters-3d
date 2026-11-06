# Fish schools, shark Ride and loading review handoff

The owner-authorized follow-up is prepared in four ready, unmerged PRs. Merge after #20 in this order:

| Order | PR | Result | Recorded parent tip |
| --- | --- | --- | --- |
| 1 | [#22](https://github.com/martines3000/autonomous-characters-3d/pull/22) | Three moving Reef shoals; public/Lab lifecycle and pointer regressions | `345ce0eb1450ff4c22b11d1873eafe7f3da982a9` |
| 2 | [#23](https://github.com/martines3000/autonomous-characters-3d/pull/23) | Coherent guided spawn, translating/turning Shark school and encounter framing | `5977c1568ecd5bf84ce7e3a9c9a207c31aa96514` |
| 3 | [#24](https://github.com/martines3000/autonomous-characters-3d/pull/24) | Actual CPU/GPU shark Ride, Chase/First person, Fish/Shark and another target | `937c2b889c1943a66111c7fa16423c683fa7c808` |
| 4 | [#25](https://github.com/martines3000/autonomous-characters-3d/pull/25) | Truthful web progress, accessible reveal and this completed handoff | `7e4b0c67417599e28a2e6ce7f90b3ca13678a037` |

Use the [morning index](2026-11-06-morning-review.md) for the complete #12–#25 squash/rebase order and
the linked deployment drafts. No merge, registry publication, production workflow, deployment,
infrastructure apply or DNS change occurred.

## Evidence and source

[Immutable evidence](https://github.com/martines3000/autonomous-characters-3d/tree/8a70142b71f3793f72a4c6549bd307de22fdd06c/fish-schools)
contains the complete report, before/after stills, 25-second Reef clips, a 60-second Shark clip,
12-second shark Chase/First person clips, loading/reveal video, raw logs, checksums and measurements.

Native integrated source is `4fc7b7b1d7b12a0d545cfad6713653a13f849db8`; clean browser source is
`c7fa288d22e2afaf8628c59c931aadbb9bfc9073`, differing only in one README line. Release
`596a0586e1010946ad95daab` has 32 byte-identical files across two fresh output builds. Later handoff
commits change documentation only. Local image ID `sha256:8f67b21307b200f17b2a9ba747bea7aa0683aa3b6531f227f18231c20dc46967`
is a local review artifact, not a published registry digest.

Formatting, **255 release tests**, **24 real-device GPU checks**, warning-free build, native smoke,
diff checks, **nine loader unit tests**, **one packaging fixture**, and **11 headed browser cases**
pass. Native/WASM Clippy retain the seven existing warnings. The GPU checks include force, turn,
guided spawn, navigation precedence, 720-byte ABI and actual moving/paused/empty-prey hunter tracking.

The full browser matrix uses an isolated headed X11/Xvfb display and the AMD RDNA 3 WebGPU adapter;
the ordinary desktop was in use and physical mouse/wheel events contaminated initial gesture runs.
The isolated run retains strict pixel equality. Normal/reduced loading and failure during reveal are
also verified in headed Wayland. See [web delivery](../../docs/web-delivery.md) and the immutable report
for exact flags, display, device and package provenance. No headless black frame is accepted as proof.

## Results and limits

Reef same-shoal nearest-neighbor fractions are 99.64–99.82% across its three tiers. Shark mean
elongation is 3.94/2.51/3.68 at Low/Medium/High; mean sampled median spacing is 4.78/4.33/4.56 units,
versus 1.41/1.38/1.33 in the pinned origin-target baseline. Panic is lower in these diagnostic windows.
The school sometimes compresses: Medium has a genuinely compact phase while still travelling.
Continuous ribbon shape, hard confinement and visibility of every hunter are not promised. Maximum
sampled outside fractions are 0.10%/0%/0.045%; some tails and hunters reach frame edges. No biological
accuracy or owner taste approval is claimed. No observed asset defect required Blender.

On Ryzen 9 7900X/RX 7900 XT, native Medium after means are Reef 1.733/1.642 ms and Shark
2.081/2.138 ms at 1853×2085. Before windows used 846×1390 and different machine loads, so no direct
speed comparison is claimed. All nine browser scene/tier combinations were measured twice at
1920×1080: means 16.664–20.151 ms, p95 17.7–23.8 ms, maximum 39.2 ms. Xvfb presentation differs from
Wayland/native. These are application/browser intervals, not GPU timestamps or capacity estimates.

The [complete immutable report](https://github.com/martines3000/autonomous-characters-3d/blob/8a70142b71f3793f72a4c6549bd307de22fdd06c/fish-schools/README.md)
states seeds, warmup, sample windows, resolutions, loads, sources and remaining limitations.
Raw PNG sequences and diagnostic trials remain under `/home/martines/Projects/ac3d-evidence/2026-11-06-fish-schools/`.

The current localhost preview is `http://127.0.0.1:8091`; the normal recorded Wayland/Vulkan flags
are needed on this machine. Other engines/devices/mobile, internet loading and CRONA capacity
remain untested. Application review/merge, separately authorized publication of the actual merged
source, a real digest, internal health and later public verification remain release prerequisites.
