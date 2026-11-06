# Public Flock release — 6 November 2026

Flock is live at **[https://flock.skippy-ai.com/](https://flock.skippy-ai.com/)**. The owner authorized publication, household PR adjustments/merges and deployment after the application stack merged. The existing protected-host, deny-ingress, GitOps and provider apply controls remain in place.

## Source and delivery

| Item | Verified value |
| --- | --- |
| Published application source | `57fa57abe53ce2a23717c4b4f0816f40fcc141c3`, clean `master` |
| Published image | `ghcr.io/martines3000/autonomous-characters-3d-web@sha256:897efa64a1b384ddb8bb43085b4600e88098e29fff60ceaeed407dad5675991f` |
| Platform / browser content | Linux/amd64 / `8ccdbb8b6a861d290b652024` |
| Workload merge | [household-cluster #1](https://github.com/martines3000/household-cluster/pull/1), `c4d8973333c6c4cf6fe4e8ed1918d135ab0b1e1d` |
| Routing merge | [household-infra #1](https://github.com/martines3000/household-infra/pull/1), `91edcc62a3c35e9c9c856d231ce369e155753b93` |

The [publication workflow](https://github.com/martines3000/autonomous-characters-3d/actions/runs/42259142094) built the actual merged source. Native asset baking took 12m 52s and the WASM build took 8m 45s on that first clean hosted build. The returned registry digest was read from its successful publication, then independently resolved and fully pulled with an empty Docker credential configuration. No new pull Secret was needed.

The cluster manifest pins that registry digest. Public `/release.json` and immutable files match the exact image. Direct container-runtime image-ID inspection was unavailable; the registry pin, public metadata and byte comparisons are separate evidence, not a claim that private inspection occurred.

## Infrastructure and rollout

The [baseline production plan](https://github.com/martines3000/household-infra/actions/runs/42259520714) refreshed the managed host/Tunnel and reported no changes. Before routing, both authoritative Cloudflare nameservers returned NXDOMAIN for Flock and historical AC3D A/AAAA/CNAME queries. The cluster render delta was exactly four additions: restricted Namespace, Deployment, ClusterIP Service and internal Traefik Ingress. Existing resource specifications were unchanged; no PVC, plaintext Secret, public Service or host port was introduced.

The [full routing plan](https://github.com/martines3000/household-infra/actions/runs/42263983404) was reviewed against unchanged source `91edcc6`. It contained only one new proxied Flock CNAME and one in-place shared Tunnel configuration update. The existing Beszel/Life OS rules and trailing 404 were preserved. There were no host, firewall, Access, unrelated DNS, deletion or replacement actions. The normal plan safety policy passed.

The [guarded apply](https://github.com/martines3000/household-infra/actions/runs/42264271963) created and checked a fresh plan with the existing confirmation. At **17:44:22 UTC**, it completed with **1 added, 1 changed, 0 destroyed**. No provider/backend credentials were copied to the workstation. Flux deploys the committed workload with readiness/liveness probes and `wait: true` retained.

## Public and exact-image proof

- The exact image passes **20 local HTTP paths** under UID/GID 101, read-only root, no capabilities, only a 32Mi writable `/tmp`, 250m CPU and 128Mi memory.
- The public HTTPS endpoint passes the same **20 HTTP paths**: `/healthz`, HTML, source/release/catalog metadata, generated WASM/JS/shader/font/environment files and explicit missing-asset errors. Checks cover MIME, gzip, cache policy and security headers. Normal certificate/hostname validation remained enabled.
- All **15 immutable public files** have the same decoded SHA-256 and byte length as the verified image. Release metadata is identical, including clean source and content hash.
- The exact-image and public site each pass **11 headed Chromium/WebGPU cases**. Public acceptance completed at **17:48:37 UTC**, with actual nonblank scenes, camera/input, pause, sizing, unsupported-device explanations and real GPU-process loss/recovery.
- Both authoritative nameservers now return Flock A/AAAA answers; historical `ac3d.skippy-ai.com` remains NXDOMAIN.
- Existing Beszel requests still redirect to the same Cloudflare Access login path. Life OS `/health` remains `200` with `{"status":"ok"}` before and after the apply.

Browser evidence uses Chromium **155.0.8059.84**, Linux x86_64, isolated headed X11/Xvfb **21.1.25-2**, AMD Radeon RX 7900 XT, RADV **26.2.5**, and the recorded Vulkan/X11 flags in [web delivery](../../docs/web-delivery.md). Screenshots were visually inspected. No headless black frame is treated as success. These are functional checks, not new GPU/CRONA capacity measurements.

Cloudflare rejects Python urllib's default signature with error 1010, so HTTP probes record a normal browser User-Agent. The actual headed browser passes separately. No Cloudflare protection was disabled for these checks.

The final repository gate also passes: formatting, **255 release tests** (27 ignored), warning-free release build, ten-second native startup and diff checks. Cargo compiled the expected application checkout with four jobs under the shared lock. The earlier 24 real-device GPU checks remain source-matched; this deployment did not change runtime code or rerun those ignored checks. Native gate log: `/home/martines/Projects/ac3d-evidence/2026-11-06-public-release/final-native-gate.log`.

## Artifacts

[Immutable release evidence](https://github.com/martines3000/autonomous-characters-3d/tree/1ede4c904a1a4312ff8e91d067038b656745e3f1/public-release-2026-11-06) contains before/after DNS, publication and plan records, local/public HTTP and browser reports, decoded-byte comparisons and real public screenshots. Cookie values and Access redirect query strings are redacted from report copies. Workflow links above retain the actual publication/plan/apply logs.

![Public Reef drift](https://raw.githubusercontent.com/martines3000/autonomous-characters-3d/1ede4c904a1a4312ff8e91d067038b656745e3f1/public-release-2026-11-06/scene-reef.png)

Large local logs and the complete screenshot set remain under `/home/martines/Projects/ac3d-evidence/2026-11-06-public-release/`.

## Limits and deferred verification

Workstation SSH authentication to CRONA was rejected and no CRONA kubeconfig was configured. The owner questioned SSH and supplied the GitHub deployment repositories; this first installation used the recorded GitHub/Flux verification path. Direct node architecture, pod/Flux state, private Host-header serving and running image-ID inspection are **unavailable**, not passed. No private credential or host configuration was changed to obtain access.

The guarded full plan verifies preservation of the managed deny-ingress firewall and protected host. Direct external IPv4/IPv6 origin-port probes were **not run**; no new network-level success is claimed. Deferred operations follow-up: with suitable read access, record CRONA architecture, Flux revision, pod image ID and internal serving; with the actual origin addresses and working dual-stack controls, run the household network-verification procedure. This follow-up is tracked here and does not reopen application implementation.

Other browser engines, mobile devices, GPUs and production capacity remain unmeasured. Earlier school-shape and rendering limits in the [follow-up report](2026-11-06-fish-schools.md) still apply. The release does not claim biological accuracy or new visual taste approval.

## Recovery

This first installation has no earlier application digest. Later normal application rollback restores the previous verified digest through the cluster GitOps repository. Route withdrawal can remove only the Flock Tunnel hostname rule while retaining DNS, existing routes and the final 404, followed by the normal full plan/guarded apply. DNS deletion or namespace pruning requires separately reviewed recovery authorization; do not weaken those guards or remove the shared Tunnel.
