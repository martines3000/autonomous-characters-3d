# Publish and deploy Flock to the household platform

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Serve the merged Flock application at `https://flock.skippy-ai.com` through CRONA's internal Traefik and existing Cloudflare Tunnel. On 2026-11-06 the owner explicitly authorized adjusting and merging the household PRs, publication, deployment and public routing. This starts a new release session after the completed review-only plans.

## Progress

- [x] (2026-11-06 17:08Z) Confirmed merged source `57fa57abe53ce2a23717c4b4f0816f40fcc141c3` is byte-identical to the tested final review tree. Household PRs #1 remain drafts with an unpublished-image prerequisite.
- [x] (2026-11-06 17:08Z) Read application delivery guidance and both household repositories' agent, contribution, architecture, security and rollout instructions. Confirmed hostname and protected CRONA identity.
- [x] (2026-11-06 17:34Z) Publication run `42259142094` succeeded from clean `57fa57a`; digest `sha256:897efa64a1b384ddb8bb43085b4600e88098e29fff60ceaeed407dad5675991f`, content `8ccdbb8b6a861d290b652024`, Linux/amd64. Anonymous manifest lookup and full pull pass.
- [x] (2026-11-06 17:40Z) The exact image passes 20 HTTP paths and 11 isolated headed Chromium/WebGPU cases at the workload restrictions. Full cluster checks pass; PR #1 merged as `c4d8973333c6c4cf6fe4e8ed1918d135ab0b1e1d`.
- [x] (2026-11-06 17:40Z) Direct CRONA architecture, Flux, pod image-ID and internal HTTP checks are explicitly unavailable without a usable private credential. Per the GitHub/Flux release decision below, retain readiness/liveness and Flux wait; public source, health, asset and browser proof remains required. Record this limit in the final release evidence.
- [x] (2026-11-06 17:41Z) Fresh authoritative DNS still returns NXDOMAIN on both nameservers; prior full refresh showed no Tunnel drift. All infrastructure contribution checks and CI pass; PR #1 merged as `91edcc62a3c35e9c9c856d231ce369e155753b93`.
- [x] (2026-11-06 17:44Z) Reviewed full main-branch plan `42263983404`: exactly one Flock DNS creation, one in-place Tunnel update and zero deletions/unrelated actions. Guarded apply `42264271963` passed its fresh-plan safety policy and completed with one added, one changed, zero destroyed.
- [x] (2026-11-06 17:48Z) Public HTTPS passes 20 HTTP paths and 11 headed WebGPU cases; all 15 immutable files match the exact image. Beszel Access and Life OS health remain intact; authoritative Flock answers exist and historical AC3D remains absent. Managed host/firewall preservation passes full-plan policy. Direct origin-port probes are deferred in the release report because origin address/dual-stack verification access is unavailable; no network probe success is claimed.
- [x] (2026-11-06 17:55Z) Published immutable evidence `1ede4c9`, recorded release/recovery and deferred checks, added the demo link to the 34-line README, and closed this plan. Final formatting, 255 release tests, warning-free build, native startup and diff checks pass.

## Surprises & Discoveries

- Unrelated `lutralabs/household` repositories also exist here. Release checkouts are `ac3d-deploy-cluster` and `ac3d-deploy-infra`, with `martines3000` remotes. Do not use the unrelated cluster or kubeconfig.
- Initial noninteractive SSH to `root@crona` stopped at host-key verification. Establish the correct Tailscale identity and trusted key before using private access.
- Authenticated Tailscale inventory resolves online `crona` to `100.103.65.58`. A task-scoped known-hosts file established first-use SSH trust on that private transport, but the available workstation key is rejected. No CRONA kubeconfig is configured; only the unrelated `household` context exists. The owner asked why access was needed; it is for internal live verification, while publication and rollout use GitHub/Flux.
- Baseline main-branch production plan `42259520714` succeeded with no changes and passed the existing safety policy. Both authoritative nameservers returned authoritative NXDOMAIN for Flock and historical AC3D A/AAAA/CNAME queries. Render comparison confirms exactly four cluster additions, no removals and no changed existing specifications.
- Cloudflare rejects the default Python urllib browser signature with error 1010. Recording a normal browser User-Agent makes the HTTP verification representative of the actual client; the real headed browser passes separately. No Cloudflare protection was disabled.
- The clean hosted publication needed 12m 52s for native baking compilation and 8m 45s for WASM. Reuse the successful immutable publication for this source.

## Decision Log

- Decision: Publish, roll out internally, then expose the route, in that order.
  Rationale: Each step depends on a real digest or observed health, making subsequent changes reviewable.
  Date: 2026-11-06.
- Decision: Use existing GitHub publication/OpenTofu workflows and Flux GitOps.
  Rationale: Preserve credential handling, plan guards, protected server identity and deny-ingress firewall.
  Date: 2026-11-06.
- Decision: Reuse source-matched native/GPU/browser evidence, then verify the published image and public endpoint.
  Rationale: Squash merges changed history but not the tested tree; deployment needs new serving/provenance checks.
  Date: 2026-11-06.
- Decision: Use GitHub/Flux and public verification without making workstation SSH access a deployment prerequisite.
  Rationale: The owner questioned SSH and supplied the GitHub deployment repositories. This is a first installation of a public static site; retain Deployment readiness/liveness and Flux wait, verify the exact published image locally, then verify actual public health/assets and browser behavior. Record private rollout/digest inspection as unavailable rather than claiming it passed. Stop or correct the route through normal GitOps if public verification fails.
  Date: 2026-11-06.

## Assumptions and Open Questions

- Assumption: Existing GitHub production credentials and private CRONA access are usable. Validate actual workflow results, trusted OpenSSH/Tailscale identity and exactly the intended `crona` node. If unavailable, continue independent checks and request only the missing access or platform approval.
- Assumption: `flock.skippy-ai.com` remains unassigned. Repeat authoritative DNS checks and inspect the authoritative provider plan. Resolve any conflict without overwriting another service.

Resolved: production workflow credentials work; private workstation CRONA access does not. The GitHub/Flux verification decision applies, with deferred checks tracked in the release report. Fresh DNS and the reviewed provider plan found no hostname conflict; public DNS and serving now pass.

## Outcomes & Retrospective

Flock is live at `https://flock.skippy-ai.com/`. Publication, both household merges and guarded production apply succeeded. Exact-image local and actual public acceptance each pass 20 HTTP paths and 11 headed browser cases. The public immutable files match the published image and other services retain their expected responses. Private cluster and direct-origin network inspection remain explicitly unavailable, with follow-up recorded in the release report. Final formatting, 255 release tests, warning-free release build, ten-second native startup and diff checks pass. Documentation and archival are complete; no application behavior or dependency changed.

## Context and Orientation

`.github/workflows/publish-web-image.yml` builds clean `master` and publishes Linux/amd64 with source metadata. `docs/web-delivery.md` defines container/browser verification. Cluster PR #1 adds `apps/flock`, its restricted namespace and internal ingress. Infrastructure PR #1 adds one hostname, proxied CNAME and shared Tunnel rule. No Access login or public origin port is needed.

## Plan of Work

1. Dispatch default-branch publication and establish private read access while it runs.
2. Pin the returned digest, verify pull and the restricted container, validate rendered cluster additions and merge C1.
3. Let Flux reconcile under the existing readiness probes and Flux wait. Use private rollout/internal asset checks if access is available. Otherwise retain that explicit verification limit and proceed with the exact-image local proof and live public checks for this owner-authorized first installation.
4. Complete I1 review and merge. Dispatch a full production plan and inspect every action. Apply the authorized route with normal confirmation and guards.
5. Verify public HTTPS, visible interactive WebGPU and existing services. Prove managed firewall preservation through the full plan; record unavailable private/network checks explicitly. Record evidence and recovery.

## Concrete Steps

Dispatch `gh workflow run publish-web-image.yml --repo martines3000/autonomous-characters-3d --ref master`; inspect the successful run's actual digest. In each household checkout run every command in `CONTRIBUTING.md`, inspect the diff and update the existing PR. Use the documented main-branch OpenTofu `operation=plan`; review the full plan, then `operation=apply` with `confirmation=apply-household-infra` only for the authorized safe delta.

Use the documented private SSH/Kubernetes tunnel, exact-image local container, HTTP checks and headed browser scripts. Serialize local builds/apps/measurements with `/tmp/ac3d-heavy.lock` and at most four Cargo jobs.

## Validation and Acceptance

- Publication succeeds from the actual merged source; registry pull and release metadata prove provenance.
- Cluster contribution checks pass; rendered additions preserve existing resources and add no public Service, PVC or plaintext Secret.
- With private access, verify CRONA `amd64`, Flux, pod health, digest and internal Host-header assets directly. Without it, record these checks as unavailable; prove exact published-image health locally and actual release provenance, health and assets through the public hostname without claiming direct cluster inspection.
- Full OpenTofu plan preserves server/firewall/Access/other routes and contains only the Flock DNS/routing delta; guarded apply succeeds.
- Public HTTPS, cache/MIME/compression, assets and a nonblank interactive scene pass on the recorded WebGPU configuration. Existing services retain expected behavior. The managed deny-ingress firewall remains unchanged; direct network probes are deferred explicitly when unavailable.

## Idempotence and Recovery

Reuse successful publication for the same source. Commit desired state separately to each repository. First installation has no previous image: stop before exposure if unhealthy. Later application rollback restores its prior verified digest through GitOps. Route withdrawal may retain DNS and remove only the Flock Tunnel rule. Never weaken destructive-action guards, remove the shared Tunnel or prune unrelated resources.

## Artifacts and Notes

Record the release in `plans/evidence/2026-11-06-public-release.md`; large logs, screenshots and temporary credentials remain outside Git. Prior source-matched evidence is linked from `plans/evidence/2026-11-06-fish-schools.md`.

## Interfaces and Dependencies

No application behavior or dependency change is planned. Dependencies are the clean-source image workflow, public GHCR image or approved encrypted pull credentials, Flux, protected CRONA, private Tailscale/OpenSSH access and the guarded OpenTofu workflow.

---

Revision notes:

- 2026-11-06: Started owner-authorized production release after the application stack merged.

- 2026-11-06: Closed after successful publication, household merges, guarded apply and actual public acceptance. Deferred private/network checks are explicitly tracked in the release report.
