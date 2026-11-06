# Publish and deploy Flock to the household platform

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Serve the merged Flock application at `https://flock.skippy-ai.com` through CRONA's internal Traefik and existing Cloudflare Tunnel. On 2026-11-06 the owner explicitly authorized adjusting and merging the household PRs, publication, deployment and public routing. This starts a new release session after the completed review-only plans.

## Progress

- [x] (2026-11-06 17:08Z) Confirmed merged source `57fa57abe53ce2a23717c4b4f0816f40fcc141c3` is byte-identical to the tested final review tree. Household PRs #1 remain drafts with an unpublished-image prerequisite.
- [x] (2026-11-06 17:08Z) Read application delivery guidance and both household repositories' agent, contribution, architecture, security and rollout instructions. Confirmed hostname and protected CRONA identity.
- [ ] Publish the actual default-branch source; record its immutable GHCR digest and pull access.
- [ ] Verify that exact image locally; update, validate and merge household-cluster PR #1.
- [ ] Verify CRONA architecture, Flux rollout, deployed digest and internal HTTP/assets.
- [ ] Recheck authoritative hostname and existing Tunnel/zone configuration; validate and merge household-infra PR #1.
- [ ] Review the full main-branch production plan; run the existing guarded apply for the authorized route.
- [ ] Verify public HTTPS/assets, headed WebGPU interaction, existing services and public-origin ingress controls.
- [ ] Record release evidence and recovery, add the live URL to the short README and close this plan.

## Surprises & Discoveries

- Unrelated `lutralabs/household` repositories also exist here. Release checkouts are `ac3d-deploy-cluster` and `ac3d-deploy-infra`, with `martines3000` remotes. Do not use the unrelated cluster or kubeconfig.
- Initial noninteractive SSH to `root@crona` stopped at host-key verification. Establish the correct Tailscale identity and trusted key before using private access.

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

## Assumptions and Open Questions

- Assumption: Existing GitHub production credentials and private CRONA access are usable. Validate actual workflow results, trusted OpenSSH/Tailscale identity and exactly the intended `crona` node. If unavailable, continue independent checks and request only the missing access or platform approval.
- Assumption: `flock.skippy-ai.com` remains unassigned. Repeat authoritative DNS checks and inspect the authoritative provider plan. Resolve any conflict without overwriting another service.

## Outcomes & Retrospective

Release in progress. No published digest, merged household change or production success is claimed yet.

## Context and Orientation

`.github/workflows/publish-web-image.yml` builds clean `master` and publishes Linux/amd64 with source metadata. `docs/web-delivery.md` defines container/browser verification. Cluster PR #1 adds `apps/flock`, its restricted namespace and internal ingress. Infrastructure PR #1 adds one hostname, proxied CNAME and shared Tunnel rule. No Access login or public origin port is needed.

## Plan of Work

1. Dispatch default-branch publication and establish private read access while it runs.
2. Pin the returned digest, verify pull and the restricted container, validate rendered cluster additions and merge C1.
3. Let Flux reconcile; diagnose only if delayed. Prove internal health/assets and image provenance before exposure.
4. Complete I1 review and merge. Dispatch a full production plan and inspect every action. Apply the authorized route with normal confirmation and guards.
5. Verify public HTTPS, visible interactive WebGPU, existing services and closed direct-origin ports. Record evidence and recovery.

## Concrete Steps

Dispatch `gh workflow run publish-web-image.yml --repo martines3000/autonomous-characters-3d --ref master`; inspect the successful run's actual digest. In each household checkout run every command in `CONTRIBUTING.md`, inspect the diff and update the existing PR. Use the documented main-branch OpenTofu `operation=plan`; review the full plan, then `operation=apply` with `confirmation=apply-household-infra` only for the authorized safe delta.

Use the documented private SSH/Kubernetes tunnel, exact-image local container, HTTP checks and headed browser scripts. Serialize local builds/apps/measurements with `/tmp/ac3d-heavy.lock` and at most four Cargo jobs.

## Validation and Acceptance

- Publication succeeds from the actual merged source; registry pull and release metadata prove provenance.
- Cluster contribution checks pass; rendered additions preserve existing resources and add no public Service, PVC or plaintext Secret.
- The intended CRONA node reports `amd64`; Flux, pod health, digest and internal Host-header asset checks pass.
- Full OpenTofu plan preserves server/firewall/Access/other routes and contains only the Flock DNS/routing delta; guarded apply succeeds.
- Public HTTPS, cache/MIME/compression, assets and a nonblank interactive scene pass on the recorded WebGPU configuration. Existing services retain expected behavior; direct-origin ingress stays closed.

## Idempotence and Recovery

Reuse successful publication for the same source. Commit desired state separately to each repository. First installation has no previous image: stop before exposure if unhealthy. Later application rollback restores its prior verified digest through GitOps. Route withdrawal may retain DNS and remove only the Flock Tunnel rule. Never weaken destructive-action guards, remove the shared Tunnel or prune unrelated resources.

## Artifacts and Notes

Record the release in `plans/evidence/2026-11-06-public-release.md`; large logs, screenshots and temporary credentials remain outside Git. Prior source-matched evidence is linked from `plans/evidence/2026-11-06-fish-schools.md`.

## Interfaces and Dependencies

No application behavior or dependency change is planned. Dependencies are the clean-source image workflow, public GHCR image or approved encrypted pull credentials, Flux, protected CRONA, private Tailscale/OpenSSH access and the guarded OpenTofu workflow.

---

Revision notes:

- 2026-11-06: Started owner-authorized production release after the application stack merged.
