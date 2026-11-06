# Plans and known limits

## Current work

No active implementation plan. Usage and maintenance instructions live in [README](../README.md),
[the full guide](../docs/reference.md) and [browser delivery](../docs/web-delivery.md).

## Future work

Create an ExecPlan from [TEMPLATE.md](TEMPLATE.md) when starting a substantial change.

1. **Follow-mode density at 100,000+ vehicles.** Dense follow flocks can overlap under seek pressure
   despite complete separation queries. This requires a behavior decision, rather than increasing
   alignment/cohesion sample caps.
2. **Rendering budgets for heavy custom models and other GPUs.** Exact vertex sharing improves
   imported-model rendering, but heavily faceted models remain expensive. Measure before adding
   approximate LOD, culling or extra render passes. Cross-device testing remains open.
3. **Hunter overhead at one million prey in wander mode.** Dense prey can make uncapped neighbor
   searches expensive. Profile hunt, flee/panic and steering separately before changing them.
   Normal showcase populations are much smaller.

Retain existing speed defaults, model colors in the Fish preset, and disabled volumetric light and
motion blur in showcase presets unless a task explicitly changes them.
