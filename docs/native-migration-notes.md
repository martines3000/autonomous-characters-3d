# Native migration API notes

Verified 2026-10-30 against **Bevy 0.20.1** and **bevy_egui 0.43.1** tagged upstream source. These notes concern the local native application. API snippets are migration guidance, not a claim that the application has compiled or passed interactive checks.

## Camera: built-in pan-orbit support needs an input adapter

Enable Bevy's `pan_orbit_camera` feature. Import `bevy::camera_controller::pan_orbit_camera::prelude::*`, add `DefaultPanOrbitCameraPlugins`, and spawn `Camera3d`, `PanOrbitCamera`, and a `Transform` together. The existing eye position `(-2, 5, 5)` can use a transform looking at the origin; initialize anchor depth to the eye-to-origin distance. [Feature declaration](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_camera_controller/Cargo.toml), [minimal example](https://github.com/bevyengine/bevy/blob/v0.20.1/examples/camera/pan_orbit_camera_minimal.rs).

**Important:** the plugin group does not provide mouse input mapping, despite some module prose describing it as a default-input group. The tagged minimal example explicitly adds its sibling `CustomInputPlugin`. Either adapt that first-party example or write a small single-camera adapter. This avoids adding another camera crate. [Actual plugin-group implementation](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_camera_controller/src/pan_orbit_camera/mod.rs), [input adapter example](https://github.com/bevyengine/bevy/blob/v0.20.1/examples/camera/pan_orbit_camera_custom_input_plugin.rs).

Available controller methods:

```rust
PanOrbitCamera::default().with_initial_anchor_depth(54.0_f64.sqrt());
// On a queried mutable controller:
controller.start_orbit(None); // optional anchor is in VIEW space, Option<DVec3>
controller.start_pan(None);
controller.start_zoom(None);
controller.send_screenspace_input(mouse_delta); // Vec2, pixel motion
controller.send_zoom_input(scroll_amount);      // f32
controller.end_move();
```

Call start only when entering a motion, send input while active, then end when released. `None` uses the remembered anchor depth. Configure `sensitivity` through `Sensitivity { orbit: Vec2, zoom: f32 }`; there is no old `mouse_rotate_sensitivity` field. Schedule inputs in `PreUpdate` before `PanOrbitCamera::update_camera_positions`; the official adapter runs after `PickingSystems::Last`. [Component and methods](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_camera_controller/src/pan_orbit_camera/controller/component.rs), [schedule ordering](https://github.com/bevyengine/bevy/blob/v0.20.1/examples/camera/pan_orbit_camera_custom_input_plugin.rs).

## Picking and target selection

`DefaultPlugins` already includes `DefaultPickingPlugins`; add `MeshPickingPlugin` separately. **Use flattened events such as `On<PointerClick>`, not `On<Pointer<Click>>`.** `PointerClick` exposes `entity`, `button`, `hit`, `duration`, and `count`; its pointer metadata is in `pointer`. [Default plugin group](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_internal/src/default_plugins.rs), [mesh example](https://github.com/bevyengine/bevy/blob/v0.20.1/examples/picking/mesh_picking.rs), [event definitions](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_picking/src/events.rs).

Replace the removed third-party `Selection` with app-owned selection state. A global primary-click observer can set selection according to whether `click.original_event_target()` has `Target`. The built-in window backend provides a background hit behind scene entities, so clicking empty window space can clear selection. Pointer events also propagate through parents and ultimately to the window; account for propagation when implementing entity-specific observers. [Window backend](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_picking/src/window.rs), [propagation implementation](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_picking/src/events.rs).

For this app's potentially large flock, opt in only the target rather than raycasting every vehicle mesh:

```rust
use bevy::picking::mesh_picking::{MeshPickingCamera, MeshPickingSettings};
// app.insert_resource(MeshPickingSettings { require_markers: true, ..default() });
// Camera entity: insert MeshPickingCamera.
// Target entity: insert Pickable::default().
```

**Source beats stale example prose:** `require_markers: true` checks `MeshPickingCamera` on cameras and `Pickable` on mesh entities. Default settings pick every visible mesh retaining CPU data, and `Pickable::IGNORE` excludes individual meshes. [Exact backend implementation](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_picking/src/mesh_picking/mod.rs).

## glTF and 3D entities

**Do not use `SceneRoot` / `Handle<Scene>` from older migration guides.** In 0.20.1 the asset type is `WorldAsset`; the component is `WorldAssetRoot(Handle<WorldAsset>)`. The new `Scene` refers to BSN scene composition. Keep the existing cone model and spawn its loaded world asset together with the vehicle transform/components:

```rust
// RenderState field:
vehicle_scene: Handle<WorldAsset>,
// Loading:
render_state.vehicle_scene = asset_server.load("cone.glb#Scene0");
// Spawning:
commands.spawn((WorldAssetRoot(render_state.vehicle_scene.clone()), transform));
```

The typed alternative is `asset_server.load(GltfAssetLabel::Scene(0).from_asset("cone.glb"))`. [World asset component](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_world_serialization/src/components.rs), [glTF usage](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_gltf/src/lib.rs), [new Scene concept](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_scene/src/lib.rs).

Replace `PbrBundle` with `(Mesh3d(mesh_handle), MeshMaterial3d(material_handle), Transform, ...)`. Geometry equivalents include `Cuboid::from_length(1.0)` and `Sphere::new(2.0).mesh().ico(3).unwrap()`. These components receive required companion components automatically. [Tagged mesh-picking example](https://github.com/bevyengine/bevy/blob/v0.20.1/examples/picking/mesh_picking.rs).

The ambient resource is now `GlobalAmbientLight { color, brightness, ..default() }`; `AmbientLight` is a camera override component. Brightness defaults to 80 and is photometric, so blindly retaining the old value of 1 may produce a dark scene. Judge the native render visually. [Ambient-light definitions](https://github.com/bevyengine/bevy/blob/v0.20.1/crates/bevy_light/src/ambient_light.rs).

## FPS display

Use `Text::new("FPS: ")` on the parent and `TextSpan` on a child marked `FpsText`. `TextFont.font` requires `.into()` on a loaded font handle, and `font_size` is `FontSize::Px(60.0)`, not an `f32`. Absolute placement uses `Node { position_type: PositionType::Absolute, top: px(5), left: px(5), ..default() }`. Query mutable `TextSpan` to change the numeric child:

```rust
fn fps_update_system(
    diagnostics: Res<DiagnosticsStore>,
    mut spans: Query<&mut TextSpan, With<FpsText>>,
) {
    if let Some(value) = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|fps| fps.smoothed())
    {
        for mut span in &mut spans {
            **span = format!("{value:.2}");
        }
    }
}
```

Import `bevy::diagnostic::DiagnosticsStore` and `bevy::text::FontSize`. [Exact tagged text/FPS example](https://github.com/bevyengine/bevy/blob/v0.20.1/examples/ui/text/text.rs).

## egui

Add `EguiPlugin::default()` and run the UI in `EguiPrimaryContextPass`. Replace the old resource with `EguiContexts`:

```rust
fn ui(mut contexts: EguiContexts, mut state: ResMut<GlobalState>) -> Result {
    egui::Window::new("Menu").show(contexts.ctx_mut()?, |ui| {
        ui.checkbox(&mut state.use_octree, "Use octree");
    });
    Ok(())
}
```

The official example creates a camera before querying the primary context. Keep mouse/keyboard camera and target controls from responding while egui consumes those inputs; verify this interactively after compilation. [Exact bevy_egui 0.43.1 example](https://github.com/vladbat00/bevy_egui/blob/v0.43.1/examples/simple.rs).
