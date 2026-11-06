//! Browser lifecycle and the native-only, reproducible environment export entry point.
use bevy::prelude::*;

/// Build tooling invokes the native baker without creating a window or graphics device.
pub(crate) fn export_assets_if_requested() -> bool {
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(directory) = std::env::var_os("AC3D_EXPORT_WEB_ASSETS") {
        crate::environment::export_web_assets(std::path::Path::new(&directory))
            .unwrap_or_else(|error| panic!("Web environment export failed: {error}"));
        return true;
    }
    false
}

pub(crate) struct BrowserPlugin;

/// Register before DefaultPlugins so optional shader-module and metadata probes stay local.
pub(crate) struct BrowserAssetsPlugin;
impl Plugin for BrowserAssetsPlugin {
    fn build(&self, _app: &mut App) {
        #[cfg(target_arch = "wasm32")]
        {
            use bevy::asset::{
                io::{AssetSourceBuilder, AssetSourceId},
                AssetApp,
            };
            _app.register_asset_source(
                AssetSourceId::Default,
                AssetSourceBuilder::new(|| Box::new(web::CatalogReader::new())),
            );
        }
    }
}

impl Plugin for BrowserPlugin {
    fn build(&self, _app: &mut App) {
        #[cfg(target_arch = "wasm32")]
        {
            let world = _app.world_mut();
            for mut window in world.query::<&mut Window>().iter_mut(world) {
                window.canvas = Some("#flock".into());
                window.title = "Flock — A small world in motion".into();
                window.fit_canvas_to_parent = true;
                window.prevent_default_event_handling = true;
            }
            _app.init_resource::<web::Lifecycle>()
                .insert_resource(bevy::render::error_handler::RenderErrorHandler(
                    web::render_error,
                ))
                .add_systems(First, web::commands)
                .add_systems(Update, (web::asset_failures, web::lifecycle).chain());
        }
    }
}

#[cfg(target_arch = "wasm32")]
pub(crate) use web::fetch_bytes;

#[cfg(target_arch = "wasm32")]
mod web {
    use super::*;
    use crate::visitor::{PopulationTier, Visitor, VisitorScene};
    use std::cell::RefCell;
    use wasm_bindgen::{prelude::wasm_bindgen, JsCast};
    use wasm_bindgen_futures::JsFuture;

    #[wasm_bindgen]
    extern "C" {
        #[wasm_bindgen(js_namespace = window, js_name = flockHasAsset)]
        fn has_asset(path: &str) -> bool;
        #[wasm_bindgen(js_namespace = window, js_name = flockHasDirectory)]
        fn has_directory(path: &str) -> bool;
        #[wasm_bindgen(js_namespace = window, js_name = flockStatus)]
        fn status(state: &str, message: &str);
        #[wasm_bindgen(js_namespace = window, js_name = flockReducedMotion)]
        fn reduced_motion() -> bool;
        #[wasm_bindgen(js_namespace = window, js_name = flockFrame)]
        fn frame(
            milliseconds: f64,
            count: usize,
            paused: bool,
            animated: bool,
            hunters: usize,
            ui_visible: bool,
            lab: bool,
            reduced_motion: bool,
            help: bool,
            name: &str,
            simulation_seconds: f64,
            camera: &str,
            hunter_camera: bool,
            first_person: bool,
            camera_animal_id: u32,
        );
    }

    pub(super) struct CatalogReader(bevy::asset::io::wasm::HttpWasmAssetReader);
    impl CatalogReader {
        pub(super) fn new() -> Self {
            Self(bevy::asset::io::wasm::HttpWasmAssetReader::new("assets"))
        }
    }
    impl bevy::asset::io::AssetReader for CatalogReader {
        async fn read<'a>(
            &'a self,
            path: &'a std::path::Path,
        ) -> Result<impl bevy::asset::io::Reader + 'a, bevy::asset::io::AssetReaderError> {
            if !has_asset(&path.to_string_lossy()) {
                return Err(bevy::asset::io::AssetReaderError::NotFound(path.into()));
            }
            bevy::asset::io::AssetReader::read(&self.0, path).await
        }
        async fn read_meta<'a>(
            &'a self,
            path: &'a std::path::Path,
        ) -> Result<impl bevy::asset::io::Reader + 'a, bevy::asset::io::AssetReaderError> {
            Err::<bevy::asset::io::VecReader, _>(bevy::asset::io::AssetReaderError::NotFound(
                path.into(),
            ))
        }
        async fn read_directory<'a>(
            &'a self,
            _path: &'a std::path::Path,
        ) -> Result<Box<bevy::asset::io::PathStream>, bevy::asset::io::AssetReaderError> {
            Ok(Box::new(bevy::tasks::futures_lite::stream::empty()))
        }
        async fn is_directory<'a>(
            &'a self,
            path: &'a std::path::Path,
        ) -> Result<bool, bevy::asset::io::AssetReaderError> {
            Ok(has_directory(&path.to_string_lossy()))
        }
    }

    pub(crate) async fn fetch_bytes(path: &str) -> Result<Vec<u8>, &'static str> {
        let window = web_sys::window().ok_or("Browser window is unavailable")?;
        let response = JsFuture::from(window.fetch_with_str(path))
            .await
            .map_err(|_| "Environment download failed; check the connection and reload")?
            .dyn_into::<web_sys::Response>()
            .map_err(|_| "Environment server returned an invalid response")?;
        if !response.ok() {
            return Err("Environment asset is missing; reload to retry");
        }
        let buffer = response
            .array_buffer()
            .map_err(|_| "Environment response could not be read")?;
        let buffer = JsFuture::from(buffer)
            .await
            .map_err(|_| "Environment download was interrupted; reload to retry")?;
        Ok(js_sys::Uint8Array::new(&buffer).to_vec())
    }

    #[derive(Resource, Default)]
    pub(super) struct Lifecycle {
        initialized: bool,
        ready: bool,
        hidden_pause: Option<bool>,
        failed: bool,
    }

    pub(super) fn render_error(
        _error: &bevy::render::error_handler::RenderError,
        world: &mut World,
        _render_world: &mut World,
    ) -> bevy::render::error_handler::RenderErrorPolicy {
        world.resource_mut::<Lifecycle>().failed = true;
        world
            .resource_mut::<crate::presentation::Presentation>()
            .paused = true;
        status(
            "error",
            "The graphics device stopped the scene. Reload to restore it.",
        );
        bevy::render::error_handler::RenderErrorPolicy::StopRendering
    }

    thread_local! {
        static SCENE_REQUEST: RefCell<Option<(VisitorScene, PopulationTier)>> = const { RefCell::new(None) };
    }

    /// Validated browser deep-link/review control; it uses the same visitor contract as the HUD.
    #[wasm_bindgen]
    pub fn flock_scene(scene: &str, population: &str) -> Result<(), wasm_bindgen::JsValue> {
        let scene =
            VisitorScene::parse(scene).map_err(|error| wasm_bindgen::JsValue::from_str(&error))?;
        let population = PopulationTier::parse(population)
            .map_err(|error| wasm_bindgen::JsValue::from_str(&error))?;
        // Rapid requests select the latest scene rather than installing intermediate async jobs.
        SCENE_REQUEST.with(|request| *request.borrow_mut() = Some((scene, population)));
        Ok(())
    }

    pub(super) fn commands(world: &mut World) {
        if !world.resource::<Lifecycle>().initialized {
            world.resource_mut::<Lifecycle>().initialized = true;
            let visitor = world.resource::<Visitor>();
            let (mut scene, mut tier) = (visitor.scene, visitor.tier);
            let mut apply = reduced_motion();
            world.resource_mut::<Visitor>().reduced_motion = apply;
            if let Some(query) = web_sys::window()
                .and_then(|window| window.location().search().ok())
                .and_then(|query| web_sys::UrlSearchParams::new_with_str(&query).ok())
            {
                if let Some(selected) = query
                    .get("scene")
                    .and_then(|name| VisitorScene::parse(&name).ok())
                {
                    scene = selected;
                    apply = true;
                }
                if let Some(selected) = query
                    .get("population")
                    .and_then(|name| PopulationTier::parse(&name).ok())
                {
                    tier = selected;
                    apply = true;
                }
            }
            if apply {
                crate::visitor::apply(world, scene, tier);
            }
        }
        if let Some((scene, tier)) = SCENE_REQUEST.with(|request| request.borrow_mut().take()) {
            crate::visitor::apply(world, scene, tier);
        }
    }

    pub(super) fn asset_failures(
        mut failures: MessageReader<bevy::asset::UntypedAssetLoadFailedEvent>,
        mut lifecycle: ResMut<Lifecycle>,
        mut presentation: ResMut<crate::presentation::Presentation>,
    ) {
        for failure in failures.read() {
            error!("Browser asset failed: {}: {}", failure.path, failure.error);
            lifecycle.failed = true;
            presentation.paused = true;
            status(
                "error",
                "A scene asset could not load. Check your connection, then reload.",
            );
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(super) fn lifecycle(
        mut lifecycle: ResMut<Lifecycle>,
        mut presentation: ResMut<crate::presentation::Presentation>,
        appearance: Res<crate::scene::Appearance>,
        environments: Res<crate::environment::EnvironmentCache>,
        models: Res<crate::vehicle_model::VehicleModels>,
        state: Res<crate::GlobalState>,
        visitor: Res<Visitor>,
        clock: Res<crate::presentation::SimulationClock>,
        time: Res<Time<Real>>,
    ) {
        if lifecycle.failed {
            return;
        }
        let hidden = web_sys::window()
            .and_then(|w| w.document())
            .is_some_and(|d| d.hidden());
        if hidden && lifecycle.hidden_pause.is_none() {
            lifecycle.hidden_pause = Some(presentation.paused);
            presentation.paused = true;
        } else if !hidden {
            if let Some(paused) = lifecycle.hidden_pause.take() {
                presentation.paused = paused;
            }
        }
        if environments.failed(appearance.environment) {
            status("error", "This scene could not load. Reload to try again.");
            return;
        }
        let ready = !models.pending()
            && crate::environment::capture_ready(&appearance, Some(&environments));
        if ready && !lifecycle.ready {
            status("ready", "A small world in motion");
        }
        lifecycle.ready = ready;
        if ready && !time.delta().is_zero() {
            frame(
                if hidden {
                    0.0
                } else {
                    time.delta_secs_f64() * 1000.0
                },
                state.vehicle_count,
                presentation.paused,
                models.animation.is_some(),
                state.groups.hunter_count(),
                presentation.ui_visible,
                visitor.lab,
                visitor.reduced_motion,
                visitor.help,
                visitor.scene.name(),
                clock.0.elapsed_secs_f64(),
                if presentation.follow.enabled() || presentation.director.enabled {
                    "ride"
                } else if presentation.orbit {
                    "scenic"
                } else {
                    "free"
                },
                presentation.follow.subject() == crate::follow::FollowSubject::Hunter,
                presentation.follow.mode == crate::follow::FollowMode::FirstPerson,
                if presentation.follow.enabled() {
                    presentation.follow.id.unwrap_or(u32::MAX)
                } else {
                    u32::MAX
                },
            );
        }
    }
}
