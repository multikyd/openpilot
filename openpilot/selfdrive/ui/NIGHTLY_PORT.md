# KYD UI and Nightly

Nightly baseline `0a1291617fb9ae894c7078b30f601970e985a641`.
31 tracked UI files match Nightly, ignoring Windows line endings.
The inventory below records the checkout before the graphics binding migration.

## C4 onroad updates

Use the same Nightly revision for every upstream file in `mici/onroad`.
Compare the two working folders before updating. Update the unchanged files from the new Nightly revision,
then review each `KYD UI:` marker in the files with local edits. Preserve the KYD-only files and
run the C4 HUD and lead/brake tests after reapplying the changes. The markers point to feature blocks.
A full comparison with Nightly shows the removed and replaced lines too.
The road view follows the stock rendering sequence, with marked KYD insertions for the viewport, lead outline, and right sidebar.

### Nightly files identical to the local reference (7)

- `alert_renderer.py`
- `cabin_camera_dialog.py`
- `cameraview.py`
- `confidence_ball.py`
- `driver_state.py`
- `navigation.py`
- `status.py`

### Nightly files with KYD edits (6)

- `__init__.py`
- `augmented_road_view.py`
- `hud_renderer.py`
- `long_indicator.py`
- `model_renderer.py`
- `torque_bar.py`

### KYD-only C4 files (7)

- `bookmark_transfer.py` — Live bookmark transfer banner.
- `brake_label_controls.py` — Retained brake/vehicle feedback widget. The current C4 view does not mount it.
- `hud_layout.py` — C4 HUD geometry and advisory selection.
- `lead_brake_panel.py` — Right sidebar current speed, camera offset, and lead/brake display.
- `lead_brake_state.py` — Lead/brake display values and freshness rules.
- `signal_presentation.py` — Traffic signal lamp and countdown presentation.
- `signal_renderer.py` — Traffic signal and pass-advice overlays.

The shared `ui_state.py` provides KYD telemetry and the local brightness preference.
The detector used by the C4 overlay lives in `selfdrive/ui/onroad/brake_light_overlay.py`.

## C4 paths to check during an update

- HUD indicators in `mici/onroad/hud_renderer.py`.
- Path, lead and lane overlays in `mici/onroad/model_renderer.py`.
- Camera rendering in `mici/onroad/cameraview.py`.
- Onroad composition in `mici/onroad/augmented_road_view.py`.
- Check the local data, audio and asset wiring in `ui_state.py`, `soundd.py` and `SConscript`.
- Search for `KYD UI:` for local exceptions annotated in the files updated here.
- The shared widgets and C4 renderer use raylib_nb. KYD keeps its HUD geometry, overlays and camera color settings. The camera and polygon rasterizer use the Nightly CPU APIs.

## Nightly changes brought over

- The C4 onroad view uses the Nightly camera selection and shared alert resolver. KYD keeps its right-hand speed and lead strip, HUD overlays, and bookmark gestures.
- The navigation panel and cabin camera dialog use the Nightly implementation.
- The shared UI state uses the Nightly startup, notifications and backlight logic. KYD retains lateral and Kisa navigation telemetry, vehicle HUD state, and the display brightness preference.
- `ui.py` adopts the Nightly scheduling setup, with local device-specific imports and a fallback for missing scheduling privileges.
- `mici/layouts/settings/network/network_layout.py` adopts the hotspot password length validation.
- `mici/layouts/settings/network/esim_ui.py` and `mici/layouts/settings/network/wifi_ui.py` adopt the Nightly vector arguments and graphics binding.

## Different from Nightly (74)

- `SConscript`
- `body/layouts/onroad.py`
- `layouts/home.py`
- `layouts/main.py`
- `layouts/onboarding.py`
- `layouts/settings/firehose.py`
- `layouts/settings/initial_cruise_speed.py`
- `layouts/settings/settings.py`
- `layouts/sidebar.py`
- `mici/layouts/home.py`
- `mici/layouts/main.py`
- `mici/layouts/offroad_alerts.py`
- `mici/layouts/onboarding.py`
- `mici/layouts/settings/developer.py`
- `mici/layouts/settings/device/device_layout.py`
- `mici/layouts/settings/device/prime.py`
- `mici/layouts/settings/firehose.py`
- `mici/layouts/settings/network/__init__.py`
- `mici/layouts/settings/network/esim_ui.py`
- `mici/layouts/settings/network/wifi_ui.py`
- `mici/layouts/settings/settings.py`
- `mici/layouts/settings/software.py`
- `mici/onroad/__init__.py`
- `mici/onroad/alert_renderer.py`
- `mici/onroad/augmented_road_view.py`
- `mici/onroad/cabin_camera_dialog.py`
- `mici/onroad/cameraview.py`
- `mici/onroad/confidence_ball.py`
- `mici/onroad/driver_state.py`
- `mici/onroad/hud_renderer.py`
- `mici/onroad/long_indicator.py`
- `mici/onroad/model_renderer.py`
- `mici/onroad/navigation.py`
- `mici/onroad/torque_bar.py`
- `mici/tests/test_widget_leaks.py`
- `mici/widgets/button.py`
- `mici/widgets/dialog.py`
- `mici/widgets/info.py`
- `mici/widgets/qr.py`
- `onroad/alert_renderer.py`
- `onroad/augmented_road_view.py`
- `onroad/cabin_camera_dialog.py`
- `onroad/cameraview.py`
- `onroad/driver_state.py`
- `onroad/hud_renderer.py`
- `onroad/model_renderer.py`
- `soundd.py`
- `tests/diff/replay.py`
- `tests/diff/replay_script.py`
- `tests/profile_onroad.py`
- `translations/app.pot`
- `translations/app_de.po`
- `translations/app_en.po`
- `translations/app_es.po`
- `translations/app_fr.po`
- `translations/app_ja.po`
- `translations/app_ko.po`
- `translations/app_pt-BR.po`
- `translations/app_th.po`
- `translations/app_tr.po`
- `translations/app_uk.po`
- `translations/app_zh-CHS.po`
- `translations/app_zh-CHT.po`
- `translations/auto_translate.sh`
- `translations/languages.json`
- `translations/potools.py`
- `ui.py`
- `ui_state.py`
- `watch3.py`
- `widgets/offroad_alerts.py`
- `widgets/pairing_dialog.py`
- `widgets/prime.py`
- `widgets/setup.py`
- `widgets/ssh_key.py`

## KYD only (38)

- `mici/layouts/settings/toggles.py`
- `mici/onroad/bookmark_transfer.py`
- `mici/onroad/brake_label_controls.py`
- `mici/onroad/hud_layout.py`
- `mici/onroad/lead_brake_panel.py`
- `mici/onroad/lead_brake_state.py`
- `mici/onroad/signal_presentation.py`
- `mici/onroad/signal_renderer.py`
- `onroad/brake_labels.py`
- `onroad/brake_light_overlay.py`
- `onroad/brake_lights.md`
- `onroad/brake_lights.py`
- `onroad/brake_vehicle_model.py`
- `onroad/exp_button.py`
- `onroad/models/ATTRIBUTION.md`
- `onroad/models/brake_light_v4.onnx`
- `onroad/models/brake_light_v4_160.onnx`
- `onroad/models/brake_light_v4_320.onnx`
- `onroad/models/brake_light_v4_320_int8.onnx`
- `onroad/models/runtime/README.md`
- `onroad/models/runtime/coloredlogs-15.0.1-py2.py3-none-any.whl`
- `onroad/models/runtime/flatbuffers-25.2.10-py2.py3-none-any.whl`
- `onroad/models/runtime/humanfriendly-10.0-py2.py3-none-any.whl`
- `onroad/models/runtime/manifest.json`
- `onroad/models/runtime/mpmath-1.3.0-py3-none-any.whl`
- `onroad/models/runtime/onnxruntime-1.22.1-cp312-cp312-manylinux_2_27_aarch64.manylinux_2_28_aarch64.whl`
- `onroad/models/runtime/packaging-25.0-py3-none-any.whl`
- `onroad/models/runtime/protobuf-5.29.5-cp38-abi3-manylinux2014_aarch64.whl`
- `onroad/models/runtime/sympy-1.13.3-py3-none-any.whl`
- `tests/test_brake_labels.py`
- `tests/test_brake_lights.py`
- `tests/test_brake_vehicle_model.py`
- `tests/test_c4_hud_layout.py`
- `tests/test_lead_brake_panel.py`
- `tests/test_nav_source_badge.py`
- `tests/test_overtake_advice.py`
- `tests/test_soundd_speech.py`
- `widgets/exp_mode_button.py`

## Nightly only (4)

- `mici/layouts/settings/driving.py`
- `mici/layouts/settings/initial_cruise_speed.py`
- `mici/onroad/status.py`
- `translations/app_ro.po`
