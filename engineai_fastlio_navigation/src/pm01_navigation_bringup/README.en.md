# pm01_navigation_bringup

[简体中文](README.md) | English

System composition and configuration resources. Selects the platform, mapping backend, Nav2, and motion worker according to profile and mode. The functional packages provide the algorithms and process management.

`navigation.launch.py` provides the simulation/hardware and mapping/fresh/saved entry point; `pm01_session` calls runtime.

Default parameters are maintained in the root [config/](../../config) directory; worlds are in this package's `worlds/` directory. Parameters are installed to `share/pm01_navigation_bringup/config`. See [Configuration overrides](../../docs/development.en.md#configuration-overrides) for the loading order.
