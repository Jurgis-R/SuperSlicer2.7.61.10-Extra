#/|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
#/|/
#/|/ SuperSlicer is released under the terms of the AGPLv3 or higher
#/|/

"""
Minimal Python plugin example.

This plugin intentionally does not modify the print. It proves the full path:

1. SuperSlicer loads python_plugin_loader.dll from the plugin repository.
2. The loader imports this .py file.
3. This file returns a Python plugin object to the loader.
4. The loader registers a normal plugin_instance through the C ABI.
5. The host later calls setup()/run() through the loader-created vtable.

The example runs late in the pipeline and only reports progress once, so it is
safe to keep enabled while testing the loader.
"""

from slic3r_api import PluginBase, STEP_POST_SLICING, report_progress, throw_if_cancelled


class ExamplePythonPlugin(PluginBase):
    def __init__(self):
        super().__init__(
            "python.example.noop",
            STEP_POST_SLICING,
            name="Python no-op example",
            description="Minimal Python plugin example that does not change print data.",
            priority=100000,
        )

    def run(self, run_ctx_address):
        throw_if_cancelled(run_ctx_address)
        if run_ctx_address:
            report_progress(run_ctx_address, 1.0, "Python example plugin")


def register_plugin(api):
    return ExamplePythonPlugin()
