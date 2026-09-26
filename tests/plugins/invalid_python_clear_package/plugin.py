from slic3r_api import PluginBase, STEP_POST_SLICING


class InvalidClearPlugin(PluginBase):
    clear = 42

    def __init__(self):
        super().__init__("python.test.clear.invalid_registration", STEP_POST_SLICING)

    def run(self, run_ctx_address):
        return None


def register_plugin(api):
    return InvalidClearPlugin()
