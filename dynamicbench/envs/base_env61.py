"""Native Isaac Lab 3 PhysX lifecycle, isolated from RoboDojo's Lab 2 wrapper."""

from types import SimpleNamespace

from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.sim.utils import get_current_stage
from isaaclab_physx.physics import PhysxCfg
from omni.physx import acquire_physx_interface


class BaseEnv61:
    def __init__(self, config, app, *, interactiveSceneCfg):
        self.config = config
        self.app = app
        self.interactiveSceneCfg = interactiveSceneCfg
        self.sim = None

    def launch_sim(self, config):
        self.context = SimulationContext(
            SimulationCfg(
                dt=float(config.dt),
                device="cpu",
                use_fabric=False,
                render_interval=int(config.render_interval),
                physics=PhysxCfg(enable_ccd=True, enable_stabilization=True),
            )
        )
        self.stage = get_current_stage()
        scene = InteractiveScene(self.interactiveSceneCfg)
        self.sim = SimpleNamespace(scene=scene, sim=self.context, _sim_step_counter=0)
        self.sim.unwrapped = self.sim
        self.sim.reset = scene.reset
        self._setup_scene(self.sim)
        self._post_setup_scene(self.sim)
        self._physics_subscription = self.physics_interface.subscribe_physics_step_events(self._physics_tick)
        self.context.reset()
        scene.update(float(config.dt))

    def _setup_scene(self, sim):
        self.setup_physics(sim)

    def setup_physics(self, sim):
        self.physics_interface = acquire_physx_interface()
        self.physics_interface.overwrite_gpu_setting(0)

    def _physics_tick(self, dt):
        self.sim._sim_step_counter += 1

    def sim_step(self, render=False):
        self.sim.scene.write_data_to_sim()
        before = self.sim._sim_step_counter
        self.context.step(render=False)
        if self.sim._sim_step_counter != before + 1:
            raise RuntimeError("PhysX_step_count_mismatch")
        self.sim.scene.update(float(self.config.dt))
        if render:
            self.render()

    def render(self):
        from isaaclab_physx.renderers.isaac_rtx_renderer_utils import ensure_isaac_rtx_render_update

        self.context.render()
        # Lab 3 excludes headless offscreen cameras from continuous rendering.
        # Explicitly pump this frame, with physics disabled by the backend helper.
        ensure_isaac_rtx_render_update(force=True)

    def close(self):
        if self.sim is not None:
            self._physics_subscription = None
            self.context.stop()
            SimulationContext.clear_instance()
            self.sim = None
