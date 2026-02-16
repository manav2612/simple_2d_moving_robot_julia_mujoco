#!/usr/bin/env python3
"""
Generate AIF_MuJoCoRobot_Code.pdf — Code Architecture & API Reference
Covers every module, export, signature, and configuration in the project.
"""

import os
from fpdf import FPDF

FONT_DIR = "/usr/share/fonts/truetype/dejavu"
OUT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_FILE = os.path.join(OUT_DIR, "AIF_MuJoCoRobot_Code.pdf")

ARROW = "\u2192"


class CodePDF(FPDF):
    """Custom PDF with Unicode DejaVu fonts and helper methods."""

    def __init__(self):
        super().__init__()
        self.add_font("DejaVu", "", f"{FONT_DIR}/DejaVuSans.ttf")
        self.add_font("DejaVu", "B", f"{FONT_DIR}/DejaVuSans-Bold.ttf")
        self.add_font("DejaVu", "I", f"{FONT_DIR}/DejaVuSans-Oblique.ttf")
        self.add_font("DejaVu", "BI", f"{FONT_DIR}/DejaVuSans-BoldOblique.ttf")
        self.add_font("Mono", "", f"{FONT_DIR}/DejaVuSansMono.ttf")
        self.add_font("Mono", "B", f"{FONT_DIR}/DejaVuSansMono-Bold.ttf")
        self.set_auto_page_break(auto=True, margin=20)

    def header(self):
        if self.page_no() == 1:
            return
        self.set_font("DejaVu", "I", 8)
        self.set_text_color(120, 120, 120)
        self.cell(0, 8, "AIF MuJoCo Robot \u2014 Code Architecture & API Reference",
                  align="C", new_x="LMARGIN", new_y="NEXT")
        self.set_draw_color(200, 200, 200)
        self.line(10, self.get_y(), 200, self.get_y())
        self.ln(4)
        self.set_text_color(0, 0, 0)

    def footer(self):
        self.set_y(-15)
        self.set_font("DejaVu", "I", 8)
        self.set_text_color(120, 120, 120)
        self.cell(0, 10, f"Page {self.page_no()}/{{nb}}", align="C")
        self.set_text_color(0, 0, 0)

    # ── building blocks ──

    def title_page(self, title, subtitle):
        self.add_page()
        self.ln(60)
        self.set_font("DejaVu", "B", 28)
        self.set_text_color(30, 100, 60)
        self.multi_cell(0, 14, title, align="C")
        self.ln(6)
        self.set_font("DejaVu", "I", 14)
        self.set_text_color(80, 80, 80)
        self.multi_cell(0, 10, subtitle, align="C")
        self.ln(12)
        self.set_draw_color(30, 100, 60)
        self.set_line_width(0.8)
        self.line(50, self.get_y(), 160, self.get_y())
        self.ln(8)
        self.set_font("DejaVu", "", 10)
        self.set_text_color(100, 100, 100)
        self.multi_cell(0, 6,
            "Complete code architecture, module APIs, CLI options,\n"
            "configuration, and MuJoCo model descriptions.",
            align="C")
        self.set_text_color(0, 0, 0)
        self.set_line_width(0.2)

    def section(self, num, title):
        self.add_page()
        self.set_font("DejaVu", "B", 18)
        self.set_text_color(30, 100, 60)
        self.cell(0, 12, f"{num}.  {title}", new_x="LMARGIN", new_y="NEXT")
        self.set_draw_color(30, 100, 60)
        self.set_line_width(0.6)
        self.line(10, self.get_y(), 200, self.get_y())
        self.ln(6)
        self.set_text_color(0, 0, 0)
        self.set_line_width(0.2)

    def subsection(self, title):
        self.ln(3)
        self.set_font("DejaVu", "B", 12)
        self.set_text_color(40, 90, 55)
        self.cell(0, 9, title, new_x="LMARGIN", new_y="NEXT")
        self.ln(2)
        self.set_text_color(0, 0, 0)

    def body(self, text):
        self.set_font("DejaVu", "", 10)
        self.multi_cell(0, 6, text)
        self.ln(2)

    def body_italic(self, text):
        self.set_font("DejaVu", "I", 9)
        self.set_text_color(100, 100, 100)
        self.multi_cell(0, 5, text)
        self.ln(1)
        self.set_text_color(0, 0, 0)

    def code_block(self, text):
        self.ln(2)
        self.set_font("Mono", "", 8.5)
        self.set_fill_color(245, 248, 245)
        self.set_draw_color(180, 210, 180)
        self.set_x(12)
        self.multi_cell(186, 5, text, border=1, fill=True, align="L")
        self.ln(2)
        self.set_font("DejaVu", "", 10)

    def bullet(self, text):
        self.set_font("DejaVu", "", 10)
        x0 = self.get_x()
        self.set_x(15)
        self.cell(6, 6, " \u2022 ", new_x="END")
        self.multi_cell(0, 6, text)
        self.set_x(x0)

    def table_header(self, cols, widths):
        self.set_font("DejaVu", "B", 9)
        self.set_fill_color(230, 240, 230)
        self.set_draw_color(180, 200, 180)
        for col, w in zip(cols, widths):
            self.cell(w, 7, f" {col}", border=1, fill=True)
        self.ln()

    def table_row(self, cols, widths):
        self.set_font("Mono", "", 8.5)
        max_h = 6
        for col, w in zip(cols, widths):
            self.cell(w, max_h, f" {col}", border=1)
        self.ln()

    def table_row_wrap(self, col1, col2, w1, w2):
        self.set_font("Mono", "", 8.5)
        x = self.get_x()
        y = self.get_y()
        self.multi_cell(w1, 5.5, f" {col1}", border="LTB")
        h = self.get_y() - y
        self.set_xy(x + w1, y)
        self.set_font("DejaVu", "", 8.5)
        self.multi_cell(w2, h, f" {col2}", border="RTB")

    def exports_table(self, rows):
        """rows = [(symbol, description), ...]"""
        self.ln(1)
        self.table_header(["Symbol", "Description"], [70, 120])
        for sym, desc in rows:
            self.table_row([sym, desc], [70, 120])
        self.ln(2)

    def note_box(self, text):
        self.ln(2)
        self.set_fill_color(235, 250, 240)
        self.set_draw_color(100, 180, 120)
        self.set_font("DejaVu", "I", 9)
        self.set_x(12)
        self.multi_cell(186, 6, text, border=1, fill=True)
        self.ln(2)
        self.set_font("DejaVu", "", 10)


def build_pdf():
    pdf = CodePDF()
    pdf.alias_nb_pages()

    # ── Title Page ──
    pdf.title_page(
        "AIF MuJoCo Robot",
        "Code Architecture & API Reference"
    )

    # ══════════════════════════════════════════════════════════
    # 1. Project Overview
    # ══════════════════════════════════════════════════════════
    pdf.section(1, "Project Overview")
    pdf.body(
        "A simple Active Inference (AIF) controller for a 3D MuJoCo robot. "
        "The robot navigates toward a goal by minimizing Expected Free Energy (EFE)."
    )
    pdf.subsection("Project Structure")
    struct = [
        ("src/AIFMuJoCoRobot.jl", "Main module"),
        ("src/aif/beliefs.jl", "Belief state (Gaussian)"),
        ("src/aif/generative_model.jl", "Transition + observation models"),
        ("src/aif/efe.jl", "Expected Free Energy"),
        ("src/aif/policy.jl", "Policy selection (argmin EFE)"),
        ("src/aif/action.jl", "Action-to-control mapping"),
        ("src/aif/rxinfer_filter.jl", "RxInfer streaming filter"),
        ("src/control/aif_controller.jl", "AIF controller (integrator)"),
        ("src/sim/mujoco_env.jl", "MuJoCo environment wrapper"),
        ("src/sim/sensors.jl", "Sensor / observation model"),
        ("src/utils/math.jl", "Math utilities"),
        ("src/utils/logging.jl", "Logging utilities"),
        ("scripts/run_cli.jl", "CLI entry point"),
        ("experiments/configs.jl", "Configuration presets"),
        ("models/robot.xml", "MuJoCo 3D robot model"),
    ]
    pdf.table_header(["Path", "Description"], [90, 100])
    for path, desc in struct:
        pdf.table_row([path, desc], [90, 100])

    pdf.subsection("Dependencies (Project.toml)")
    deps = [
        ("BangBang", "Efficient in-place operations"),
        ("Distributions", "Probability distributions"),
        ("LinearAlgebra", "Matrix/vector operations (stdlib)"),
        ("MuJoCo", "MuJoCo.jl physics simulation"),
        ("Plots", "Plotting trajectories"),
        ("Printf", "Formatted printing (stdlib)"),
        ("Random", "RNG (stdlib)"),
        ("Rocket", "Reactive streams for RxInfer"),
        ("RxInfer", "Bayesian inference on factor graphs"),
    ]
    pdf.table_header(["Package", "Purpose"], [70, 120])
    for pkg, purpose in deps:
        pdf.table_row([pkg, purpose], [70, 120])

    pdf.subsection("Quick Start")
    pdf.code_block(
        "cd aif_mujoco_robot\n"
        "julia --project=. -e 'using Pkg; Pkg.instantiate()'\n"
        "julia --project=. scripts/run_cli.jl \\\n"
        "  --goal 0.8 0.8 0.4 --init -0.5 -0.5 0.2 \\\n"
        "  --steps 500 --ctrl_scale 5.0 --save_plot trajectory.png"
    )

    # ══════════════════════════════════════════════════════════
    # 2. Main Module
    # ══════════════════════════════════════════════════════════
    pdf.section(2, "Main Module (AIFMuJoCoRobot.jl)")
    pdf.body("Entry point that includes all submodules and exports the public API.")

    pdf.subsection("Include Order")
    pdf.code_block(
        "utils/math.jl  ->  utils/logging.jl\n"
        "aif/beliefs.jl -> aif/generative_model.jl -> aif/efe.jl\n"
        "aif/action.jl  -> aif/policy.jl -> aif/rxinfer_filter.jl\n"
        "sim/sensors.jl -> sim/mujoco_env.jl\n"
        "control/aif_controller.jl\n"
        "experiments/configs.jl"
    )

    pdf.subsection("Exports")
    pdf.exports_table([
        ("run_simulation", "Run the full AIF + MuJoCo loop"),
        ("default_model_path", "Path to models/robot.xml"),
        ("BeliefState", "Gaussian belief struct"),
        ("init_belief", "Create initial belief"),
        ("update_belief!", "Bayesian update with observation"),
        ("EnvState", "MuJoCo environment struct"),
        ("load_env", "Load MuJoCo model"),
        ("reset!", "Reset environment"),
        ("step!", "Step MuJoCo simulation"),
        ("get_position", "Read current [x,y,z]"),
        ("get_goal", "Read goal [x,y,z]"),
    ])

    pdf.subsection("run_simulation Signature")
    pdf.code_block(
        "run_simulation(;\n"
        "    steps::Int = 100,\n"
        "    goal::Vector{Float64} = [0.8, 0.8, 0.4],\n"
        "    init_pos::Vector{Float64} = [-0.5, -0.5, 0.2],\n"
        "    obs_noise::Real = 0.01,\n"
        "    \u03b3::Real = 1.0,  \u03b2::Real = 0.1,\n"
        "    ctrl_scale::Real = 1.0,\n"
        "    nsteps_per_ctrl::Int = 5,\n"
        "    model_path::String = default_model_path(),\n"
        "    seed::Union{Int,Nothing} = 42,\n"
        "    process_noise = 0.005,\n"
        "    verbose::Bool = true,\n"
        "    inference_backend::Symbol = :analytic,  # or :rxinfer\n"
        ")"
    )

    pdf.subsection("Control Loop (Architecture)")
    pdf.code_block(
        "  +------------------+\n"
        "  | compute_control  |  belief + goal -> (ctrl, action, efe)\n"
        "  +--------+---------+\n"
        "           |\n"
        "  +--------v---------+\n"
        "  | predict_belief!  |  OR  rxinfer_step! (if :rxinfer)\n"
        "  +--------+---------+\n"
        "           |\n"
        "  +--------v---------+\n"
        "  | MuJoCo step!     |  Apply ctrl, advance physics\n"
        "  +--------+---------+\n"
        "           |\n"
        "  +--------v---------+\n"
        "  | read_observation  |  qpos + noise -> obs\n"
        "  +--------+---------+\n"
        "           |\n"
        "  +--------v---------+\n"
        "  | update_belief!   |  OR  rxinfer_step! (if :rxinfer)\n"
        "  +--------+---------+\n"
        "           |\n"
        "        [repeat]"
    )

    # ══════════════════════════════════════════════════════════
    # 3. Belief State
    # ══════════════════════════════════════════════════════════
    pdf.section(3, "Belief State (beliefs.jl)")
    pdf.body("Represents the agent's belief over 3D position as a diagonal Gaussian.")

    pdf.subsection("BeliefState Struct")
    pdf.code_block(
        "struct BeliefState\n"
        "    mean::Vector{Float64}   # [\u03bc_x, \u03bc_y, \u03bc_z]\n"
        "    cov::Vector{Float64}    # [\u03c3\u00b2_x, \u03c3\u00b2_y, \u03c3\u00b2_z]\n"
        "end"
    )

    pdf.subsection("Exports")
    pdf.exports_table([
        ("BeliefState", "Struct with mean and cov"),
        ("init_belief", "Create initial belief from mean/cov"),
        ("update_belief!", "Bayesian update with observation"),
        ("predict_belief!", "Predict forward given control"),
        ("entropy", "Entropy of diagonal Gaussian"),
    ])

    pdf.subsection("Key Constants")
    pdf.bullet("COV_MIN = 1e-5 (lower bound on variance)")
    pdf.bullet("COV_MAX = 2.0 (upper bound on variance)")
    pdf.note_box(
        "When inference_backend = :rxinfer, predict_belief! and update_belief! "
        "are bypassed. The RxInfer posterior is written directly into "
        "belief.mean and belief.cov."
    )

    # ══════════════════════════════════════════════════════════
    # 4. Generative Model
    # ══════════════════════════════════════════════════════════
    pdf.section(4, "Generative Model (generative_model.jl)")
    pdf.body("Transition and observation likelihood for Active Inference.")
    pdf.exports_table([
        ("predict_transition", "s' = s + u (3D additive)"),
        ("observation_likelihood", "P(o|s) diagonal Gaussian"),
        ("predict_observation", "Expected obs from state (noiseless)"),
    ])
    pdf.body_italic(
        "These functions are used by EFE/policy selection regardless of "
        "which inference backend is active."
    )

    # ══════════════════════════════════════════════════════════
    # 5. EFE
    # ══════════════════════════════════════════════════════════
    pdf.section(5, "Expected Free Energy (efe.jl)")
    pdf.body("Computes EFE = pragmatic - epistemic for policy selection.")
    pdf.exports_table([
        ("compute_efe", "Full EFE for an action"),
        ("pragmatic_term", "Goal-distance cost (with ctrl_scale)"),
        ("epistemic_term", "Uncertainty / information term"),
    ])
    pdf.subsection("compute_efe Signature")
    pdf.code_block(
        "compute_efe(belief_mean, belief_cov, action, goal;\n"
        "    \u03b3=1.0, \u03b2=0.1, axis_weights=nothing,\n"
        "    ctrl_scale=1.0, ctrl_lim=1.2)"
    )

    # ══════════════════════════════════════════════════════════
    # 6. Policy
    # ══════════════════════════════════════════════════════════
    pdf.section(6, "Policy Selection (policy.jl)")
    pdf.body("Selects the action minimizing EFE over a discrete 5\u00d75\u00d75 action grid.")
    pdf.exports_table([
        ("select_action", "Choose best action for belief+goal"),
        ("get_action_set", "125-action grid (step_size=0.08)"),
    ])
    pdf.subsection("select_action Signature")
    pdf.code_block(
        "select_action(belief, goal;\n"
        "    actions=get_action_set(), \u03b3=1.0, \u03b2=0.1,\n"
        "    axis_weights=nothing, ctrl_scale=1.0, ctrl_lim=1.2)"
    )

    # ══════════════════════════════════════════════════════════
    # 7. Action
    # ══════════════════════════════════════════════════════════
    pdf.section(7, "Action (action.jl)")
    pdf.body("Converts abstract velocity actions into MuJoCo control signals.")
    pdf.exports_table([
        ("to_control", "action -> scaled+clamped control"),
        ("clamp_action", "Clamp action to limits"),
    ])
    pdf.subsection("to_control Signature")
    pdf.code_block("to_control(action; scale=1.0, ctrl_lim=1.2)")

    # ══════════════════════════════════════════════════════════
    # 8. RxInfer Filter
    # ══════════════════════════════════════════════════════════
    pdf.section(8, "RxInfer Filter (rxinfer_filter.jl)")
    pdf.body(
        "Alternative inference backend using RxInfer.jl's reactive message-passing. "
        "Three independent 1D engines (x, y, z) run a linear-Gaussian SSM."
    )
    pdf.exports_table([
        ("RxInferFilter3D", "Struct with 3 per-axis engines"),
        ("init_rxinfer_filter", "Create and start 3-axis filter"),
        ("rxinfer_step!", "Push (ctrl, obs), return posterior"),
        ("rxinfer_stop!", "Stop engines, unsubscribe"),
    ])

    pdf.subsection("Streaming Design")
    pdf.body(
        "Each axis uses a single RecentSubject{NamedTuple{(:y,:u)}} stream. "
        "Pushing a (y, u) tuple triggers exactly one inference cycle per step, "
        "avoiding the combineLatest double-fire issue."
    )

    pdf.subsection("init_rxinfer_filter Signature")
    pdf.code_block(
        "init_rxinfer_filter(\n"
        "    init_mean::AbstractVector,\n"
        "    init_cov::AbstractVector;\n"
        "    obs_noise = 0.01,\n"
        "    process_noise = 0.005,\n"
        ")"
    )

    pdf.subsection("rxinfer_step! Signature")
    pdf.code_block(
        "rxinfer_step!(filter, ctrl, obs)\n"
        f"  {ARROW} (post_mean::Vector, post_var::Vector)"
    )

    # ══════════════════════════════════════════════════════════
    # 9. Controller
    # ══════════════════════════════════════════════════════════
    pdf.section(9, "AIF Controller (aif_controller.jl)")
    pdf.body("Integrates belief, policy, and action into a single control step.")
    pdf.exports_table([
        ("compute_control", "belief+goal -> (ctrl, action, efe)"),
    ])
    pdf.subsection("compute_control Flow")
    pdf.code_block(
        "1. select_action(belief, goal; ...)\n"
        "2. to_control(action; scale, ctrl_lim)\n"
        "3. compute_efe(belief.mean, belief.cov, action, goal; ...)\n"
        f"4. return (ctrl=..., action=..., efe=...)"
    )
    pdf.note_box(
        "The controller is backend-agnostic: it always reads belief.mean "
        "and belief.cov regardless of whether they came from analytic or "
        "RxInfer updates."
    )

    # ══════════════════════════════════════════════════════════
    # 10. MuJoCo Env
    # ══════════════════════════════════════════════════════════
    pdf.section(10, "MuJoCo Environment (mujoco_env.jl)")
    pdf.body("Wraps MuJoCo for the 3D point-mass robot.")
    pdf.exports_table([
        ("EnvState", "Struct: model, data, goal"),
        ("load_env", "Load XML model"),
        ("reset!", "Reset to initial position"),
        ("step!", "Apply control, advance physics"),
        ("get_position", "Current [x,y,z] from qpos"),
        ("get_goal", "Goal [x,y,z]"),
    ])
    pdf.subsection("Model Layout")
    pdf.bullet("qpos[1], qpos[2], qpos[3]: x, y, z (slide joints)")
    pdf.bullet("ctrl[1], ctrl[2], ctrl[3]: displacement (target = pos + ctrl)")

    pdf.subsection("step! Signature")
    pdf.code_block("step!(env, ctrl; nsteps=5)")
    pdf.body("Applies ctrl and runs nsteps MuJoCo physics steps for stability.")

    # ══════════════════════════════════════════════════════════
    # 11. Sensors
    # ══════════════════════════════════════════════════════════
    pdf.section(11, "Sensors (sensors.jl)")
    pdf.body("Extracts observations from MuJoCo state with optional noise.")
    pdf.exports_table([
        ("read_position", "Extract [x,y,z] from qpos"),
        ("read_observation", "Position + Gaussian noise"),
    ])
    pdf.subsection("read_observation Signature")
    pdf.code_block("read_observation(qpos; obs_noise=0.01, rng=default_rng())")
    pdf.body(
        "obs_noise: scalar or [\u03c3\u00b2_x, \u03c3\u00b2_y, \u03c3\u00b2_z]. "
        "Adds sqrt(\u03c3\u00b2) * randn() per dimension when \u03c3\u00b2 > 0."
    )

    # ══════════════════════════════════════════════════════════
    # 12. Utilities
    # ══════════════════════════════════════════════════════════
    pdf.section(12, "Utilities")

    pdf.subsection("Math Utilities (math.jl)")
    pdf.exports_table([
        ("normalize!", "Normalize probability vector in place"),
        ("gaussian_pdf", "Univariate/multivariate diagonal PDF"),
        ("gaussian_entropy", "Entropy of univariate Gaussian"),
        ("softmax", "Numerically stable softmax"),
        ("clamp_vec", "Clamp vector elements to bounds"),
    ])

    pdf.subsection("Logging Utilities (logging.jl)")
    pdf.exports_table([
        ("log_step", "Print step: pos, goal, belief, efe"),
        ("log_summary", "Print summary: steps, dist, converged"),
    ])

    # ══════════════════════════════════════════════════════════
    # 13. CLI
    # ══════════════════════════════════════════════════════════
    pdf.section(13, "CLI (run_cli.jl)")
    pdf.body("Command-line interface for running simulations and visualisations.")
    pdf.subsection("Usage")
    pdf.code_block(
        "julia --project=. scripts/run_cli.jl \\\n"
        "  --goal <x> <y> <z> --init <x> <y> <z> [options]"
    )
    pdf.subsection("All Options")
    opts = [
        ("--goal", "Goal position (x y z)", "0.8 0.8 0.4"),
        ("--init", "Initial position (x y z)", "-0.5 -0.5 0.2"),
        ("--steps", "Max simulation steps", "500"),
        ("--ctrl_scale", "Control scaling factor", "4.0"),
        ("--obs_noise", "Observation variance", "0.01"),
        ("--process_noise", "Process noise variance", "0.005"),
        ("--backend", "analytic | rxinfer", "analytic"),
        ("--save_plot", "Path to save trajectory PNG", "(none)"),
        ("--render", "Launch MuJoCo visualiser", "false"),
        ("--renderarm", "Panda arm pick-and-place", "false"),
        ("--agent_color", "Agent color r g b [a]", "(default)"),
        ("--goal_color", "Goal color r g b [a]", "(default)"),
        ("--seed", "Random seed", "42"),
        ("--verbose", "Print step logs", "true"),
    ]
    pdf.table_header(["Option", "Description", "Default"], [45, 90, 55])
    for opt, desc, default in opts:
        pdf.table_row([opt, desc, default], [45, 90, 55])

    pdf.subsection("Render Arm Sequence")
    pdf.body(
        "With --renderarm, the AIF trajectory is replayed on a Panda arm:\n"
        "1. Arm moves to init position (pickup)\n"
        "2. Red ball follows arm along AIF path (carry)\n"
        "3. Red ball placed on green box at goal (drop)"
    )
    pdf.body(
        "Panda workspace bounds: X [0.2, 0.8], Y [-0.4, 0.4], "
        "Z [0.1, 0.5]. Trajectory positions are clamped."
    )
    pdf.subsection("Example (RxInfer + Panda Arm)")
    pdf.code_block(
        "julia --project=. scripts/run_cli.jl \\\n"
        "  --backend rxinfer \\\n"
        "  --goal 0.8 0.8 0.4 --init 0.3 0.0 0.2 \\\n"
        "  --steps 500 --ctrl_scale 5.0 \\\n"
        "  --save_plot trajectory.png --renderarm"
    )

    # ══════════════════════════════════════════════════════════
    # 14. Configuration
    # ══════════════════════════════════════════════════════════
    pdf.section(14, "Configuration (configs.jl)")
    pdf.body("Predefined configuration presets for experiments.")

    pdf.subsection("default_config()")
    cfgs = [
        ("steps", "500"),
        ("goal", "[0.8, 0.8, 0.4]"),
        ("init_pos", "[-0.5, -0.5, 0.2]"),
        ("obs_noise", "0.01"),
        ("process_noise", "0.005"),
        ("\u03b3", "1.2"),
        ("\u03b2", "0.05"),
        ("ctrl_scale", "4.0"),
        ("nsteps_per_ctrl", "5"),
        ("seed", "42"),
        ("verbose", "true"),
        ("inference_backend", ":analytic"),
    ]
    pdf.table_header(["Parameter", "Default Value"], [80, 110])
    for param, val in cfgs:
        pdf.table_row([param, val], [80, 110])

    pdf.subsection("Presets")
    pdf.bullet(f"config_goal_seeking(): \u03b3=2.0, \u03b2=0.05 (emphasize goal)")
    pdf.bullet(f"config_exploration(): \u03b3=0.5, \u03b2=0.3 (emphasize exploration)")

    # ══════════════════════════════════════════════════════════
    # 15. MuJoCo Models
    # ══════════════════════════════════════════════════════════
    pdf.section(15, "MuJoCo Models")

    pdf.subsection("models/robot.xml")
    pdf.body("Main 3D robot scene for AIF simulation:")
    pdf.bullet("World: ground plane, light, target site at (0.8, 0.8, 0.4)")
    pdf.bullet("Robot: body with three slide joints (x, y, z), sphere geom")
    pdf.bullet("Actuators: position actuators on slide_x/y/z, range \u00b110")

    pdf.subsection("panda_render_scene.xml")
    pdf.body("Scene for --renderarm: Panda arm with pick-and-place visuals.")
    pdf.bullet("Red sphere (mocap) at initial position")
    pdf.bullet("Green box (static) at goal position")
    pdf.bullet("Includes panda_mocap.xml for the arm model")
    pdf.bullet("Positions substituted at runtime from --init and --goal")

    # ── Save ──
    pdf.output(OUT_FILE)
    print(f"Generated: {OUT_FILE}")


if __name__ == "__main__":
    build_pdf()
