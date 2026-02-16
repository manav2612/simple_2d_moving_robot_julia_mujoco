#!/usr/bin/env python3
"""
Generate AIF_MuJoCoRobot_Math.pdf — Mathematical Reference
Covers every mathematical element in the AIF MuJoCo Robot project.
"""

import os
from fpdf import FPDF

FONT_DIR = "/usr/share/fonts/truetype/dejavu"
OUT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_FILE = os.path.join(OUT_DIR, "AIF_MuJoCoRobot_Math.pdf")

# ── Unicode helpers ──────────────────────────────────────────
MU = "\u03bc"
SIGMA = "\u03c3"
SIGMA_SQ = "\u03c3\u00b2"
GAMMA = "\u03b3"
BETA = "\u03b2"
TAU = "\u03c4"
PI = "\u03c0"
SUM = "\u2211"
SQRT = "\u221a"
ARROW = "\u2192"
IN = "\u2208"
LEQ = "\u2264"
GEQ = "\u2265"
CDOT = "\u00b7"
TIMES = "\u00d7"
SUB_I = "\u1d62"
SUP2 = "\u00b2"
APPROX = "\u2248"
INF = "\u221e"
FORALL = "\u2200"
NE = "\u2260"
PROP = "\u221d"


class MathPDF(FPDF):
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
        self.cell(0, 8, "AIF MuJoCo Robot \u2014 Mathematical Reference", align="C", new_x="LMARGIN", new_y="NEXT")
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
        self.set_text_color(30, 60, 120)
        self.multi_cell(0, 14, title, align="C")
        self.ln(6)
        self.set_font("DejaVu", "I", 14)
        self.set_text_color(80, 80, 80)
        self.multi_cell(0, 10, subtitle, align="C")
        self.ln(12)
        self.set_draw_color(30, 60, 120)
        self.set_line_width(0.8)
        self.line(50, self.get_y(), 160, self.get_y())
        self.ln(8)
        self.set_font("DejaVu", "", 10)
        self.set_text_color(100, 100, 100)
        self.multi_cell(0, 6, "Active Inference controller for a 3D MuJoCo robot.\nBeliefs, generative model, EFE, policy, and RxInfer factor graph.", align="C")
        self.set_text_color(0, 0, 0)
        self.set_line_width(0.2)

    def section(self, num, title):
        self.add_page()
        self.set_font("DejaVu", "B", 18)
        self.set_text_color(30, 60, 120)
        self.cell(0, 12, f"{num}.  {title}", new_x="LMARGIN", new_y="NEXT")
        self.set_draw_color(30, 60, 120)
        self.set_line_width(0.6)
        self.line(10, self.get_y(), 200, self.get_y())
        self.ln(6)
        self.set_text_color(0, 0, 0)
        self.set_line_width(0.2)

    def subsection(self, title):
        self.ln(4)
        self.set_font("DejaVu", "B", 13)
        self.set_text_color(50, 80, 140)
        self.cell(0, 9, title, new_x="LMARGIN", new_y="NEXT")
        self.ln(2)
        self.set_text_color(0, 0, 0)

    def body(self, text):
        self.set_font("DejaVu", "", 10)
        self.multi_cell(0, 6, text)
        self.ln(2)

    def body_italic(self, text):
        self.set_font("DejaVu", "I", 10)
        self.multi_cell(0, 6, text)
        self.ln(2)

    def equation(self, text):
        self.ln(3)
        self.set_font("Mono", "", 11)
        self.set_fill_color(240, 243, 250)
        self.set_draw_color(180, 190, 210)
        x = 20
        w = 170
        self.set_x(x)
        self.multi_cell(w, 7, f"  {text}", border=1, fill=True, align="L")
        self.ln(3)
        self.set_font("DejaVu", "", 10)

    def code_block(self, text):
        self.ln(2)
        self.set_font("Mono", "", 9)
        self.set_fill_color(245, 245, 245)
        self.set_draw_color(200, 200, 200)
        x = 15
        w = 180
        self.set_x(x)
        self.multi_cell(w, 5.5, text, border=1, fill=True, align="L")
        self.ln(2)
        self.set_font("DejaVu", "", 10)

    def bullet(self, text):
        self.set_font("DejaVu", "", 10)
        self.set_x(15)
        self.cell(5, 6, "\u2022")
        self.multi_cell(0, 6, text)

    def note_box(self, text):
        self.ln(2)
        self.set_fill_color(255, 250, 230)
        self.set_draw_color(200, 180, 100)
        self.set_font("DejaVu", "I", 9)
        self.set_x(15)
        self.multi_cell(180, 6, f"Note: {text}", border=1, fill=True)
        self.ln(2)
        self.set_font("DejaVu", "", 10)


def build_pdf():
    pdf = MathPDF()
    pdf.alias_nb_pages()

    # ── Title ──
    pdf.title_page(
        "AIF MuJoCo Robot",
        "Mathematical Reference"
    )

    # ══════════════════════════════════════════════════════════
    # Section 1: State-Space Model
    # ══════════════════════════════════════════════════════════
    pdf.section(1, "State-Space Model")
    pdf.body(
        "The robot operates in 3D Cartesian space (x, y, z). The state is the "
        "position vector s = [s_x, s_y, s_z]. The dynamics are additive: "
        "control u shifts the state directly."
    )
    pdf.subsection("Transition Model")
    pdf.body("Given state s and control u, the next state is:")
    pdf.equation(f"s' = s + u")
    pdf.body(
        f"With per-dimension process noise Q = diag(q_x, q_y, q_z), the "
        f"stochastic transition becomes:"
    )
    pdf.equation(f"s'{SUB_I} = s{SUB_I} + u{SUB_I} + {SIGMA}{SUB_I},   {SIGMA}{SUB_I} ~ N(0, q{SUB_I})")
    pdf.body(f"where i {IN} {{x, y, z}}.")

    pdf.subsection("Observation Model")
    pdf.body("Observations are noisy readings of the true state:")
    pdf.equation(f"y{SUB_I} ~ N(s{SUB_I}, r{SUB_I})")
    pdf.body(
        f"where r{SUB_I} is the observation noise variance for axis i. "
        f"The observation noise can be a scalar (same for all axes) or per-axis "
        f"[r_x, r_y, r_z]."
    )
    pdf.body_italic("Source: src/aif/generative_model.jl")

    # ══════════════════════════════════════════════════════════
    # Section 2: Belief State
    # ══════════════════════════════════════════════════════════
    pdf.section(2, "Belief State")
    pdf.body(
        "The agent maintains a Gaussian belief over its 3D position with a "
        "diagonal covariance matrix:"
    )
    pdf.equation(f"b(s) = N({MU}, diag({SIGMA}{SUP2}_x, {SIGMA}{SUP2}_y, {SIGMA}{SUP2}_z))")
    pdf.body(f"where {MU} = [{MU}_x, {MU}_y, {MU}_z] is the belief mean and "
             f"{SIGMA}{SUP2}{SUB_I} is the marginal variance for axis i.")

    pdf.subsection("Covariance Bounds")
    pdf.body("To prevent numerical instability:")
    pdf.equation(f"1{TIMES}10{chr(0x207B)}{chr(0x2075)} {LEQ} {SIGMA}{SUP2}{SUB_I} {LEQ} 2.0")
    pdf.body("All covariance updates are clamped to these bounds.")
    pdf.body_italic("Source: src/aif/beliefs.jl  |  COV_MIN = 1e-5, COV_MAX = 2.0")

    # ══════════════════════════════════════════════════════════
    # Section 3: Prediction Step
    # ══════════════════════════════════════════════════════════
    pdf.section(3, "Prediction Step")
    pdf.body(
        "Before observing, the belief is propagated forward using the applied "
        "control (the actual scaled control, not the raw action):"
    )
    pdf.subsection("Mean Prediction")
    pdf.equation(f"{MU}'{SUB_I} = {MU}{SUB_I} + ctrl{SUB_I}")

    pdf.subsection("Covariance Prediction")
    pdf.equation(f"{SIGMA}{SUP2}'{SUB_I} = clamp({SIGMA}{SUP2}{SUB_I} + q{SUB_I},  COV_MIN,  COV_MAX)")
    pdf.body(
        "Process noise q can be a scalar (applied equally to all axes) or a "
        "3-vector [q_x, q_y, q_z]. Default: q = 0.005."
    )
    pdf.note_box(
        "The prediction uses the actual applied control (ctrl = clamp(action "
        f"{TIMES} scale)), not the raw action, to match the true dynamics."
    )
    pdf.body_italic("Source: src/aif/beliefs.jl  |  predict_belief!()")

    # ══════════════════════════════════════════════════════════
    # Section 4: Bayesian Update
    # ══════════════════════════════════════════════════════════
    pdf.section(4, "Bayesian Update (Observation)")
    pdf.body(
        "When an observation y arrives, the belief is updated using "
        "precision-weighted fusion (conjugate Gaussian update), independently "
        "per axis:"
    )
    pdf.subsection("Precision Fusion")
    pdf.equation(f"{TAU}_prior{SUB_I} = 1 / {SIGMA}{SUP2}{SUB_I}")
    pdf.equation(f"{TAU}_lik{SUB_I}   = 1 / r{SUB_I}")
    pdf.equation(f"{TAU}_post{SUB_I}  = {TAU}_prior{SUB_I} + {TAU}_lik{SUB_I}")

    pdf.subsection("Posterior Covariance")
    pdf.equation(f"{SIGMA}{SUP2}_post{SUB_I} = clamp(1 / {TAU}_post{SUB_I},  COV_MIN,  COV_MAX)")

    pdf.subsection("Posterior Mean")
    pdf.equation(f"{MU}_post{SUB_I} = ({TAU}_prior{SUB_I} {CDOT} {MU}{SUB_I} + {TAU}_lik{SUB_I} {CDOT} y{SUB_I}) / {TAU}_post{SUB_I}")
    pdf.body(
        "This is the standard Kalman filter update for the diagonal case: "
        "the posterior mean is a precision-weighted average of the prior mean "
        "and the observation."
    )
    pdf.body_italic("Source: src/aif/beliefs.jl  |  update_belief!()")

    # ══════════════════════════════════════════════════════════
    # Section 5: Expected Free Energy (EFE)
    # ══════════════════════════════════════════════════════════
    pdf.section(5, "Expected Free Energy (EFE)")
    pdf.body(
        "Policy selection minimizes Expected Free Energy. EFE decomposes into "
        "a pragmatic (goal-seeking) term and an epistemic (exploration) term:"
    )
    pdf.equation(f"EFE(a) = G_pragmatic(a) \u2212 G_epistemic")

    pdf.subsection("Pragmatic Term (Goal-Seeking)")
    pdf.body(
        "Measures expected squared distance to goal after applying action a. "
        "The predicted position uses the actual control scaling:"
    )
    pdf.equation(f"pred{SUB_I} = {MU}{SUB_I} + clamp(a{SUB_I} {TIMES} ctrl_scale, \u2212ctrl_lim, ctrl_lim)")
    pdf.equation(f"G_pragmatic = {GAMMA} {CDOT} {SUM}{SUB_I} w{SUB_I} {CDOT} (pred{SUB_I} \u2212 goal{SUB_I}){SUP2}")
    pdf.body(
        f"where {GAMMA} is the pragmatic weight (default 1.2) and w{SUB_I} are "
        f"optional per-axis weights (default [1, 1, 1])."
    )

    pdf.subsection("Epistemic Term (Exploration)")
    pdf.body("Encourages uncertainty reduction based on belief entropy:")
    pdf.equation(f"G_epistemic = \u2212{BETA} {CDOT} {SUM}{SUB_I} log({SIGMA}{SUP2}{SUB_I})")
    pdf.body(
        f"where {BETA} is the epistemic weight (default 0.05). Covariance is "
        f"clamped to [1e-8, 100] for numerical stability in the log."
    )

    pdf.subsection("Combined EFE")
    pdf.equation(f"EFE(a) = {GAMMA} {SUM} w{SUB_I}(pred{SUB_I} \u2212 goal{SUB_I}){SUP2} + {BETA} {SUM} log({SIGMA}{SUP2}{SUB_I})")
    pdf.body("Lower EFE = better action (closer to goal + reduces uncertainty).")
    pdf.body_italic("Source: src/aif/efe.jl")

    # ══════════════════════════════════════════════════════════
    # Section 6: Policy Selection
    # ══════════════════════════════════════════════════════════
    pdf.section(6, "Policy Selection")
    pdf.body(
        "The agent selects the action that minimizes EFE over a discrete "
        "action set:"
    )
    pdf.equation(f"a* = argmin_{{a {IN} A}} EFE(a)")

    pdf.subsection("Action Set")
    pdf.body(
        f"A = 5{TIMES}5{TIMES}5 = 125 actions. Each axis has 5 velocity "
        f"levels: {{\u2212s, \u2212s/2, 0, s/2, s}} with step_size s = 0.08."
    )
    pdf.code_block(
        "A = { [dx, dy, dz] : dx, dy, dz \u2208 {-0.08, -0.04, 0, 0.04, 0.08} }"
    )
    pdf.body(
        "The finer grid (compared to a 3-action set) yields smoother trajectories "
        "and better convergence."
    )
    pdf.body_italic("Source: src/aif/policy.jl")

    # ══════════════════════════════════════════════════════════
    # Section 7: Action to Control
    # ══════════════════════════════════════════════════════════
    pdf.section(7, "Action to Control Mapping")
    pdf.body(
        "The raw action a is scaled and clamped to produce the MuJoCo control signal:"
    )
    pdf.equation(f"ctrl{SUB_I} = clamp(a{SUB_I} {TIMES} scale, \u2212ctrl_lim, ctrl_lim)")
    pdf.body(
        f"Default: scale = 4.0, ctrl_lim = 1.2. This prevents overshooting "
        f"while allowing sufficient movement per step."
    )
    pdf.body(
        "The control vector ctrl = [ctrl_x, ctrl_y, ctrl_z] is added to the "
        "current position in MuJoCo: target = pos + ctrl."
    )
    pdf.body_italic("Source: src/aif/action.jl")

    # ══════════════════════════════════════════════════════════
    # Section 8: RxInfer Factor Graph
    # ══════════════════════════════════════════════════════════
    pdf.section(8, "RxInfer Factor Graph")
    pdf.body(
        "An alternative inference backend expresses the same state-space model "
        "as a factor graph using RxInfer.jl. Each axis (x, y, z) runs an "
        "independent 1D linear-Gaussian model via reactive message passing."
    )

    pdf.subsection("Per-Axis Factor Graph")
    pdf.code_block(
        "x_prev ~ Normal(mean = m_prev, variance = v_prev)    [prior]\n"
        "x      ~ Normal(mean = x_prev + u, variance = q)     [transition]\n"
        "y      ~ Normal(mean = x, variance = r)               [observation]"
    )
    pdf.body(
        "where m_prev and v_prev are the posterior parameters from the "
        "previous timestep, u is the applied control, q is process noise "
        "variance, and r is observation noise variance."
    )

    pdf.subsection("Message Passing Schedule")
    pdf.body("At each timestep t:")
    pdf.bullet(f"Forward message: x_prev {ARROW} x carries N(m_prev + u, v_prev + q)")
    pdf.bullet(f"Likelihood message: y {ARROW} x carries N(y, r)")
    pdf.bullet(f"Posterior: q(x) = N({MU}_post, {SIGMA}{SUP2}_post) via precision fusion")

    pdf.subsection("Autoupdates (Prior Propagation)")
    pdf.body("After each inference cycle, the posterior becomes the next prior:")
    pdf.equation(f"m_prev, v_prev {ARROW} mean(q(x)), var(q(x))")
    pdf.body(
        "This is implemented with RxInfer's @autoupdates macro and a single "
        "atomic RecentSubject stream to avoid synchronization issues."
    )

    pdf.subsection("Equivalence to Analytic Update")
    pdf.body(
        "For the linear-Gaussian model, belief propagation is exact. The "
        "RxInfer posterior matches the closed-form Kalman update to machine "
        f"epsilon ({APPROX} 2.2{TIMES}10\u207B\u00b9\u2076). Both backends "
        "are interchangeable."
    )
    pdf.body_italic("Source: src/aif/rxinfer_filter.jl")

    # ══════════════════════════════════════════════════════════
    # Section 9: Utility Functions
    # ══════════════════════════════════════════════════════════
    pdf.section(9, "Utility Functions")

    pdf.subsection("Normalize")
    pdf.body("Normalizes a probability vector in place:")
    pdf.equation(f"p{SUB_I} {ARROW} p{SUB_I} / {SUM}_j p_j    (if {SUM} > 0, else uniform)")

    pdf.subsection("Gaussian PDF")
    pdf.body("Univariate:")
    pdf.equation(f"p(x | {MU}, {SIGMA}{SUP2}) = (1 / {SQRT}(2{PI}{SIGMA}{SUP2})) {CDOT} exp(\u2212(x\u2212{MU}){SUP2} / (2{SIGMA}{SUP2}))")
    pdf.body("Multivariate (diagonal covariance):")
    pdf.equation(f"p(x | {MU}, {SIGMA}{SUP2}) = {chr(0x220F)}{SUB_I} p(x{SUB_I} | {MU}{SUB_I}, {SIGMA}{SUP2}{SUB_I})")

    pdf.subsection("Gaussian Entropy")
    pdf.body("Entropy of a univariate Gaussian:")
    pdf.equation(f"H = 0.5 {CDOT} log(2{PI}e{SIGMA}{SUP2})")
    pdf.body("For the diagonal belief state, the total entropy is the sum over axes.")

    pdf.subsection("Softmax")
    pdf.body("Numerically stable softmax (subtract max for overflow protection):")
    pdf.equation(f"softmax(x){SUB_I} = exp(x{SUB_I} \u2212 max(x)) / {SUM}_j exp(x_j \u2212 max(x))")

    pdf.body_italic("Source: src/utils/math.jl")

    # ── Save ──
    pdf.output(OUT_FILE)
    print(f"Generated: {OUT_FILE}")


if __name__ == "__main__":
    build_pdf()
